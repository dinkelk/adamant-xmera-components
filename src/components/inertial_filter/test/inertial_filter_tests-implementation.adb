--------------------------------------------------------------------------------
-- Inertial_Filter Tests Body
--------------------------------------------------------------------------------

with Ada.Assertions;
with Ada.Numerics.Elementary_Functions;
with Ada.Real_Time;
use Ada.Numerics.Elementary_Functions;
with AUnit.Assertions;
with Basic_Assertions; use Basic_Assertions;
with Command;
with Command_Enums;
with Command_Response.Assertion; use Command_Response.Assertion;
with Inertial_Filter_Output;
with Inertial_Filter_Parameters;
with Interfaces; use Interfaces;
with Nav_Att_Output;
with Packed_F32x3;
with Packed_F64;
with Packed_F64x6;
with Packed_U16.Assertion; use Packed_U16.Assertion;
with Parameter;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;
with Sys_Time;

package body Inertial_Filter_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Test configuration:
   -------------------------------------------------------------------------

   -- Filter tuning from the algorithm's Python reference test
   -- (_tests/test_inertialFilter.py).
   Alpha : constant Packed_F64.T := (Value => 0.02);
   Beta : constant Packed_F64.T := (Value => 2.0);
   Zero_State : constant Packed_F64x6.T := [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
   Initial_Covariance : constant Packed_F64x6.T := [0.04, 0.04, 0.04, 0.004, 0.004, 0.004];
   Process_Noise : constant Packed_F64x6.T := [2.89E-6, 2.89E-6, 2.89E-6, 2.89E-8, 2.89E-8, 2.89E-8];
   St_Noise : constant Packed_F64.T := (Value => 0.00017);
   Rate_Noise : constant Packed_F64.T := (Value => 0.0017);

   Zero_Vector : constant Packed_F32x3.T := [0.0, 0.0, 0.0];

   -- The reference test steps the filter at half second intervals. A star tracker
   -- reading at step I is stamped I * Dt, and the tick that consumes it advances the
   -- filter to (I + 1) * Dt, so every reading lies in the past of the tick.
   Half_Second : constant Sys_Time.Subseconds_Type := Sys_Time.Subseconds_Type (Natural (Sys_Time.Subseconds_Type'Modulus) / 2);
   Half_Second_Ns : constant Unsigned_64 := 500_000_000;

   -------------------------------------------------------------------------
   -- Helpers:
   -------------------------------------------------------------------------

   -- Stage and apply the whole configuration.
   procedure Apply_Configuration (Self : in out Instance; Initial_State : in Packed_F64x6.T := Zero_State) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Inertial_Filter_Parameters.Instance;
   begin
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alpha (Alpha)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Beta (Beta)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Process_Noise_Diagonal (Process_Noise)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_State (Initial_State)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_Covariance_Diagonal (Initial_Covariance)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.St_Measurement_Noise_Std (St_Noise)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Rate_Measurement_Noise_Std (Rate_Noise)), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);
   end Apply_Configuration;

   -- The tick time for step I, (I + 1) half seconds.
   function Tick_Time (Step : in Natural) return Sys_Time.T is
      Half_Seconds : constant Natural := Step + 1;
   begin
      return (Seconds => Unsigned_32 (Half_Seconds / 2), Subseconds => (if Half_Seconds mod 2 = 1 then Half_Second else 0));
   end Tick_Time;

   -- Offer a star tracker reading stamped at step I and send the tick for step I. The
   -- time tag advances only when Fresh is set, so an unchanged reading is offered
   -- otherwise, as a product that has not been refreshed would be. The product itself
   -- is stamped with the tick time, as the producer earlier in the same tick would.
   -- The histories only hold the outputs of this tick, since the reference runs are
   -- far longer than the history depth.
   procedure Send_Tick (Self : in out Instance; Step : in Natural; Sigma_Bn : in Packed_F32x3.T; Omega_Bn_B : in Packed_F32x3.T; Fresh : in Boolean := True) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      if Fresh then
         T.Star_Tracker_Attitude := (Time_Tag => Unsigned_64 (Step) * Half_Second_Ns, Sigma_Bn => Sigma_Bn, Omega_Bn_B => Omega_Bn_B);
      end if;
      T.Data_Dependency_Timestamp_Override := Tick_Time (Step);
      T.Data_Product_Fetch_T_Service_History.Clear;
      T.Data_Product_T_Recv_Sync_History.Clear;
      T.Attitude_Estimate_History.Clear;
      T.Packet_T_Recv_Sync_History.Clear;
      T.Filter_Diagnostics_History.Clear;
      T.Sys_Time_T_Return_History.Clear;
      T.Tick_T_Send ((Time => Tick_Time (Step), Count => 0));
   end Send_Tick;

   -- The estimate published by the last tick.
   function Last_Estimate (Self : in Instance) return Nav_Att_Output.U is
      (Nav_Att_Output.Unpack (Self.Tester.Attitude_Estimate_History.Get (1)));

   -- The diagnostics packet sent by the last tick.
   function Diagnostics (Self : in Instance) return Inertial_Filter_Output.U is
      (Inertial_Filter_Output.Unpack (Self.Tester.Filter_Diagnostics_History.Get (1)));

   -- The diagonal entry I of the covariance in a diagnostics packet.
   function Variance (Output : in Inertial_Filter_Output.U; I : in Natural) return Long_Float is
      (Output.Covariance (I * Packed_F64x6.Length + I));

   -- Turn the diagnostics packet on at every tick, or off. The long runs keep it off
   -- except on the ticks whose snapshot is checked, since every packet is a large
   -- record to log on the target.
   procedure Set_Diagnostics (Self : in out Instance; On : in Boolean) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Command_T_Send (T.Commands.Set_Diagnostics_Packet_Period ((Value => (if On then 1 else 0))));
   end Set_Diagnostics;

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;

      -- The star tracker product starts out never refreshed:
      Self.Tester.Star_Tracker_Attitude := (Time_Tag => 0, Sigma_Bn => Zero_Vector, Omega_Bn_B => Zero_Vector);

      -- The product is produced every tick, so it gets no stale limit:
      Self.Tester.Component_Instance.Map_Data_Dependencies (Star_Tracker_Attitude_Id => 0, Star_Tracker_Attitude_Stale_Limit => Ada.Real_Time.Time_Span_Zero);

      -- Call component init here.
      Self.Tester.Component_Instance.Init;

      -- Call the component set up method that the assembly would normally call.
      Self.Tester.Component_Instance.Set_Up;
   end Set_Up_Test;

   overriding procedure Tear_Down_Test (Self : in out Instance) is
   begin
      -- Free the C++ algorithm heap:
      Self.Tester.Component_Instance.Destroy;
      -- Free component heap:
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Feed a constant star tracker attitude and check the estimate converges to it
   -- while the covariance shrinks, as in the algorithm's Python reference test, to
   -- ensure the Ada to C to C++ integration is sound.
   overriding procedure Test (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Attitude : constant Packed_F32x3.T := [0.3, 0.4, 0.5];
      Steps : constant := 400;
   begin
      Apply_Configuration (Self);
      Set_Diagnostics (Self, On => True);

      -- The reference test lets the filter propagate for the first steps before the
      -- star tracker delivers:
      Send_Tick (Self, 0, Zero_Vector, Zero_Vector, Fresh => False);
      declare
         Initial : constant Inertial_Filter_Output.U := Diagnostics (Self);
      begin
         Boolean_Assert.Eq (Initial.St_Att_Residuals.Valid, False);
         Set_Diagnostics (Self, On => False);
         for Step in 1 .. 21 loop
            Send_Tick (Self, Step, Zero_Vector, Zero_Vector, Fresh => False);
         end loop;
         for Step in 22 .. Steps - 2 loop
            Send_Tick (Self, Step, Attitude, Zero_Vector);
         end loop;
         Set_Diagnostics (Self, On => True);
         Send_Tick (Self, Steps - 1, Attitude, Zero_Vector);
         Natural_Assert.Eq (T.Attitude_Estimate_History.Get_Count, 1);
         Natural_Assert.Eq (T.Filter_Diagnostics_History.Get_Count, 1);

         -- The attitude covariance shrank, and the last reading was applied:
         for I in 0 .. 2 loop
            Long_Float_Assert.Lt (Variance (Diagnostics (Self), I), Variance (Initial, I));
         end loop;
         Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, True);
         Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, True);
      end;

      -- The estimate has converged to the measured attitude and rate:
      declare
         Estimate : constant Nav_Att_Output.U := Last_Estimate (Self);
      begin
         for I in 0 .. 2 loop
            Short_Float_Assert.Eq (Estimate.Sigma_Bn (I), Attitude (I), Epsilon => 1.0E-4);
            Short_Float_Assert.Eq (Estimate.Omega_Bn_B (I), 0.0, Epsilon => 1.0E-4);
            Short_Float_Assert.Eq (Estimate.Veh_Sun_Pnt_Bdy (I), 0.0);
         end loop;
         -- The estimate is stamped with the time the filter advanced to:
         Long_Float_Assert.Eq (Estimate.Time_Tag, Long_Float (Steps) * 0.5);
      end;

   end Test;

   -- Feed a constant body rate about one axis with the attitude that rotation gives,
   -- and check the rate estimate converges to it while the attitude tracks the truth.
   overriding procedure Test_Rate_Update (Self : in out Instance) is
      Rate : constant Short_Float := 0.01;
      Steps : constant := 400;
      -- A rotation about the first body axis at a constant rate has the MRP
      -- [tan (angle / 4), 0, 0], so the truth needs no integration.
      function Truth_Attitude (Step : in Natural) return Packed_F32x3.T is
         ([Short_Float (Tan (Float (Rate) * Float (Step) * 0.5 / 4.0)), 0.0, 0.0]);
   begin
      Apply_Configuration (Self);
      for Step in 1 .. Steps loop
         Send_Tick (Self, Step, Truth_Attitude (Step), [Rate, 0.0, 0.0]);
      end loop;

      -- The rate estimate converged to the measured rate, and the attitude tracks
      -- the truth at the time the filter advanced to:
      declare
         Estimate : constant Nav_Att_Output.U := Last_Estimate (Self);
         Truth : constant Packed_F32x3.T := Truth_Attitude (Steps + 1);
      begin
         Short_Float_Assert.Eq (Estimate.Omega_Bn_B (0), Rate, Epsilon => 1.0E-4);
         Short_Float_Assert.Eq (Estimate.Omega_Bn_B (1), 0.0, Epsilon => 1.0E-4);
         Short_Float_Assert.Eq (Estimate.Omega_Bn_B (2), 0.0, Epsilon => 1.0E-4);
         for I in 0 .. 2 loop
            Short_Float_Assert.Eq (Estimate.Sigma_Bn (I), Truth (I), Epsilon => 1.0E-3);
         end loop;
      end;
   end Test_Rate_Update;

   -- Ensure a star tracker product whose time tag has not advanced is not fed to the
   -- filter, so the estimate holds and the covariance grows under the process noise.
   overriding procedure Test_Propagation (Self : in out Instance) is
      Seed : constant Packed_F64x6.T := [0.1, 0.2, -0.1, 0.0, 0.0, 0.0];
      Steps : constant := 50;
   begin
      Apply_Configuration (Self, Initial_State => Seed);
      Set_Diagnostics (Self, On => True);

      -- The product is offered with a time tag of zero throughout, so no reading is
      -- ever fresh:
      Send_Tick (Self, 0, Zero_Vector, Zero_Vector, Fresh => False);
      declare
         Initial : constant Inertial_Filter_Output.U := Diagnostics (Self);
      begin
         Set_Diagnostics (Self, On => False);
         for Step in 1 .. Steps - 2 loop
            Send_Tick (Self, Step, Zero_Vector, Zero_Vector, Fresh => False);
         end loop;
         Set_Diagnostics (Self, On => True);
         Send_Tick (Self, Steps - 1, Zero_Vector, Zero_Vector, Fresh => False);

         -- With a zero rate the attitude stays at the seed:
         for I in 0 .. 2 loop
            Short_Float_Assert.Eq (Last_Estimate (Self).Sigma_Bn (I), Short_Float (Seed (I)), Epsilon => 1.0E-6);
            Short_Float_Assert.Eq (Last_Estimate (Self).Omega_Bn_B (I), 0.0, Epsilon => 1.0E-6);
         end loop;

         -- No measurement fired, and the covariance grew:
         for I in 0 .. 5 loop
            Long_Float_Assert.Gt (Variance (Diagnostics (Self), I), Variance (Initial, I));
         end loop;
         Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);
         Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);
      end;
   end Test_Propagation;

   -- Ensure the estimate reset re-seeds the state and covariance while the
   -- measurements reset keeps them.
   overriding procedure Test_Reset (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Attitude : constant Packed_F32x3.T := [0.3, 0.4, 0.5];
      Steps : constant := 200;
   begin
      Apply_Configuration (Self);
      for Step in 1 .. Steps loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
      end loop;
      Set_Diagnostics (Self, On => True);

      -- The measurements reset keeps the estimate and the covariance. The next tick
      -- has no fresh reading and nothing pending, so it only propagates, and the
      -- estimate stays converged:
      T.Reset_Measurements_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Send_Tick (Self, Steps + 1, Attitude, Zero_Vector, Fresh => False);
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Last_Estimate (Self).Sigma_Bn (I), Attitude (I), Epsilon => 1.0E-4);
         Long_Float_Assert.Lt (Variance (Diagnostics (Self), I), Initial_Covariance (I) / 10.0);
      end loop;
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);

      -- The estimate reset starts over from the configured seed and covariance:
      T.Reset_Estimate_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Send_Tick (Self, Steps + 2, Attitude, Zero_Vector, Fresh => False);
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Last_Estimate (Self).Sigma_Bn (I), 0.0, Epsilon => 1.0E-6);
         Long_Float_Assert.Eq (Variance (Diagnostics (Self), I), Initial_Covariance (I), Epsilon => 1.0E-3);
      end loop;
   end Test_Reset;

   -- A reset restarts the filter's time base at the next tick, and a reading from
   -- before that point cannot be placed on the new base, so it is dropped. The next
   -- reading after the restart is applied.
   overriding procedure Test_Reading_Before_Time_Base (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Attitude : constant Packed_F32x3.T := [0.3, 0.4, 0.5];
   begin
      Apply_Configuration (Self);
      Set_Diagnostics (Self, On => True);
      for Step in 1 .. 30 loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
      end loop;

      -- The reset tick restarts the time base at step 31. A reading newer than the
      -- last one consumed but stamped before that tick is offered with it:
      T.Reset_Measurements_Tick_T_Send ((Time => T.System_Time, Count => 0));
      T.Star_Tracker_Attitude := (Time_Tag => 30 * Half_Second_Ns + Half_Second_Ns / 2, Sigma_Bn => Attitude, Omega_Bn_B => Zero_Vector);
      Send_Tick (Self, 31, Attitude, Zero_Vector, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);

      -- Readings stamped before the restart tick, and one stamped at the restart tick
      -- itself, are not after it either:
      Send_Tick (Self, 31, Attitude, Zero_Vector);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);
      Send_Tick (Self, 32, Attitude, Zero_Vector);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);

      -- The next reading is applied:
      Send_Tick (Self, 33, Attitude, Zero_Vector);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, True);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, True);
   end Test_Reading_Before_Time_Base;

   -- A reading stamped after the tick cannot be a measurement yet, since the filter
   -- only advances to the tick, so it waits. A tick stamped before the time base is a
   -- clock that stepped back, which restarts the base. Neither shuts out the readings
   -- that follow.
   overriding procedure Test_Time_Anomalies (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Attitude : constant Packed_F32x3.T := [0.3, 0.4, 0.5];
   begin
      Apply_Configuration (Self);
      Set_Diagnostics (Self, On => True);
      for Step in 1 .. 5 loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
      end loop;

      -- A reading stamped well after the tick is held back, and the next reading in
      -- order is still applied:
      T.Star_Tracker_Attitude := (Time_Tag => 20 * Half_Second_Ns, Sigma_Bn => Attitude, Omega_Bn_B => Zero_Vector);
      Send_Tick (Self, 6, Attitude, Zero_Vector, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);
      Send_Tick (Self, 7, Attitude, Zero_Vector);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, True);

      -- A tick before the time base restarts it there. The reading still carries a
      -- later stamp, so it waits, and the first reading after the restart is applied:
      Send_Tick (Self, 0, Attitude, Zero_Vector, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, False);
      Send_Tick (Self, 2, Attitude, Zero_Vector);
      Boolean_Assert.Eq (Diagnostics (Self).St_Att_Residuals.Valid, True);
   end Test_Time_Anomalies;

   -- Ensure the diagnostics packet is sent every commanded number of ticks, that a
   -- period of zero turns it off, and that the command is reported.
   overriding procedure Test_Diagnostics_Packet (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Attitude : constant Packed_F32x3.T := [0.3, 0.4, 0.5];
   begin
      Apply_Configuration (Self);

      -- Off until commanded:
      for Step in 1 .. 3 loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
         Natural_Assert.Eq (T.Packet_T_Recv_Sync_History.Get_Count, 0);
      end loop;

      -- Every third tick once commanded, and the command is acknowledged and reported:
      T.Command_T_Send (T.Commands.Set_Diagnostics_Packet_Period ((Value => 3)));
      Natural_Assert.Eq (T.Command_Response_T_Recv_Sync_History.Get_Count, 1);
      Command_Response_Assert.Eq (T.Command_Response_T_Recv_Sync_History.Get (1), (
         Source_Id => 0,
         Registration_Id => 0,
         Command_Id => T.Commands.Get_Set_Diagnostics_Packet_Period_Id,
         Status => Command_Enums.Command_Response_Status.Success
      ));
      Natural_Assert.Eq (T.Diagnostics_Packet_Period_Set_History.Get_Count, 1);
      Packed_U16_Assert.Eq (T.Diagnostics_Packet_Period_Set_History.Get (1), (Value => 3));
      for Step in 4 .. 9 loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
         Natural_Assert.Eq (T.Packet_T_Recv_Sync_History.Get_Count, (if Step mod 3 = 0 then 1 else 0));
         Natural_Assert.Eq (T.Filter_Diagnostics_History.Get_Count, (if Step mod 3 = 0 then 1 else 0));
      end loop;

      -- The packet carries the snapshot the estimate was taken from:
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Short_Float (Diagnostics (Self).State (I)), Last_Estimate (Self).Sigma_Bn (I));
      end loop;

      -- A period of zero turns the packet off:
      T.Command_T_Send (T.Commands.Set_Diagnostics_Packet_Period ((Value => 0)));
      for Step in 10 .. 15 loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
         Natural_Assert.Eq (T.Packet_T_Recv_Sync_History.Get_Count, 0);
      end loop;
      Natural_Assert.Eq (T.Diagnostics_Packet_Period_Set_History.Get_Count, 2);
   end Test_Diagnostics_Packet;

   -- Ensure a parameter update rebuilds the filter from the new configuration,
   -- restarting the estimate from the configured seed.
   overriding procedure Test_Parameter_Update (Self : in out Instance) is
      Attitude : constant Packed_F32x3.T := [0.3, 0.4, 0.5];
      Seed : constant Packed_F64x6.T := [0.1, 0.0, 0.0, 0.0, 0.0, 0.0];
   begin
      Apply_Configuration (Self);
      for Step in 1 .. 100 loop
         Send_Tick (Self, Step, Attitude, Zero_Vector);
      end loop;
      Short_Float_Assert.Eq (Last_Estimate (Self).Sigma_Bn (0), Attitude (0), Epsilon => 1.0E-3);

      -- The update takes effect on the next tick, which has no fresh reading, so the
      -- estimate is the new seed:
      Apply_Configuration (Self, Initial_State => Seed);
      Send_Tick (Self, 101, Attitude, Zero_Vector, Fresh => False);
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Last_Estimate (Self).Sigma_Bn (I), Short_Float (Seed (I)), Epsilon => 1.0E-6);
      end loop;
   end Test_Parameter_Update;

   -- The algorithm requires alpha in (0, 1], beta in [0, 2], positive semi-definite
   -- noise and covariance, a finite initial state, and noise deviations that are not
   -- negative. Validation is the only guard keeping a rejected value out of the
   -- throwing Create, so exercise it directly.
   overriding procedure Test_Invalid_Parameter (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Inertial_Filter_Parameters.Instance;

      -- Stage a known-good set, so each rejection below is caused by the single
      -- perturbed value rather than by leftover staging state.
      procedure Stage_Valid_Configuration is
      begin
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alpha (Alpha)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Beta (Beta)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Process_Noise_Diagonal (Process_Noise)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_State (Zero_State)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_Covariance_Diagonal (Initial_Covariance)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.St_Measurement_Noise_Std (St_Noise)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Rate_Measurement_Noise_Std (Rate_Noise)), Success);
      end Stage_Valid_Configuration;

      -- Stage one perturbed parameter on top of the valid set and check the set is rejected.
      procedure Expect_Rejection (Par : in Parameter.T) is
      begin
         Stage_Valid_Configuration;
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Par), Success);
         Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);
      end Expect_Rejection;
   begin
      -- The reference configuration is accepted:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);

      Expect_Rejection (Params.Alpha ((Value => 0.0)));
      Expect_Rejection (Params.Beta ((Value => 3.0)));
      Expect_Rejection (Params.Process_Noise_Diagonal ([-1.0E-6, 2.89E-6, 2.89E-6, 2.89E-8, 2.89E-8, 2.89E-8]));
      Expect_Rejection (Params.Initial_Covariance_Diagonal ([0.04, 0.04, 0.04, 0.004, 0.004, -0.004]));
      Expect_Rejection (Params.St_Measurement_Noise_Std ((Value => -0.1)));
      Expect_Rejection (Params.Rate_Measurement_Noise_Std ((Value => -0.1)));

      -- A non-finite initial state is rejected. The value is injected as raw bytes because
      -- the compiler will not let a non-finite Long_Float be written as a literal, and
      -- because that is how one would arrive: as bytes from the ground. Staging accepts
      -- it, and converting it for the algorithm raises, which validation reports as a
      -- rejection.
      declare
         Par : Parameter.T := Params.Initial_State (Zero_State);
      begin
         -- Overwrite the first of the six big-endian doubles with +infinity.
         Par.Buffer (Par.Buffer'First .. Par.Buffer'First + 7) := [16#7F#, 16#F0#, 0, 0, 0, 0, 0, 0];
         Expect_Rejection (Par);
      end;

      -- Restoring the reference configuration makes the set acceptable again, so the
      -- rejections above were caused by the perturbed values:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);
   end Test_Invalid_Parameter;

   -- Ensure a malformed command is reported rather than acted on.
   overriding procedure Test_Invalid_Command (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      -- Build a valid command, then corrupt its argument buffer length so the framework
      -- rejects it and routes to the Invalid_Command handler:
      Invalid_Cmd : Command.T := T.Commands.Set_Diagnostics_Packet_Period ((Value => 1));
   begin
      Invalid_Cmd.Header.Arg_Buffer_Length := 0;
      T.Command_T_Send (Invalid_Cmd);

      -- The command is rejected with a length error and reported. The errant field of
      -- a length error is not meaningful, so only the count is checked:
      Natural_Assert.Eq (T.Command_Response_T_Recv_Sync_History.Get_Count, 1);
      Command_Response_Assert.Eq (T.Command_Response_T_Recv_Sync_History.Get (1), (
         Source_Id => 0,
         Registration_Id => 0,
         Command_Id => T.Commands.Get_Set_Diagnostics_Packet_Period_Id,
         Status => Command_Enums.Command_Response_Status.Length_Error
      ));
      Natural_Assert.Eq (T.Event_T_Recv_Sync_History.Get_Count, 1);
      Natural_Assert.Eq (T.Invalid_Command_Received_History.Get_Count, 1);
      Natural_Assert.Eq (T.Diagnostics_Packet_Period_Set_History.Get_Count, 0);
   end Test_Invalid_Command;

   -- A data dependency that comes back with the wrong identifier means the assembly
   -- is wired incorrectly. The component asserts rather than publishing anything.
   overriding procedure Test_Invalid_Data_Dependency (Self : in out Instance) is
      T : Component.Inertial_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Data_Dependency_Return_Id_Override := 999;
      begin
         T.Tick_T_Send ((Time => T.System_Time, Count => 0));
         AUnit.Assertions.Assert (False, "A dependency with the wrong identifier should have failed an assertion.");
      exception
         when Ada.Assertions.Assertion_Error =>
            null; -- Expected.
      end;
      Natural_Assert.Eq (T.Attitude_Estimate_History.Get_Count, 0);
   end Test_Invalid_Data_Dependency;

end Inertial_Filter_Tests.Implementation;
