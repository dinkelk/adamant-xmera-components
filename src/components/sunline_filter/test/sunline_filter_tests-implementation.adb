--------------------------------------------------------------------------------
-- Sunline_Filter Tests Body
--------------------------------------------------------------------------------

with Ada.Assertions;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Real_Time;
with AUnit.Assertions;
with Basic_Assertions; use Basic_Assertions;
with Command;
with Command_Enums;
with Command_Response.Assertion; use Command_Response.Assertion;
with Css_Sensor_Values;
with Interfaces; use Interfaces;
with Nav_Att_Output;
with Packed_F32x24;
with Packed_F32x3;
with Packed_F64;
with Packed_F64x7;
with Packed_F64x8;
with Packed_U16.Assertion; use Packed_U16.Assertion;
with Packed_U32;
with Parameter;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;
with Sunline_Filter_Output;
with Sunline_Filter_Parameters;
with Sys_Time;

package body Sunline_Filter_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Test configuration:
   -------------------------------------------------------------------------

   -- Filter tuning and sensor geometry from the algorithm's Python reference test
   -- (_tests/test_sunlineFilter.py).
   Alpha : constant Packed_F64.T := (Value => 0.02);
   Beta : constant Packed_F64.T := (Value => 2.0);
   Initial_Covariance : constant Packed_F64x7.T := [1.0E-4, 1.0E-4, 1.0E-4, 1.0E-4, 1.0E-4, 1.0E-4, 1.0];
   Process_Noise : constant Packed_F64x7.T := [1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10];
   Bias_Lower_Bound : constant Packed_F64.T := (Value => 0.5);
   Bias_Upper_Bound : constant Packed_F64.T := (Value => 1.5);
   Css_N_Hat_B : constant Packed_F32x24.T :=
      [+0.70710678, -0.50000000, +0.50000000,
       +0.70710678, -0.50000000, -0.50000000,
       +0.70710678, +0.50000000, -0.50000000,
       +0.70710678, +0.50000000, +0.50000000,
       -0.70710678, +0.00000000, +0.70710678,
       -0.70710678, +0.70710678, +0.00000000,
       -0.70710678, +0.00000000, -0.70710678,
       -0.70710678, -0.70710678, +0.00000000];
   Css_Scale_Factor : constant Packed_F64x8.T := [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
   Number_Of_Css : constant Packed_U32.T := (Value => 8);
   Sensor_Threshold : constant Packed_F64.T := (Value => 0.0);
   Css_Noise : constant Packed_F64.T := (Value => 1.0E-4);
   Gyro_Noise : constant Packed_F64.T := (Value => 1.0E-5);

   -- The truth: the body rotates about its third axis at a constant rate, so the sun,
   -- along the first body axis at the start, turns in the first two body axes. The
   -- sensor intensity bias is one.
   Rate : constant Long_Float := 0.01;
   Truth_Rate : constant Packed_F32x3.T := [0.0, 0.0, Short_Float (Rate)];
   Truth_Bias : constant Long_Float := 1.0;
   -- The seed matches the truth at time zero.
   Seed : constant Packed_F64x7.T := [1.0, 0.0, 0.0, 0.0, 0.0, Rate, Truth_Bias];

   -- The reference test steps the filter at one second intervals. The products
   -- consumed at step I are stamped with the tick time of step I.
   Dt : constant := 1.0;

   -------------------------------------------------------------------------
   -- Helpers:
   -------------------------------------------------------------------------

   -- The sun direction in the body frame at step I.
   function Truth_Sun (Step : in Natural) return Packed_F32x3.T is
      Angle : constant Long_Float := Rate * Long_Float (Step) * Dt;
   begin
      return [Short_Float (Cos (Angle)), Short_Float (-Sin (Angle)), 0.0];
   end Truth_Sun;

   -- The cosine each sensor reads for the sun direction at step I, scaled by the bias.
   -- The packed tables are unpacked first, so each element is read whole.
   function Truth_Cosines (Step : in Natural) return Css_Sensor_Values.T is
      Sun : constant Packed_F32x3.U := Packed_F32x3.Unpack (Truth_Sun (Step));
      N_Hat : constant Packed_F32x24.U := Packed_F32x24.Unpack (Css_N_Hat_B);
      Cosines : Packed_F64x8.U;
   begin
      for Sensor in 0 .. 7 loop
         Cosines (Sensor) := Truth_Bias * (Long_Float (N_Hat (Sensor * 3)) * Long_Float (Sun (0))
            + Long_Float (N_Hat (Sensor * 3 + 1)) * Long_Float (Sun (1))
            + Long_Float (N_Hat (Sensor * 3 + 2)) * Long_Float (Sun (2)));
      end loop;
      return (Data => Packed_F64x8.Pack (Cosines));
   end Truth_Cosines;

   -- Stage and apply the whole configuration.
   procedure Apply_Configuration (Self : in out Instance; Initial_State : in Packed_F64x7.T := Seed; Gyro_Noise_Std : in Packed_F64.T := Gyro_Noise; Sensors : in Packed_U32.T := Number_Of_Css; Threshold : in Packed_F64.T := Sensor_Threshold) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Sunline_Filter_Parameters.Instance;
   begin
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alpha (Alpha)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Beta (Beta)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Process_Noise_Diagonal (Process_Noise)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_State (Initial_State)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_Covariance_Diagonal (Initial_Covariance)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Bias_Lower_Bound (Bias_Lower_Bound)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Bias_Upper_Bound (Bias_Upper_Bound)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Css_N_Hat_B (Css_N_Hat_B)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Css_Scale_Factor (Css_Scale_Factor)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Number_Of_Css (Sensors)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Sensor_Threshold (Threshold)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Css_Measurement_Noise_Std (Css_Noise)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Gyro_Measurement_Noise_Std (Gyro_Noise_Std)), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);
   end Apply_Configuration;

   -- The tick time for step I.
   function Tick_Time (Step : in Natural) return Sys_Time.T is
      ((Seconds => Unsigned_32 (Step), Subseconds => 0));

   -- Offer the truth readings for step I and send its tick. The products are stamped
   -- with the tick time when Fresh is set, as the producers earlier in the same tick
   -- would stamp them, and are left at the previous stamp otherwise. The histories
   -- only hold the outputs of this tick, since the reference runs are far longer than
   -- the history depth.
   procedure Send_Tick (Self : in out Instance; Step : in Natural; Fresh : in Boolean := True) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      if Fresh then
         T.Body_Rate := Truth_Rate;
         T.Css_Cosines := Truth_Cosines (Step);
         T.Data_Dependency_Timestamp_Override := Tick_Time (Step);
      end if;
      T.Data_Product_Fetch_T_Service_History.Clear;
      T.Data_Product_T_Recv_Sync_History.Clear;
      T.Sun_Direction_Estimate_History.Clear;
      T.Packet_T_Recv_Sync_History.Clear;
      T.Filter_Diagnostics_History.Clear;
      T.Sys_Time_T_Return_History.Clear;
      T.Tick_T_Send ((Time => Tick_Time (Step), Count => 0));
   end Send_Tick;

   -- The estimate published by the last tick.
   function Last_Estimate (Self : in Instance) return Nav_Att_Output.U is
      (Nav_Att_Output.Unpack (Self.Tester.Sun_Direction_Estimate_History.Get (1)));

   -- The diagnostics packet sent by the last tick.
   function Diagnostics (Self : in Instance) return Sunline_Filter_Output.U is
      (Sunline_Filter_Output.Unpack (Self.Tester.Filter_Diagnostics_History.Get (1)));

   -- The diagonal entry I of the covariance in a diagnostics packet.
   function Variance (Output : in Sunline_Filter_Output.U; I : in Natural) return Long_Float is
      (Output.Filter_State.Covariance (I * Packed_F64x7.Length + I));

   -- Turn the diagnostics packet on at every tick, or off. The long runs keep it off
   -- except on the ticks whose snapshot is checked, since every packet is a large
   -- record to log on the target.
   procedure Set_Diagnostics (Self : in out Instance; On : in Boolean) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Command_T_Send (T.Commands.Set_Diagnostics_Packet_Period ((Value => (if On then 1 else 0))));
   end Set_Diagnostics;

   -- Check the last estimate against the truth at step I.
   procedure Check_Estimate (Self : in Instance; Step : in Natural; Sun_Epsilon : in Short_Float; Rate_Epsilon : in Short_Float) is
      Estimate : constant Nav_Att_Output.U := Last_Estimate (Self);
      Sun : constant Packed_F32x3.T := Truth_Sun (Step);
   begin
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Estimate.Veh_Sun_Pnt_Bdy (I), Sun (I), Epsilon => Sun_Epsilon);
         Short_Float_Assert.Eq (Estimate.Omega_Bn_B (I), Truth_Rate (I), Epsilon => Rate_Epsilon);
         Short_Float_Assert.Eq (Estimate.Sigma_Bn (I), 0.0);
      end loop;
      Long_Float_Assert.Eq (Estimate.Time_Tag, Long_Float (Step) * Dt);
   end Check_Estimate;

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;

      -- The products start out with a zero reading, stamped before the filter's time
      -- base, so nothing is fresh until a tick offers a reading:
      Self.Tester.Body_Rate := [0.0, 0.0, 0.0];
      Self.Tester.Css_Cosines := (Data => [others => 0.0]);
      Self.Tester.Data_Dependency_Timestamp_Override := (Seconds => 0, Subseconds => 1);

      -- The products are produced every tick, so they get no stale limit:
      Self.Tester.Component_Instance.Map_Data_Dependencies (
         Body_Rate_Id => 0, Body_Rate_Stale_Limit => Ada.Real_Time.Time_Span_Zero,
         Css_Cosines_Id => 1, Css_Cosines_Stale_Limit => Ada.Real_Time.Time_Span_Zero);

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

   -- Feed the sun sensor cosines and body rate of a slow rotation and check the sun
   -- direction, rate, and bias estimates converge to the truth while the covariance
   -- shrinks, as in the algorithm's Python reference test, to ensure the Ada to C to
   -- C++ integration is sound.
   overriding procedure Test (Self : in out Instance) is
      -- Start away from the truth, as the reference test's initial error case does.
      Wrong_Seed : constant Packed_F64x7.T := [0.0, 0.0, 1.0, -0.02, 0.005, -0.01, 0.6];
      Steps : constant := 400;
   begin
      Apply_Configuration (Self, Initial_State => Wrong_Seed);
      Set_Diagnostics (Self, On => True);

      Send_Tick (Self, 1);
      declare
         Initial : constant Sunline_Filter_Output.U := Diagnostics (Self);
      begin
         Set_Diagnostics (Self, On => False);
         for Step in 2 .. Steps - 1 loop
            Send_Tick (Self, Step);
         end loop;
         Set_Diagnostics (Self, On => True);
         Send_Tick (Self, Steps);

         -- The estimate has converged to the truth, and the bias with it:
         Check_Estimate (Self, Steps, Sun_Epsilon => 1.0E-3, Rate_Epsilon => 1.0E-4);
         Long_Float_Assert.Eq (Diagnostics (Self).Filter_State.State (6), Truth_Bias, Epsilon => 0.05);

         -- The covariance shrank, and both readings were applied:
         for I in 0 .. 6 loop
            Long_Float_Assert.Lt (Variance (Diagnostics (Self), I), Variance (Initial, I));
         end loop;
         Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, True);
         Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, True);
         Natural_Assert.Gt (Natural (Diagnostics (Self).Css_Residuals.Number_Of_Active_Css), 0);
      end;
   end Test;

   -- Ensure products whose timestamps have not advanced are not fed to the filter,
   -- so the sun direction is propagated with the seeded rate and the covariance
   -- grows under the process noise.
   overriding procedure Test_Propagation (Self : in out Instance) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      -- Each tick propagates from the anchor over the whole time since the base
      -- restarted, so the run is kept short.
      Steps : constant := 20;
   begin
      -- The parameters reach the filter on the next tick, and the seed takes effect
      -- through the estimate reset, whose next tick restarts the time base at the
      -- seed. The products keep their stamp from before the time base, so nothing is
      -- ever fresh:
      Apply_Configuration (Self);
      Send_Tick (Self, 1, Fresh => False);
      T.Reset_Estimate_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Set_Diagnostics (Self, On => True);
      Send_Tick (Self, 2, Fresh => False);
      declare
         Initial : constant Sunline_Filter_Output.U := Diagnostics (Self);
      begin
         Set_Diagnostics (Self, On => False);
         for Step in 3 .. Steps + 1 loop
            Send_Tick (Self, Step, Fresh => False);
         end loop;
         Set_Diagnostics (Self, On => True);
         Send_Tick (Self, Steps + 2, Fresh => False);

         -- The seeded rate turns the sun direction along the truth for the time since
         -- the time base restarted:
         declare
            Estimate : constant Nav_Att_Output.U := Last_Estimate (Self);
            Sun : constant Packed_F32x3.T := Truth_Sun (Steps);
         begin
            for I in 0 .. 2 loop
               Short_Float_Assert.Eq (Estimate.Veh_Sun_Pnt_Bdy (I), Sun (I), Epsilon => 1.0E-5);
               Short_Float_Assert.Eq (Estimate.Omega_Bn_B (I), Truth_Rate (I), Epsilon => 1.0E-6);
            end loop;
         end;

         -- No measurement fired, and the covariance grew:
         for I in 0 .. 6 loop
            Long_Float_Assert.Gt (Variance (Diagnostics (Self), I), Variance (Initial, I));
         end loop;
         Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, False);
         Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);
      end;
   end Test_Propagation;

   -- A reset restarts the filter's time base at the next tick, and products from
   -- before that point cannot be placed on the new base, so they are dropped. The
   -- next products after the restart are applied.
   overriding procedure Test_Reading_Before_Time_Base (Self : in out Instance) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      Apply_Configuration (Self);
      Set_Diagnostics (Self, On => True);
      for Step in 1 .. 30 loop
         Send_Tick (Self, Step);
      end loop;

      -- The reset tick restarts the time base at step 31. Products newer than the
      -- last ones consumed but stamped before that tick are offered with it:
      T.Reset_Measurements_Tick_T_Send ((Time => T.System_Time, Count => 0));
      T.Data_Dependency_Timestamp_Override := (Seconds => 30, Subseconds => 1);
      Send_Tick (Self, 31, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, False);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);

      -- Products stamped at the restart tick itself are not after it either:
      Send_Tick (Self, 31);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, False);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);

      -- The next products are applied:
      Send_Tick (Self, 32);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, True);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, True);
   end Test_Reading_Before_Time_Base;

   -- A product stamped after the tick cannot be a measurement yet, since the filter
   -- only advances to the tick, so it waits. A tick stamped before the time base is a
   -- clock that stepped back, which restarts the base. Neither shuts out the readings
   -- that follow.
   overriding procedure Test_Time_Anomalies (Self : in out Instance) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      Apply_Configuration (Self);
      Set_Diagnostics (Self, On => True);
      for Step in 1 .. 5 loop
         Send_Tick (Self, Step);
      end loop;

      -- Products stamped one second after the tick are held back, and applied once
      -- the tick reaches them:
      T.Body_Rate := Truth_Rate;
      T.Css_Cosines := Truth_Cosines (7);
      T.Data_Dependency_Timestamp_Override := Tick_Time (7);
      Send_Tick (Self, 6, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, False);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);
      Send_Tick (Self, 7, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, True);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, True);

      -- A tick before the time base restarts it there. The products still carry the
      -- later stamp, so they wait, and the ones stamped at the next tick are applied:
      Send_Tick (Self, 0, Fresh => False);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, False);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, False);
      Send_Tick (Self, 1);
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, True);
      Boolean_Assert.Eq (Diagnostics (Self).Rate_Residuals.Valid, True);
   end Test_Time_Anomalies;

   -- Only the first configured number of sensor slots is read, and of those only the
   -- ones reading above the threshold count as active. With the sun near the first
   -- body axis, the four sensors on that side each read about 0.7.
   overriding procedure Test_Fewer_Sensors (Self : in out Instance) is
      Steps : constant := 10;
   begin
      Apply_Configuration (Self, Sensors => (Value => 4), Threshold => (Value => 0.5));
      Set_Diagnostics (Self, On => True);
      for Step in 1 .. Steps loop
         Send_Tick (Self, Step);
      end loop;
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, True);
      Natural_Assert.Eq (Natural (Diagnostics (Self).Css_Residuals.Number_Of_Active_Css), 4);
      Check_Estimate (Self, Steps, Sun_Epsilon => 1.0E-2, Rate_Epsilon => 1.0E-3);

      -- A threshold above their reading leaves no sensor active:
      Apply_Configuration (Self, Sensors => (Value => 4), Threshold => (Value => 0.8));
      Send_Tick (Self, Steps + 1);
      Natural_Assert.Eq (Natural (Diagnostics (Self).Css_Residuals.Number_Of_Active_Css), 0);
   end Test_Fewer_Sensors;

   -- Ensure the estimate reset re-seeds the state and covariance while the
   -- measurements reset keeps them.
   overriding procedure Test_Reset (Self : in out Instance) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Wrong_Seed : constant Packed_F64x7.T := [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.6];
      Steps : constant := 200;
   begin
      Apply_Configuration (Self, Initial_State => Wrong_Seed);
      for Step in 1 .. Steps loop
         Send_Tick (Self, Step);
      end loop;
      Check_Estimate (Self, Steps, Sun_Epsilon => 1.0E-3, Rate_Epsilon => 1.0E-4);
      Set_Diagnostics (Self, On => True);

      -- The measurements reset keeps the estimate and the covariance. The next tick
      -- restarts the filter's time base, so it neither applies a reading nor
      -- propagates, and the estimate is the converged one:
      T.Reset_Measurements_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Send_Tick (Self, Steps + 1, Fresh => False);
      declare
         Estimate : constant Nav_Att_Output.U := Last_Estimate (Self);
         Sun : constant Packed_F32x3.T := Truth_Sun (Steps);
      begin
         for I in 0 .. 2 loop
            Short_Float_Assert.Eq (Estimate.Veh_Sun_Pnt_Bdy (I), Sun (I), Epsilon => 1.0E-3);
            Short_Float_Assert.Eq (Estimate.Omega_Bn_B (I), Truth_Rate (I), Epsilon => 1.0E-4);
         end loop;
      end;
      for I in 0 .. 2 loop
         Long_Float_Assert.Lt (Variance (Diagnostics (Self), I), Initial_Covariance (I) / 10.0);
      end loop;
      Boolean_Assert.Eq (Diagnostics (Self).Css_Residuals.Valid, False);

      -- The estimate reset starts over from the configured seed and covariance:
      T.Reset_Estimate_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Send_Tick (Self, Steps + 2, Fresh => False);
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Last_Estimate (Self).Veh_Sun_Pnt_Bdy (I), Short_Float (Wrong_Seed (I)), Epsilon => 1.0E-6);
         Long_Float_Assert.Eq (Variance (Diagnostics (Self), I), Initial_Covariance (I), Epsilon => 1.0E-5);
      end loop;
   end Test_Reset;

   -- Ensure the diagnostics packet is sent every commanded number of ticks, that a
   -- period of zero turns it off, and that the command is reported.
   overriding procedure Test_Diagnostics_Packet (Self : in out Instance) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      Apply_Configuration (Self);

      -- Off until commanded:
      for Step in 1 .. 3 loop
         Send_Tick (Self, Step);
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
         Send_Tick (Self, Step);
         Natural_Assert.Eq (T.Packet_T_Recv_Sync_History.Get_Count, (if Step mod 3 = 0 then 1 else 0));
         Natural_Assert.Eq (T.Filter_Diagnostics_History.Get_Count, (if Step mod 3 = 0 then 1 else 0));
      end loop;

      -- The packet carries the snapshot the estimate was taken from:
      for I in 0 .. 2 loop
         Short_Float_Assert.Eq (Short_Float (Diagnostics (Self).Filter_State.State (I)), Last_Estimate (Self).Veh_Sun_Pnt_Bdy (I));
      end loop;

      -- A period of zero turns the packet off:
      T.Command_T_Send (T.Commands.Set_Diagnostics_Packet_Period ((Value => 0)));
      for Step in 10 .. 15 loop
         Send_Tick (Self, Step);
         Natural_Assert.Eq (T.Packet_T_Recv_Sync_History.Get_Count, 0);
      end loop;
      Natural_Assert.Eq (T.Diagnostics_Packet_Period_Set_History.Get_Count, 2);
   end Test_Diagnostics_Packet;

   -- Ensure a parameter update reaches the filter while keeping the current estimate.
   overriding procedure Test_Parameter_Update (Self : in out Instance) is
      Wrong_Seed : constant Packed_F64x7.T := [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.6];
   begin
      Apply_Configuration (Self, Initial_State => Wrong_Seed);
      for Step in 1 .. 100 loop
         Send_Tick (Self, Step);
      end loop;
      Check_Estimate (Self, 100, Sun_Epsilon => 1.0E-2, Rate_Epsilon => 1.0E-3);

      -- A new gyro noise reaches the filter on the next tick without disturbing the
      -- estimate, which keeps tracking:
      Apply_Configuration (Self, Initial_State => Wrong_Seed, Gyro_Noise_Std => (Value => 1.0E-4));
      Send_Tick (Self, 101);
      Check_Estimate (Self, 101, Sun_Epsilon => 1.0E-2, Rate_Epsilon => 1.0E-3);
   end Test_Parameter_Update;

   -- The algorithm requires alpha in (0, 1], beta in [0, 2], positive semi-definite
   -- noise and covariance, a finite initial state, ordered positive bias bounds, a
   -- sensor count in [1, 8], unit boresights, and scale factors, a threshold, and noise
   -- deviations that are not negative. Validation is the only guard keeping a rejected
   -- value out of the throwing Create and Set_Config, so exercise it directly.
   overriding procedure Test_Invalid_Parameter (Self : in out Instance) is
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Sunline_Filter_Parameters.Instance;

      -- Stage a known-good set, so each rejection below is caused by the single
      -- perturbed value rather than by leftover staging state.
      procedure Stage_Valid_Configuration is
      begin
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alpha (Alpha)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Beta (Beta)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Process_Noise_Diagonal (Process_Noise)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_State (Seed)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Initial_Covariance_Diagonal (Initial_Covariance)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Bias_Lower_Bound (Bias_Lower_Bound)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Bias_Upper_Bound (Bias_Upper_Bound)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Css_N_Hat_B (Css_N_Hat_B)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Css_Scale_Factor (Css_Scale_Factor)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Number_Of_Css (Number_Of_Css)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Sensor_Threshold (Sensor_Threshold)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Css_Measurement_Noise_Std (Css_Noise)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Gyro_Measurement_Noise_Std (Gyro_Noise)), Success);
      end Stage_Valid_Configuration;

      -- Stage one perturbed parameter on top of the valid set and check the set is rejected.
      procedure Expect_Rejection (Par : in Parameter.T) is
      begin
         Stage_Valid_Configuration;
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Par), Success);
         Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);
      end Expect_Rejection;

      Not_Unit : Packed_F32x24.T := Css_N_Hat_B;
   begin
      -- The reference configuration is accepted:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);

      Expect_Rejection (Params.Alpha ((Value => 0.0)));
      Expect_Rejection (Params.Beta ((Value => 3.0)));
      Expect_Rejection (Params.Process_Noise_Diagonal ([-1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10, 1.0E-10]));
      Expect_Rejection (Params.Initial_Covariance_Diagonal ([1.0E-4, 1.0E-4, 1.0E-4, 1.0E-4, 1.0E-4, 1.0E-4, -1.0]));
      Expect_Rejection (Params.Bias_Lower_Bound ((Value => 0.0)));
      Expect_Rejection (Params.Bias_Upper_Bound ((Value => 0.4)));
      Not_Unit (0) := 2.0;
      Expect_Rejection (Params.Css_N_Hat_B (Not_Unit));
      Expect_Rejection (Params.Css_Scale_Factor ([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0]));
      Expect_Rejection (Params.Number_Of_Css ((Value => 0)));
      Expect_Rejection (Params.Number_Of_Css ((Value => 9)));
      Expect_Rejection (Params.Sensor_Threshold ((Value => -0.1)));
      Expect_Rejection (Params.Css_Measurement_Noise_Std ((Value => -0.1)));
      Expect_Rejection (Params.Gyro_Measurement_Noise_Std ((Value => -0.1)));

      -- A non-finite initial state is rejected. The value is injected as raw bytes because
      -- the compiler will not let a non-finite Long_Float be written as a literal, and
      -- because that is how one would arrive: as bytes from the ground. Staging accepts
      -- it, and converting it for the algorithm raises, which validation reports as a
      -- rejection.
      declare
         Par : Parameter.T := Params.Initial_State (Seed);
      begin
         -- Overwrite the first of the seven big-endian doubles with +infinity.
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
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
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
      T : Component.Sunline_Filter.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Data_Dependency_Return_Id_Override := 999;
      begin
         T.Tick_T_Send ((Time => T.System_Time, Count => 0));
         AUnit.Assertions.Assert (False, "A dependency with the wrong identifier should have failed an assertion.");
      exception
         when Ada.Assertions.Assertion_Error =>
            null; -- Expected.
      end;
      Natural_Assert.Eq (T.Sun_Direction_Estimate_History.Get_Count, 0);
   end Test_Invalid_Data_Dependency;

end Sunline_Filter_Tests.Implementation;
