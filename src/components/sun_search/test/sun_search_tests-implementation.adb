--------------------------------------------------------------------------------
-- Sun_Search Tests Body
--------------------------------------------------------------------------------

with Basic_Assertions; use Basic_Assertions;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with Packed_F32x9;
with Slew_Properties;
with Att_Guid;
with Sun_Search_Parameters;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;
with Ada.Real_Time;
with Interfaces;
with Sys_Time;

package body Sun_Search_Tests.Implementation is

   package Short_Float_Math is new Ada.Numerics.Generic_Elementary_Functions (Short_Float);
   use Short_Float_Math;

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;

      -- Component Init will be called manually in test body

      -- Call the component set up method that the assembly would normally call.
      Self.Tester.Component_Instance.Set_Up;
   end Set_Up_Test;

   overriding procedure Tear_Down_Test (Self : in out Instance) is
   begin
      -- Free component heap:
      Self.Tester.Component_Instance.Destroy;
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Helper function to compute kinematic properties (matches Python test)
   procedure Compute_Kinematic_Properties
     (Theta_R      : Short_Float;
      T_R          : Short_Float;
      U_M          : Short_Float;
      I            : Short_Float;
      Omega_M      : Short_Float;
      Alpha        : out Short_Float;
      Omega        : out Short_Float;
      T_Total      : out Short_Float;
      T_C          : out Short_Float)
   is
      Alpha_M : constant Short_Float := U_M / I;
   begin
      -- Computing the fastest bang-bang slew with no coasting arc
      Alpha := 4.0 * Theta_R / (T_R ** 2);
      Omega := 2.0 * Theta_R / T_R;
      T_Total := T_R;
      T_C := T_R / 2.0;

      -- If angular acceleration exceeds limit, decrease acceleration and increase slew time
      if Alpha > Alpha_M then
         Alpha := Alpha_M;
         T_Total := 2.0 * Sqrt (Theta_R / Alpha);
         T_C := T_Total / 2.0;
         Omega := Alpha * T_C;
      end if;

      -- If angular rate exceeds limit, increase slew time adding a coasting arc
      if Omega > Omega_M then
         Omega := Omega_M;
         T_Total := Theta_R / Omega + Omega / Alpha;
         T_C := Omega / Alpha;
      end if;
   end Compute_Kinematic_Properties;

   -- Run algorithm to ensure integration is sound.
   overriding procedure Test (Self : in out Instance) is
      use Interfaces;

      T : Component.Sun_Search.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Sun_Search_Parameters.Instance;

      -- Test data from Python test
      -- Inertia matrix (diagonal elements): [100, 200, 300]
      Inertia : constant Packed_F32x9.T := [100.0, 0.0, 0.0,
                                             0.0, 200.0, 0.0,
                                             0.0, 0.0, 300.0];

      -- Slew properties matching Python test
      Pi : constant Short_Float := 3.141592653589793;
      T_R : constant Short_Float := 1.0;        -- [s] requested slew time
      U_M : constant Short_Float := 1.0;        -- [Nm] max torque
      Omega_M : constant Short_Float := Pi / 18.0; -- [rad/s] max rate

      -- Slew 1: theta1 = pi/2, axis 1
      Theta1 : constant Short_Float := Pi / 2.0;
      Slew_1 : constant Slew_Properties.T := (
         Slew_Time => T_R,
         Slew_Angle => Theta1,
         Slew_Max_Rate => Omega_M,
         Slew_Max_Torque => U_M,
         Slew_Rot_Axis => 1
      );

      -- Slew 2: theta2 = pi, axis 2
      Theta2 : constant Short_Float := Pi;
      Slew_2 : constant Slew_Properties.T := (
         Slew_Time => T_R,
         Slew_Angle => Theta2,
         Slew_Max_Rate => Omega_M,
         Slew_Max_Torque => U_M,
         Slew_Rot_Axis => 2
      );

      -- Slew 3: theta3 = 2*pi, axis 3
      Theta3 : constant Short_Float := 2.0 * Pi;
      Slew_3 : constant Slew_Properties.T := (
         Slew_Time => T_R,
         Slew_Angle => Theta3,
         Slew_Max_Rate => Omega_M,
         Slew_Max_Torque => U_M,
         Slew_Rot_Axis => 3
      );

      -- Computed kinematic properties for each slew
      Alpha1, Omega1, T1, Tc1 : Short_Float;
      Alpha2, Omega2, T2, Tc2 : Short_Float;
      Alpha3, Omega3, T3, Tc3 : Short_Float;

      Tolerance : constant Short_Float := 1.0e-4;  -- Match Python test tolerance
      Output : Att_Guid.T;
      Time_Resolution : constant Short_Float := 1.0 / 65536.0;

      function To_Sys_Time (Seconds : Short_Float) return Sys_Time.T is
         Seconds_LF : constant Long_Float := Long_Float (Seconds);
         Whole_Seconds_Int : constant Integer := Integer (Long_Float'Floor (Seconds_LF));
         Whole_Seconds : Interfaces.Unsigned_32 := Interfaces.Unsigned_32 (Whole_Seconds_Int);
         Fraction : constant Long_Float := Seconds_LF - Long_Float'Floor (Seconds_LF);
         Sub_Int  : Integer := Integer (Long_Float'Floor (Fraction * 65536.0 + 0.5));
      begin
         if Sub_Int = 65536 then
            Whole_Seconds := Whole_Seconds + Interfaces.Unsigned_32 (1);
            Sub_Int := 0;
         end if;

         return (
            Seconds    => Whole_Seconds,
            Subseconds => Interfaces.Unsigned_16 (Sub_Int)
         );
      end To_Sys_Time;

      procedure Send_Tick (Count : Natural; Time_Seconds : Short_Float) is
         Tick_Time : constant Sys_Time.T := To_Sys_Time (Time_Seconds);
      begin
         T.Data_Dependency_Timestamp_Override := Tick_Time;
         T.System_Time := Tick_Time;
         T.Tick_T_Send ((Time => Tick_Time, Count => Interfaces.Unsigned_32 (Count)));
      end Send_Tick;
   begin
      -- Initialize component
      T.Component_Instance.Init;
      T.Component_Instance.Map_Data_Dependencies (
         Spacecraft_Attitude_Id => 0,
         Spacecraft_Attitude_Stale_Limit => Ada.Real_Time.Time_Span_Zero
      );

      -- Debug: Print test start
      Ada.Text_IO.Put_Line ("Starting sun_search test...");

      -- Set parameters
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Spacecraft_Inertia (Inertia)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Slew_1_Properties (Slew_1)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Slew_2_Properties (Slew_2)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Slew_3_Properties (Slew_3)), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      -- Compute expected kinematic properties
      Compute_Kinematic_Properties (Theta1, T_R, U_M, 100.0, Omega_M, Alpha1, Omega1, T1, Tc1);
      Compute_Kinematic_Properties (Theta2, T_R, U_M, 200.0, Omega_M, Alpha2, Omega2, T2, Tc2);
      Compute_Kinematic_Properties (Theta3, T_R, U_M, 300.0, Omega_M, Alpha3, Omega3, T3, Tc3);

      -----------------------------------------------------------------------
      -- Test Case 1: Beginning of Slew 1 (t = 0.1s, acceleration phase)
      -----------------------------------------------------------------------
      -- Set data dependency with matching time_tag
      T.Spacecraft_Attitude := (
         Time_Tag => 0.1,
         Sigma_Bn => [0.0, 0.0, 0.0],
         Omega_Bn_B => [0.0, 0.0, 0.0],
         Veh_Sun_Pnt_Bdy => [1.0, 0.0, 0.0]
      );
      Send_Tick (Count => 0, Time_Seconds => 0.1);

      Ada.Text_IO.Put_Line ("After first tick, message count: " & Natural'Image (T.Attitude_Guidance_History.Get_Count));
      Ada.Text_IO.Put_Line ("Data product count: " & Natural'Image (T.Data_Product_T_Recv_Sync_History.Get_Count));

      Natural_Assert.Eq (T.Attitude_Guidance_History.Get_Count, 1);
      Output := T.Attitude_Guidance_History.Get (1);

      -- At t=0.1s during slew 1: omega_RN_B[0] = omega1 * t / tc1
      -- Expected: omega_RN_B[0] = omega1 * 0.1 / tc1
      -- omega_BR_B = omega_BN_B - omega_RN_B = [0,0,0] - omega_RN_B
      -- domega_RN_B[0] = alpha1
      declare
         T_Elapsed : constant Short_Float := 0.1;
         Expected_Omega_RN_0 : constant Short_Float := Omega1 * T_Elapsed / Tc1;
      begin
         Short_Float_Assert.Eq (Output.Omega_Rn_B (0), Expected_Omega_RN_0, Tolerance);
         Short_Float_Assert.Eq (Output.Omega_Rn_B (1), 0.0, Tolerance);
         Short_Float_Assert.Eq (Output.Omega_Rn_B (2), 0.0, Tolerance);
         Short_Float_Assert.Eq (Output.Omega_Br_B (0), -Expected_Omega_RN_0, Tolerance);
         Short_Float_Assert.Eq (Output.Domega_Rn_B (0), Alpha1, Tolerance);
      end;

      -----------------------------------------------------------------------
      -- Test Case 2: Middle of Slew 1 (t = tc1, peak rate)
      -----------------------------------------------------------------------
      declare
         Slew1_Mid_Time : constant Short_Float := Tc1 - Time_Resolution;
      begin
         -- Set fresh data dependency
         T.Spacecraft_Attitude := (
            Time_Tag => Slew1_Mid_Time,
            Sigma_Bn => [0.0, 0.0, 0.0],
            Omega_Bn_B => [0.0, 0.0, 0.0],
            Veh_Sun_Pnt_Bdy => [1.0, 0.0, 0.0]
         );
         Send_Tick (Count => 1, Time_Seconds => Slew1_Mid_Time);
      end;

      Natural_Assert.Eq (T.Attitude_Guidance_History.Get_Count, 2);
      Output := T.Attitude_Guidance_History.Get (2);

      -- Close to peak of slew 1: omega_RN_B[0] ≈ omega1, domega = alpha1 (still accelerating)
      declare
         Slew1_Mid_Time : constant Short_Float := Tc1 - Time_Resolution;
         Expected_Omega_RN_0 : constant Short_Float := Omega1 * Slew1_Mid_Time / Tc1;
      begin
         Short_Float_Assert.Eq (Output.Omega_Rn_B (0), Expected_Omega_RN_0, Tolerance);
      end;
      Short_Float_Assert.Eq (Output.Domega_Rn_B (0), Alpha1, Tolerance * 10.0); -- Slightly larger tolerance at boundary

      -----------------------------------------------------------------------
      -- Test Case 3: Beginning of Slew 2 (t = T1 + 0.1s)
      -----------------------------------------------------------------------
      declare
         T_Test : constant Short_Float := T1 + 0.1;
      begin
         -- Set fresh data dependency
         T.Spacecraft_Attitude := (
            Time_Tag => T_Test,
            Sigma_Bn => [0.0, 0.0, 0.0],
            Omega_Bn_B => [0.0, 0.0, 0.0],
            Veh_Sun_Pnt_Bdy => [1.0, 0.0, 0.0]
         );
         Send_Tick (Count => 2, Time_Seconds => T_Test);
      end;

      Natural_Assert.Eq (T.Attitude_Guidance_History.Get_Count, 3);
      Output := T.Attitude_Guidance_History.Get (3);

      -- At t=T1+0.1s during slew 2: omega_RN_B[1] = omega2 * 0.1 / tc2
      declare
         T_Local : constant Short_Float := 0.1;
         Expected_Omega_RN_1 : constant Short_Float := Omega2 * T_Local / Tc2;
      begin
         Short_Float_Assert.Eq (Output.Omega_Rn_B (0), 0.0, Tolerance);  -- Slew 1 finished
         Short_Float_Assert.Eq (Output.Omega_Rn_B (1), Expected_Omega_RN_1, Tolerance);
         Short_Float_Assert.Eq (Output.Omega_Rn_B (2), 0.0, Tolerance);
         Short_Float_Assert.Eq (Output.Domega_Rn_B (1), Alpha2, Tolerance);
      end;

      -----------------------------------------------------------------------
      -- Test Case 4: Beginning of Slew 3 (t = T1 + T2 + 0.1s)
      -----------------------------------------------------------------------
      declare
         T_Test : constant Short_Float := T1 + T2 + 0.1;
      begin
         -- Set fresh data dependency
         T.Spacecraft_Attitude := (
            Time_Tag => T_Test,
            Sigma_Bn => [0.0, 0.0, 0.0],
            Omega_Bn_B => [0.0, 0.0, 0.0],
            Veh_Sun_Pnt_Bdy => [1.0, 0.0, 0.0]
         );
         Send_Tick (Count => 3, Time_Seconds => T_Test);
      end;

      Natural_Assert.Eq (T.Attitude_Guidance_History.Get_Count, 4);
      Output := T.Attitude_Guidance_History.Get (4);

      -- At t=T1+T2+0.1s during slew 3: omega_RN_B[2] = omega3 * 0.1 / tc3
      declare
         T_Local : constant Short_Float := 0.1;
         Expected_Omega_RN_2 : constant Short_Float := Omega3 * T_Local / Tc3;
      begin
         Short_Float_Assert.Eq (Output.Omega_Rn_B (0), 0.0, Tolerance);
         Short_Float_Assert.Eq (Output.Omega_Rn_B (1), 0.0, Tolerance);  -- Slew 2 finished
         Short_Float_Assert.Eq (Output.Omega_Rn_B (2), Expected_Omega_RN_2, Tolerance);
         Short_Float_Assert.Eq (Output.Domega_Rn_B (2), Alpha3, Tolerance);
      end;

      -----------------------------------------------------------------------
      -- Test Case 5: After all slews complete (t = T1 + T2 + T3 + 0.1s)
      -----------------------------------------------------------------------
      declare
         T_Test : constant Short_Float := T1 + T2 + T3 + 0.1;
      begin
         -- Set fresh data dependency
         T.Spacecraft_Attitude := (
            Time_Tag => T_Test,
            Sigma_Bn => [0.0, 0.0, 0.0],
            Omega_Bn_B => [0.0, 0.0, 0.0],
            Veh_Sun_Pnt_Bdy => [1.0, 0.0, 0.0]
         );
         Send_Tick (Count => 4, Time_Seconds => T_Test);
      end;

      Natural_Assert.Eq (T.Attitude_Guidance_History.Get_Count, 5);
      Output := T.Attitude_Guidance_History.Get (5);

      -- After all slews: all rates should be zero
      Short_Float_Assert.Eq (Output.Omega_Rn_B (0), 0.0, Tolerance);
      Short_Float_Assert.Eq (Output.Omega_Rn_B (1), 0.0, Tolerance);
      Short_Float_Assert.Eq (Output.Omega_Rn_B (2), 0.0, Tolerance);
      Short_Float_Assert.Eq (Output.Domega_Rn_B (0), 0.0, Tolerance);
      Short_Float_Assert.Eq (Output.Domega_Rn_B (1), 0.0, Tolerance);
      Short_Float_Assert.Eq (Output.Domega_Rn_B (2), 0.0, Tolerance);
   end Test;

end Sun_Search_Tests.Implementation;
