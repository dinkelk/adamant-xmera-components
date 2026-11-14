--------------------------------------------------------------------------------
-- Gimbal_Angles_To_Motor_Angles Tests Body
--------------------------------------------------------------------------------

with Ada.Assertions;
with AUnit.Assertions;
with Parameter;
with Basic_Assertions; use Basic_Assertions;
with Gimbal_Angles_To_Motor_Angles_Parameters;
with Packed_F32;
with Packed_F32.Assertion; use Packed_F32.Assertion;
with Packed_F32x9;
with Packed_U32;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;

package body Gimbal_Angles_To_Motor_Angles_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Test configuration:
   -------------------------------------------------------------------------

   -- The two motor angles of one tick.
   type Motor_Angles is record
      Motor_1 : Short_Float;
      Motor_2 : Short_Float;
   end record;

   -- The tests run with the default parameter values. The expected motor angles
   -- are a double precision evaluation of the default spline surfaces at the
   -- requested gimbal angles, multiplied by the step angle 2*pi/628.
   --
   -- Motor angles at zero gimbal angles:
   Zero_Gimbal_Angles : constant Motor_Angles := (Motor_1 => -150.5479400, Motor_2 => 150.5650514);
   -- Motor angles at a tip of 0.1 rad and a tilt of -0.05 rad:
   Inside_Tip : constant Short_Float := 0.1;
   Inside_Tilt : constant Short_Float := -0.05;
   Inside_Angles : constant Motor_Angles := (Motor_1 => -161.0974441, Motor_2 => 172.8912739);

   -- The motor angles are near 150 rad, where single precision resolves about
   -- 1.5E-5 rad. The tolerance leaves room for the rounding of the surface
   -- evaluation and is a tenth of the default step angle.
   Epsilon : constant := 1.0E-3;

   -------------------------------------------------------------------------
   -- Helpers:
   -------------------------------------------------------------------------

   -- Tick once with the given gimbal angles and check the published motor angles.
   procedure Tick_And_Expect (
      Self : in out Instance;
      Tick_Number : in Natural;
      Tip : in Short_Float;
      Tilt : in Short_Float;
      Expected : in Motor_Angles
   ) is
      T : Component.Gimbal_Angles_To_Motor_Angles.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      -- The component does not read the thrust direction.
      T.Gimbal_Command := (Gimbal_Angle_1 => Tip, Gimbal_Angle_2 => Tilt, Thrust_Hat_B => [0.0, 0.0, 1.0]);
      T.Tick_T_Send ((Time => T.System_Time, Count => 0));
      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 2 * Tick_Number);
      Natural_Assert.Eq (T.Motor_1_Angle_History.Get_Count, Tick_Number);
      Natural_Assert.Eq (T.Motor_2_Angle_History.Get_Count, Tick_Number);
      Packed_F32_Assert.Eq (T.Motor_1_Angle_History.Get (Tick_Number), (Value => Expected.Motor_1), Epsilon => Epsilon);
      Packed_F32_Assert.Eq (T.Motor_2_Angle_History.Get (Tick_Number), (Value => Expected.Motor_2), Epsilon => Epsilon);
   end Tick_And_Expect;

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;

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

   -- Compute the motor angles for known gimbal angles with the default spline
   -- surfaces and compare against a double precision evaluation of the same
   -- surfaces, to ensure the Ada to C to C++ integration is sound. The requests
   -- are in four different spline intervals, so a coefficient row or column in the
   -- wrong place changes a result.
   overriding procedure Test (Self : in out Instance) is
   begin
      Tick_And_Expect (Self, 1, Tip => 0.0, Tilt => 0.0, Expected => Zero_Gimbal_Angles);
      Tick_And_Expect (Self, 2, Tip => Inside_Tip, Tilt => Inside_Tilt, Expected => Inside_Angles);
      Tick_And_Expect (Self, 3, Tip => -0.15, Tilt => 0.1, Expected => (Motor_1 => -136.0616974, Motor_2 => 110.0214133));
      Tick_And_Expect (Self, 4, Tip => 0.05, Tilt => 0.2, Expected => (Motor_1 => -182.4826258, Motor_2 => 136.5570549));
   end Test;

   -- Check that a request outside the addressable rhombus keeps the last computed
   -- motor angles, and that before a first valid request and after a reset it gives
   -- the motor angles at zero gimbal angles. The default rhombus reaches a tip of
   -- about 0.284 rad on the tip axis.
   overriding procedure Test_Outside_Rhombus (Self : in out Instance) is
      T : Component.Gimbal_Angles_To_Motor_Angles.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      -- No motor angles were computed yet, so a request outside the rhombus gives the
      -- motor angles at zero gimbal angles:
      Tick_And_Expect (Self, 1, Tip => 0.3, Tilt => 0.0, Expected => Zero_Gimbal_Angles);

      -- A request inside the rhombus is computed:
      Tick_And_Expect (Self, 2, Tip => Inside_Tip, Tilt => Inside_Tilt, Expected => Inside_Angles);

      -- Requests outside the rhombus now keep the computed motor angles:
      Tick_And_Expect (Self, 3, Tip => 0.3, Tilt => 0.0, Expected => Inside_Angles);
      Tick_And_Expect (Self, 4, Tip => 0.2, Tilt => 0.3, Expected => Inside_Angles);

      -- A reset clears them:
      T.Reset_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Tick_And_Expect (Self, 5, Tip => 0.3, Tilt => 0.0, Expected => Zero_Gimbal_Angles);
   end Test_Outside_Rhombus;

   -- Check that a reset, not a tick, applies a parameter update. Twice the steps per
   -- revolution is half the step angle, so the same request gives half the motor
   -- angles.
   overriding procedure Test_Parameter_Update (Self : in out Instance) is
      T : Component.Gimbal_Angles_To_Motor_Angles.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Gimbal_Angles_To_Motor_Angles_Parameters.Instance;
      Half_Angles : constant Motor_Angles :=
         (Motor_1 => Inside_Angles.Motor_1 / 2.0, Motor_2 => Inside_Angles.Motor_2 / 2.0);
   begin
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Steps_Per_Revolution ((Value => 1_256))), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      -- A tick still runs with the default steps per revolution:
      Tick_And_Expect (Self, 1, Tip => Inside_Tip, Tilt => Inside_Tilt, Expected => Inside_Angles);

      -- The reset applies the update:
      T.Reset_Tick_T_Send ((Time => T.System_Time, Count => 0));
      Tick_And_Expect (Self, 2, Tip => Inside_Tip, Tilt => Inside_Tilt, Expected => Half_Angles);
   end Test_Parameter_Update;

   -- The algorithm requires steps per revolution in [1, 100000], half-diagonals in
   -- (0, pi/2], a cone angle that fits inside the rhombus, and coefficients in
   -- [-100000, 100000]. Validation is the only guard keeping a rejected value out of
   -- the throwing Set_Config, so exercise it directly.
   overriding procedure Test_Invalid_Parameter (Self : in out Instance) is
      T : Component.Gimbal_Angles_To_Motor_Angles.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Gimbal_Angles_To_Motor_Angles_Parameters.Instance;

      -- Values of the reference configuration that the cases below perturb.
      Steps : constant Packed_U32.T := (Value => 628);
      Tip : constant Packed_F32.T := (Value => 0.28);
      Tilt : constant Packed_F32.T := (Value => 0.43);
      Cone : constant Packed_F32.T := (Value => 0.23);
      Flat_Row : constant Packed_F32x9.T := [others => 100.0];

      -- Stage a known-good value for every parameter a case perturbs, so each
      -- rejection below is caused by the single perturbed value rather than by
      -- leftover staging state.
      procedure Stage_Valid_Configuration is
      begin
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Steps_Per_Revolution (Steps)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Half_Diagonal_Tip (Tip)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Half_Diagonal_Tilt (Tilt)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Theta_Max (Cone)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Motor_1_Coefficients_Row_0 (Flat_Row)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Motor_2_Coefficients_Row_8 (Flat_Row)), Success);
      end Stage_Valid_Configuration;

      -- Stage the reference configuration with one parameter replaced and check that
      -- validation rejects the set.
      procedure Expect_Rejected (Par : in Parameter.T) is
      begin
         Stage_Valid_Configuration;
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Par), Success);
         Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);
      end Expect_Rejected;
   begin
      -- The reference configuration is accepted:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);

      -- Zero steps per revolution, and more than 100000, are rejected:
      Expect_Rejected (Params.Steps_Per_Revolution ((Value => 0)));
      Expect_Rejected (Params.Steps_Per_Revolution ((Value => 100_001)));

      -- A half-diagonal of zero, and one beyond 90 degrees, are rejected:
      Expect_Rejected (Params.Half_Diagonal_Tip ((Value => 0.0)));
      Expect_Rejected (Params.Half_Diagonal_Tilt ((Value => 1.6)));

      -- A cone angle of zero, and one that does not fit inside the rhombus, are
      -- rejected. The largest circle in the reference rhombus has a radius of about
      -- 0.2346 rad.
      Expect_Rejected (Params.Theta_Max ((Value => 0.0)));
      Expect_Rejected (Params.Theta_Max ((Value => 0.24)));

      -- A coefficient beyond 100000 steps is rejected, in the first row of motor 1
      -- and in the last row of motor 2:
      Expect_Rejected (Params.Motor_1_Coefficients_Row_0 ([0 => 100_001.0, others => 100.0]));
      Expect_Rejected (Params.Motor_2_Coefficients_Row_8 ([8 => -100_001.0, others => 100.0]));

      -- A non-finite coefficient is rejected. The value is injected as raw bytes
      -- because the compiler will not let a non-finite Short_Float be written as a
      -- literal, and because that is how one would arrive: as bytes from the ground.
      -- Staging accepts it, and converting it for the algorithm raises, which
      -- validation reports as a rejection.
      declare
         Par : Parameter.T := Params.Motor_1_Coefficients_Row_0 (Flat_Row);
      begin
         -- Overwrite the first of the nine big-endian floats with +infinity.
         Par.Buffer (Par.Buffer'First .. Par.Buffer'First + 3) := [16#7F#, 16#80#, 16#00#, 16#00#];
         Expect_Rejected (Par);
      end;

      -- Restoring the reference configuration makes the set acceptable again, so the
      -- rejections above were caused by the perturbed values:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);
      T.Reset_Tick_T_Send ((Time => T.System_Time, Count => 0));
   end Test_Invalid_Parameter;

   -- A data dependency that comes back with the wrong identifier means the assembly
   -- is wired incorrectly. The component asserts rather than publishing anything.
   overriding procedure Test_Invalid_Data_Dependency (Self : in out Instance) is
      T : Component.Gimbal_Angles_To_Motor_Angles.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Data_Dependency_Return_Id_Override := 999;
      begin
         T.Tick_T_Send ((Time => T.System_Time, Count => 0));
         AUnit.Assertions.Assert (False, "A dependency with the wrong identifier should have failed an assertion.");
      exception
         when Ada.Assertions.Assertion_Error =>
            null; -- Expected.
      end;
      Natural_Assert.Eq (T.Motor_1_Angle_History.Get_Count, 0);
      Natural_Assert.Eq (T.Motor_2_Angle_History.Get_Count, 0);
   end Test_Invalid_Data_Dependency;

end Gimbal_Angles_To_Motor_Angles_Tests.Implementation;
