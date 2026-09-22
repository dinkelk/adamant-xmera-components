--------------------------------------------------------------------------------
-- Solar_Array_Reference Tests Body
--------------------------------------------------------------------------------

with Ada.Assertions;
with Ada.Real_Time;
with AUnit.Assertions;
with Parameter;
with Basic_Assertions; use Basic_Assertions;
with Packed_F32.Assertion; use Packed_F32.Assertion;
with Packed_F32;
with Packed_F32x3;
with Packed_Tracking_Mode;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;
with Solar_Array_Reference_Enums; use Solar_Array_Reference_Enums;
with Solar_Array_Reference_Parameters;

package body Solar_Array_Reference_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Test configuration:
   -------------------------------------------------------------------------

   -- Array axes, attitudes, and sun directions from the Python reference test
   -- (_tests/test_solarArrayReference.py). The reference test gives the sun direction
   -- in the inertial frame; the body frame directions below are that vector rotated by
   -- the corresponding body attitude.
   Drive_Axis : constant Packed_F32x3.T := [1.0, 0.0, 0.0];
   Surface_Normal : constant Packed_F32x3.T := [0.0, 1.0, 0.0];
   Alignment_Threshold : constant Packed_F32.T := (Value => 0.1);
   Auto_Track : constant Packed_Tracking_Mode.T := (Value => Tracking_Mode.Auto_Track);
   Specified_Angle : constant Packed_Tracking_Mode.T := (Value => Tracking_Mode.Specified_Angle);
   Attitude_A : constant Packed_F32x3.T := [0.1, 0.2, 0.3];
   Reference_A : constant Packed_F32x3.T := [0.3, 0.2, 0.1];
   Attitude_B : constant Packed_F32x3.T := [0.5, 0.4, 0.3];
   Reference_B : constant Packed_F32x3.T := [0.9, 0.7, 0.8];
   -- Inertial x and z sun directions seen from attitude A and attitude B.
   Sun_X_From_A : constant Packed_F32x3.T := [0.1997537704, -0.6709756848, 0.7140658664];
   Sun_Z_From_A : constant Packed_F32x3.T := [-0.3447214528, 0.6340412435, 0.6922129886];
   Sun_X_From_B : constant Packed_F32x3.T := [0.1111111111, 0.4444444444, 0.8888888889];
   Sun_Z_From_B : constant Packed_F32x3.T := [0.1777777778, 0.8711111111, -0.4577777778];

   -- The reference angles are computed in double precision; the single precision
   -- algorithm agrees to a few parts in a million.
   Epsilon : constant := 1.0E-5;

   -------------------------------------------------------------------------
   -- Helpers:
   -------------------------------------------------------------------------

   -- Stage and apply the axes and the alignment threshold. The array axes are the same
   -- in every case.
   procedure Apply_Configuration (Self : in out Instance; Threshold : in Packed_F32.T) is
      T : Component.Solar_Array_Reference.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Solar_Array_Reference_Parameters.Instance;
   begin
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Drive_Axis (Drive_Axis)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Surface_Normal (Surface_Normal)), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alignment_Threshold (Threshold)), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);
   end Apply_Configuration;

   -- Command the mode and angles, send one tick with the given attitude, reference, and
   -- sun direction, and check the published reference angle.
   procedure Send_Tick_And_Check (
      Self : in out Instance;
      Tick_Number : in Natural;
      Mode : in Packed_Tracking_Mode.T;
      Specified : in Short_Float;
      Offset : in Short_Float;
      Attitude : in Packed_F32x3.T;
      Reference : in Packed_F32x3.T;
      Sun_Body : in Packed_F32x3.T;
      Expected_Angle : in Short_Float
   ) is
      T : Component.Solar_Array_Reference.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Tracking_Mode := Mode;
      T.Specified_Array_Angle := (Value => Specified);
      T.Offset_Angle := (Value => Offset);
      T.Navigation_Attitude := (Time_Tag => 0.0, Sigma_Bn => Attitude, Omega_Bn_B => [0.0, 0.0, 0.0], Veh_Sun_Pnt_Bdy => [0.0, 0.0, 0.0]);
      T.Attitude_Reference := (Sigma_Rn => Reference, Omega_Rn_N => [0.0, 0.0, 0.0], Domega_Rn_N => [0.0, 0.0, 0.0]);
      T.Sun_Direction_Body := Sun_Body;
      T.Tick_T_Send ((Time => T.System_Time, Count => 0));
      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, Tick_Number);
      Natural_Assert.Eq (T.Reference_Angle_History.Get_Count, Tick_Number);
      Packed_F32_Assert.Eq (T.Reference_Angle_History.Get (Tick_Number), (Value => Expected_Angle), Epsilon => Epsilon);
   end Send_Tick_And_Check;

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

   -- Run the algorithm on the sun tracking cases of the Python reference test to ensure
   -- the Ada to C to C++ integration is sound. The expected angles are computed
   -- independently from the reference model: the sun direction is carried into the
   -- reference attitude and the angle turns the surface normal toward it.
   overriding procedure Test (Self : in out Instance) is
   begin
      Apply_Configuration (Self, Threshold => Alignment_Threshold);
      Send_Tick_And_Check (Self, 1, Auto_Track, 0.0, 0.0, Attitude_A, Reference_A, Sun_X_From_A, 1.4252804701);
      Send_Tick_And_Check (Self, 2, Auto_Track, 0.0, 0.0, Attitude_A, Reference_A, Sun_Z_From_A, 0.2144368067);
      Send_Tick_And_Check (Self, 3, Auto_Track, 0.0, 0.0, Attitude_B, Reference_B, Sun_X_From_B, 0.3706993963);
      Send_Tick_And_Check (Self, 4, Auto_Track, 0.0, 0.0, Attitude_B, Reference_B, Sun_Z_From_B, -1.0129138132);
   end Test;

   -- Check the aligned sun fallback, the commanded offset, and the commanded angle mode
   -- with wrapping against the Python reference model.
   overriding procedure Test_Tracking_Mode_Variants (Self : in out Instance) is
      Zero_Attitude : constant Packed_F32x3.T := [0.0, 0.0, 0.0];
   begin
      Apply_Configuration (Self, Threshold => Alignment_Threshold);

      -- With the sun along the drive axis the array has no preferred angle, so the
      -- reference holds the current angle, which is not wired and passed as zero:
      Send_Tick_And_Check (Self, 1, Auto_Track, 0.0, 0.0, Zero_Attitude, Zero_Attitude, Drive_Axis, 0.0);

      -- The commanded offset is added to the tracked angle:
      Send_Tick_And_Check (Self, 2, Auto_Track, 0.0, 0.3, Attitude_A, Reference_A, Sun_Z_From_A, 0.5144368067);

      -- The commanded angle mode ignores the attitude and sun inputs. The sum of the
      -- commanded and offset angles is wrapped to [-pi, pi]:
      Send_Tick_And_Check (Self, 3, Specified_Angle, 0.5, 0.3, Attitude_A, Reference_A, Sun_Z_From_A, 0.8);
      Send_Tick_And_Check (Self, 4, Specified_Angle, 2.0, 2.0, Attitude_A, Reference_A, Sun_Z_From_A, -2.2831853072);
      Send_Tick_And_Check (Self, 5, Specified_Angle, -2.0, -2.0, Attitude_A, Reference_A, Sun_Z_From_A, 2.2831853072);
   end Test_Tracking_Mode_Variants;

   -- The mode and angles are commanded sporadically, so on most ticks they come back
   -- stale. A stale command must leave the last configuration in place, and a
   -- parameter update in the meantime must reapply that configuration rather than
   -- the defaults.
   overriding procedure Test_Stale_Command (Self : in out Instance) is
      T : Component.Solar_Array_Reference.Implementation.Tester.Instance_Access renames Self.Tester;
      -- Only the commanded dependencies get a stale limit, so they alone come back
      -- Stale once the tick time runs ahead of the product time.
      Never : constant Ada.Real_Time.Time_Span := Ada.Real_Time.Time_Span_Zero;
      One_Second : constant Ada.Real_Time.Time_Span := Ada.Real_Time.Seconds (1);
   begin
      T.Component_Instance.Map_Data_Dependencies (
         Navigation_Attitude_Id => 0, Navigation_Attitude_Stale_Limit => Never,
         Attitude_Reference_Id => 1, Attitude_Reference_Stale_Limit => Never,
         Sun_Direction_Body_Id => 2, Sun_Direction_Body_Stale_Limit => Never,
         Tracking_Mode_Id => 3, Tracking_Mode_Stale_Limit => One_Second,
         Specified_Array_Angle_Id => 4, Specified_Array_Angle_Stale_Limit => One_Second,
         Offset_Angle_Id => 5, Offset_Angle_Stale_Limit => One_Second);
      Apply_Configuration (Self, Threshold => Alignment_Threshold);

      -- A fresh command configures the algorithm:
      Send_Tick_And_Check (Self, 1, Specified_Angle, 0.5, 0.3, Attitude_A, Reference_A, Sun_Z_From_A, 0.8);

      -- The commands go stale: the products are stamped in the past and the tick runs
      -- well ahead of them. Different commanded values are offered, and ignored:
      T.Data_Dependency_Timestamp_Override := (Seconds => 1, Subseconds => 0);
      T.Tracking_Mode := Auto_Track;
      T.Specified_Array_Angle := (Value => 1.0);
      T.Offset_Angle := (Value => 1.0);
      T.Tick_T_Send ((Time => (Seconds => 100, Subseconds => 0), Count => 0));
      Natural_Assert.Eq (T.Reference_Angle_History.Get_Count, 2);
      Packed_F32_Assert.Eq (T.Reference_Angle_History.Get (2), (Value => 0.8), Epsilon => Epsilon);

      -- A parameter update reapplies the last command rather than the defaults:
      Apply_Configuration (Self, Threshold => (Value => 0.2));
      T.Tick_T_Send ((Time => (Seconds => 101, Subseconds => 0), Count => 0));
      Natural_Assert.Eq (T.Reference_Angle_History.Get_Count, 3);
      Packed_F32_Assert.Eq (T.Reference_Angle_History.Get (3), (Value => 0.8), Epsilon => Epsilon);
   end Test_Stale_Command;

   -- The algorithm requires unit and orthogonal array axes and an alignment threshold
   -- in [1e-3, pi/2]. Validation is the only guard keeping a rejected value out of the
   -- throwing Set_Config, so exercise it directly.
   overriding procedure Test_Invalid_Parameter (Self : in out Instance) is
      T : Component.Solar_Array_Reference.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Solar_Array_Reference_Parameters.Instance;

      -- Stage a known-good set, so each rejection below is caused by the single
      -- perturbed value rather than by leftover staging state.
      procedure Stage_Valid_Configuration is
      begin
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Drive_Axis (Drive_Axis)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Surface_Normal (Surface_Normal)), Success);
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alignment_Threshold (Alignment_Threshold)), Success);
      end Stage_Valid_Configuration;
   begin
      -- The reference configuration is accepted:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);

      -- A drive axis that is not a unit vector is rejected:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Drive_Axis ([2.0, 0.0, 0.0])), Success);
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);

      -- A surface normal that is not orthogonal to the drive axis is rejected:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Surface_Normal ([0.7071067812, 0.7071067812, 0.0])), Success);
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);

      -- An alignment threshold below the minimum is rejected:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Params.Alignment_Threshold ((Value => 0.0))), Success);
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);

      -- A non-finite drive axis is rejected. The value is injected as raw bytes because the
      -- compiler will not let a non-finite Short_Float be written as a literal, and
      -- because that is how one would arrive: as bytes from the ground. Staging accepts
      -- it, and converting it for the algorithm raises, which validation reports as a
      -- rejection.
      Stage_Valid_Configuration;
      declare
         Par : Parameter.T := Params.Drive_Axis (Drive_Axis);
      begin
         -- Overwrite the first of the three big-endian floats with +infinity.
         Par.Buffer (Par.Buffer'First .. Par.Buffer'First + 3) := [16#7F#, 16#80#, 16#00#, 16#00#];
         Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (Par), Success);
      end;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Validation_Error);

      -- Restoring the reference configuration makes the set acceptable again, so the
      -- rejections above were caused by the perturbed values:
      Stage_Valid_Configuration;
      Parameter_Update_Status_Assert.Eq (T.Validate_Parameters, Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);
   end Test_Invalid_Parameter;

   -- A commanded angle outside [-pi, pi] would be rejected by the algorithm. Validating
   -- what is commanded belongs to the component that commands it, so the component
   -- treats such a value as a defect and asserts rather than publishing anything.
   overriding procedure Test_Invalid_Command (Self : in out Instance) is
      T : Component.Solar_Array_Reference.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      Apply_Configuration (Self, Threshold => Alignment_Threshold);
      begin
         Send_Tick_And_Check (Self, 1, Specified_Angle, 4.0, 0.0, Attitude_A, Reference_A, Sun_Z_From_A, 0.0);
         AUnit.Assertions.Assert (False, "A commanded angle outside the accepted range should have failed an assertion.");
      exception
         when Ada.Assertions.Assertion_Error =>
            null; -- Expected.
      end;
      Natural_Assert.Eq (T.Reference_Angle_History.Get_Count, 0);
   end Test_Invalid_Command;

   -- A data dependency that comes back with the wrong identifier means the assembly
   -- is wired incorrectly. The component asserts rather than publishing anything.
   overriding procedure Test_Invalid_Data_Dependency (Self : in out Instance) is
      T : Component.Solar_Array_Reference.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      T.Data_Dependency_Return_Id_Override := 999;
      begin
         T.Tick_T_Send ((Time => T.System_Time, Count => 0));
         AUnit.Assertions.Assert (False, "A dependency with the wrong identifier should have failed an assertion.");
      exception
         when Ada.Assertions.Assertion_Error =>
            null; -- Expected.
      end;
      Natural_Assert.Eq (T.Reference_Angle_History.Get_Count, 0);
   end Test_Invalid_Data_Dependency;

end Solar_Array_Reference_Tests.Implementation;
