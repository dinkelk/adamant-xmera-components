--------------------------------------------------------------------------------
-- Inertial_3d Tests Body
--------------------------------------------------------------------------------

with Packed_F32x3;
with Packed_F32x3.Assertion; use Packed_F32x3.Assertion;
with Component.Inertial_3d.Implementation.Tester;
with Att_Ref;
with Tick;

package body Inertial_3d_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Helpers:
   -------------------------------------------------------------------------

   -- Request a tick and return the reference it produces.
   function Request_Tick (Self : in out Instance; Arg : in Tick.T) return Att_Ref.T is
      T : Component.Inertial_3d.Implementation.Tester.Instance_Access renames Self.Tester;
   begin
      return T.Tick_T_Request (Arg);
   end Request_Tick;

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
      -- Free component heap:
      Self.Tester.Component_Instance.Destroy;
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Run algorithm to ensure integration is sound.
   overriding procedure Test (Self : in out Instance) is
      T : Component.Inertial_3d.Implementation.Tester.Instance_Access renames Self.Tester;

      -- Test inputs based on Python unit test scenarios with and without a set sigma reference.
      type Test_Case is record
         Sigma_Input : Packed_F32x3.T;
      end record;

      Test_Cases : constant array (1 .. 2) of Test_Case := [
         (Sigma_Input => [0.0, 0.0, 0.0]),
         (Sigma_Input => [0.1, -0.2, 0.3])
      ];

      Zero_Vector : constant Packed_F32x3.T := [0.0, 0.0, 0.0];
      Epsilon : constant := 1.0E-6;
   begin
      for I in Test_Cases'Range loop
         -- Provide sigma reference input for this tick.
         T.Sigma_Reference := (Value => Test_Cases (I).Sigma_Input);

         -- Trigger the component execution and check the reference it returns.
         declare
            Output : constant Att_Ref.T := Request_Tick (Self, (Time => T.System_Time, Count => 0));
         begin
            Packed_F32x3_Assert.Eq (Output.Sigma_Rn, Test_Cases (I).Sigma_Input, Epsilon => Epsilon);
            Packed_F32x3_Assert.Eq (Output.Omega_Rn_N, Zero_Vector, Epsilon => Epsilon);
            Packed_F32x3_Assert.Eq (Output.Domega_Rn_N, Zero_Vector, Epsilon => Epsilon);
         end;
      end loop;
   end Test;

   -- The reference attitude is immutable algorithm configuration, so the component
   -- pushes it across the FFI boundary only when the fetched value differs from the
   -- one already applied. Both paths must return the fetched attitude, and it is
   -- that -- not the skipped Set_Config, which the tester cannot observe -- that is
   -- asserted here. The skip itself shows up as branch coverage on the change test.
   overriding procedure Test_Reconfigures_Only_On_Change (Self : in out Instance) is
      T : Component.Inertial_3d.Implementation.Tester.Instance_Access renames Self.Tester;
      Attitude : constant Packed_F32x3.T := [0.4, 0.5, -0.6];
      Moved : constant Packed_F32x3.T := [-0.1, 0.2, 0.3];
      Zero_Vector : constant Packed_F32x3.T := [0.0, 0.0, 0.0];
      Epsilon : constant := 1.0E-6;
   begin
      -- First tick applies a non-zero attitude, changed from the zero configuration
      -- Init constructed the algorithm with.
      T.Sigma_Reference := (Value => Attitude);
      Packed_F32x3_Assert.Eq (Request_Tick (Self, (Time => T.System_Time, Count => 0)).Sigma_Rn, Attitude, Epsilon => Epsilon);

      -- Second tick fetches the same attitude. The reconfiguration is skipped, but
      -- the reference is still returned, and still carries the configured value.
      Packed_F32x3_Assert.Eq (Request_Tick (Self, (Time => T.System_Time, Count => 1)).Sigma_Rn, Attitude, Epsilon => Epsilon);

      -- Third tick moves the attitude, so the algorithm must be reconfigured and the
      -- new value must appear in the returned reference.
      T.Sigma_Reference := (Value => Moved);
      declare
         Output : constant Att_Ref.T := Request_Tick (Self, (Time => T.System_Time, Count => 2));
      begin
         Packed_F32x3_Assert.Eq (Output.Sigma_Rn, Moved, Epsilon => Epsilon);
         Packed_F32x3_Assert.Eq (Output.Omega_Rn_N, Zero_Vector, Epsilon => Epsilon);
         Packed_F32x3_Assert.Eq (Output.Domega_Rn_N, Zero_Vector, Epsilon => Epsilon);
      end;

      -- Returning to the original attitude is a change again, so it is re-applied.
      T.Sigma_Reference := (Value => Attitude);
      Packed_F32x3_Assert.Eq (Request_Tick (Self, (Time => T.System_Time, Count => 3)).Sigma_Rn, Attitude, Epsilon => Epsilon);
   end Test_Reconfigures_Only_On_Change;

end Inertial_3d_Tests.Implementation;
