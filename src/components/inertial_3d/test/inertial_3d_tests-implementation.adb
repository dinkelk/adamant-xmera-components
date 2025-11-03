--------------------------------------------------------------------------------
-- Inertial_3d Tests Body
--------------------------------------------------------------------------------

with Basic_Assertions; use Basic_Assertions;
with Packed_F32x3;
with Packed_F32x3.Assertion; use Packed_F32x3.Assertion;
with Component.Inertial_3d.Implementation.Tester;
with Att_Ref;

package body Inertial_3d_Tests.Implementation is

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

         -- Trigger the component execution.
         T.Tick_T_Send ((Time => T.System_Time, Count => 0));

         -- Ensure output was published.
         Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, I);
         Natural_Assert.Eq (T.Attitude_Reference_History.Get_Count, I);

         declare
            Output : constant Att_Ref.T := T.Attitude_Reference_History.Get (I);
         begin
            Packed_F32x3_Assert.Eq (Output.Sigma_Rn, Test_Cases (I).Sigma_Input, Epsilon => Epsilon);
            Packed_F32x3_Assert.Eq (Output.Omega_Rn_N, Zero_Vector, Epsilon => Epsilon);
            Packed_F32x3_Assert.Eq (Output.Domega_Rn_N, Zero_Vector, Epsilon => Epsilon);
         end;
      end loop;
   end Test;

end Inertial_3d_Tests.Implementation;
