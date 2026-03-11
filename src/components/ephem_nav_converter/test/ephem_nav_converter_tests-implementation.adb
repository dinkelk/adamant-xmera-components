--------------------------------------------------------------------------------
-- Ephem_Nav_Converter Tests Body
--------------------------------------------------------------------------------

with Basic_Assertions; use Basic_Assertions;
with Packed_F64x3.Assertion; use Packed_F64x3.Assertion;
with Ephemeris;
with Nav_Trans;

package body Ephem_Nav_Converter_Tests.Implementation is

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
      -- Clean up component:
      Self.Tester.Component_Instance.Destroy;
      -- Free component heap:
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Run algorithm to ensure integration is sound.
   overriding procedure Test (Self : in out Instance) is
      T : Component.Ephem_Nav_Converter.Implementation.Tester.Instance_Access renames Self.Tester;

      -- Test ephemeris input (Earth-like orbital parameters)
      Test_Eph : constant Ephemeris.T := (
         R_Bdy_Zero_N => [1.496e11, 0.0, 0.0],  -- Approx Earth orbit radius in m
         V_Bdy_Zero_N => [0.0, 2.978e4, 0.0],   -- Approx Earth orbital velocity in m/s
         Sigma_Bn => [0.0, 0.0, 0.0],           -- Zero attitude for simplicity
         Omega_Bn_B => [0.0, 0.0, 0.0],         -- Zero angular velocity
         Time_Tag => 1.0                         -- 1 second
      );
   begin
      -- Set the ephemeris data dependency
      T.Input_Ephemeris := Test_Eph;

      -- Send tick to run the algorithm
      T.Tick_T_Send ((Time => T.System_Time, Count => 0));

      -- Verify one data product was sent
      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 1);

      -- Get the output navigation translation message
      declare
         Nav_Trans_Out : constant Nav_Trans.T := T.Navigation_Translation_History.Get (1);
      begin
         -- Verify timeTag was copied correctly
         pragma Assert (Nav_Trans_Out.Time_Tag = Test_Eph.Time_Tag);

         -- Verify position was copied correctly (r_BN_N should equal r_BdyZero_N)
         Packed_F64x3_Assert.Eq (
            Nav_Trans_Out.R_Bn_N,
            Test_Eph.R_Bdy_Zero_N,
            Epsilon => 1.0
         );

         -- Verify velocity was copied correctly (v_BN_N should equal v_BdyZero_N)
         Packed_F64x3_Assert.Eq (
            Nav_Trans_Out.V_Bn_N,
            Test_Eph.V_Bdy_Zero_N,
            Epsilon => 1.0e-3
         );
      end;
   end Test;

end Ephem_Nav_Converter_Tests.Implementation;
