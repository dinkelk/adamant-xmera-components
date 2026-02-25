--------------------------------------------------------------------------------
-- Inertial_Ukf Tests Spec
--------------------------------------------------------------------------------

-- This is a unit test suite for the Inertial UKF component.
package Inertial_Ukf_Tests.Implementation is

   -- Test data and state:
   type Instance is new Inertial_Ukf_Tests.Base_Instance with private;
   type Class_Access is access all Instance'Class;

private
   -- Fixture procedures:
   overriding procedure Set_Up_Test (Self : in out Instance);
   overriding procedure Tear_Down_Test (Self : in out Instance);

   -- Run algorithm with known inputs to verify the pass-through of star-tracker
   -- attitude to nav attitude estimate and filter output.
   overriding procedure Test_Pass_Through (Self : in out Instance);

   -- Test data and state:
   type Instance is new Inertial_Ukf_Tests.Base_Instance with record
      null;
   end record;
end Inertial_Ukf_Tests.Implementation;
