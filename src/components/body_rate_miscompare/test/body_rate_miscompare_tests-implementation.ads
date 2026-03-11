--------------------------------------------------------------------------------
-- Body_Rate_Miscompare Tests Spec
--------------------------------------------------------------------------------

-- This is a unit test suite for the Body Rate Miscompare component
package Body_Rate_Miscompare_Tests.Implementation is

   -- Test data and state:
   type Instance is new Body_Rate_Miscompare_Tests.Base_Instance with private;
   type Class_Access is access all Instance'Class;

private
   -- Fixture procedures:
   overriding procedure Set_Up_Test (Self : in out Instance);
   overriding procedure Tear_Down_Test (Self : in out Instance);

   -- Run algorithm to ensure integration is sound.
   overriding procedure Test (Self : in out Instance);

   -- Test data and state:
   type Instance is new Body_Rate_Miscompare_Tests.Base_Instance with record
      null;
   end record;
end Body_Rate_Miscompare_Tests.Implementation;
