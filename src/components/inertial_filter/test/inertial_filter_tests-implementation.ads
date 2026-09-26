--------------------------------------------------------------------------------
-- Inertial_Filter Tests Spec
--------------------------------------------------------------------------------

-- This is a unit test suite for the Inertial Filter component
package Inertial_Filter_Tests.Implementation is

   -- Test data and state:
   type Instance is new Inertial_Filter_Tests.Base_Instance with private;
   type Class_Access is access all Instance'Class;

private
   -- Fixture procedures:
   overriding procedure Set_Up_Test (Self : in out Instance);
   overriding procedure Tear_Down_Test (Self : in out Instance);

   -- Feed a constant star tracker attitude and check the estimate converges to it
   -- while the covariance shrinks, as in the algorithm's Python reference test, to
   -- ensure the Ada to C to C++ integration is sound.
   overriding procedure Test (Self : in out Instance);
   -- Feed a constant body rate about one axis with the attitude that rotation gives,
   -- and check the rate estimate converges to it while the attitude tracks the
   -- truth.
   overriding procedure Test_Rate_Update (Self : in out Instance);
   -- Ensure a star tracker product whose time tag has not advanced is not fed to the
   -- filter, so the estimate holds and the covariance grows under the process noise.
   overriding procedure Test_Propagation (Self : in out Instance);
   -- Ensure the estimate reset re-seeds the state and covariance while the
   -- measurements reset keeps them.
   overriding procedure Test_Reset (Self : in out Instance);
   -- Ensure a star tracker reading stamped before the time base restarted by a reset
   -- is dropped, and a later one is applied.
   overriding procedure Test_Reading_Before_Time_Base (Self : in out Instance);
   -- Ensure a reading stamped after the tick waits until the tick reaches it, and a
   -- tick before the time base restarts it, without either shutting out the readings
   -- that follow.
   overriding procedure Test_Time_Anomalies (Self : in out Instance);
   -- Ensure the diagnostics packet is sent every commanded number of ticks, that a
   -- period of zero turns it off, and that the command is reported.
   overriding procedure Test_Diagnostics_Packet (Self : in out Instance);
   -- Ensure a parameter update rebuilds the filter from the new configuration,
   -- restarting the estimate from the configured seed.
   overriding procedure Test_Parameter_Update (Self : in out Instance);
   -- Ensure a staged configuration the algorithm would reject is refused at
   -- validation.
   overriding procedure Test_Invalid_Parameter (Self : in out Instance);
   -- Ensure a malformed command is reported rather than acted on.
   overriding procedure Test_Invalid_Command (Self : in out Instance);
   -- Ensure a data dependency with the wrong identifier is treated as a wiring
   -- defect and fails the tick's assertion.
   overriding procedure Test_Invalid_Data_Dependency (Self : in out Instance);

   -- Test data and state:
   type Instance is new Inertial_Filter_Tests.Base_Instance with record
      null;
   end record;
end Inertial_Filter_Tests.Implementation;
