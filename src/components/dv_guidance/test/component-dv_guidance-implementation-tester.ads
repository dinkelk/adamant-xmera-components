--------------------------------------------------------------------------------
-- Dv_Guidance Component Tester Spec
--------------------------------------------------------------------------------

-- Includes:
with Component.Dv_Guidance_Reciprocal;

-- Delta-V guidance. Produces the inertially fixed attitude reference for a
-- delta-V burn. The burn frame has its third axis along the commanded delta-V and
-- its second axis as close as possible to a preferred direction, both sent by the
-- state manager when a burn is commanded, so the reference attitude is a constant
-- between burn commands and the reference rate and acceleration are zero. A
-- rotation about the burn axis, when wanted, is applied by a downstream reference
-- rotation taking this reference as its base. Wraps the DvGuidanceAlgorithm C++
-- algorithm via its C shim.
package Component.Dv_Guidance.Implementation.Tester is

   use Component.Dv_Guidance_Reciprocal;
   -- Component class instance:
   type Instance is new Component.Dv_Guidance_Reciprocal.Base_Instance with record
      -- The component instance under test:
      Component_Instance : aliased Component.Dv_Guidance.Implementation.Instance;
   end record;
   type Instance_Access is access all Instance;

   ---------------------------------------
   -- Test initialization functions:
   ---------------------------------------
   procedure Connect (Self : in out Instance);

end Component.Dv_Guidance.Implementation.Tester;
