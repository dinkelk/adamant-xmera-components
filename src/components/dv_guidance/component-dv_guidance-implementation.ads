--------------------------------------------------------------------------------
-- Dv_Guidance Component Implementation Spec
--------------------------------------------------------------------------------

-- Includes:
with Att_Ref;
with Tick;
with Packed_Command_Execution_Status;
with Dv_Burn_Frame;
with Dv_Guidance_Algorithm_C; use Dv_Guidance_Algorithm_C;

-- Delta-V guidance. Produces the inertially fixed attitude reference for a
-- delta-V burn. The burn frame has its third axis along the commanded delta-V and
-- its second axis as close as possible to a preferred direction, both sent by the
-- state manager when a burn is commanded, so the reference attitude is a constant
-- between burn commands and the reference rate and acceleration are zero. A
-- rotation about the burn axis, when wanted, is applied by a downstream reference
-- rotation taking this reference as its base. Wraps the DvGuidanceAlgorithm C++
-- algorithm via its C shim.
package Component.Dv_Guidance.Implementation is

   -- The component class instance record:
   type Instance is new Dv_Guidance.Base_Instance with private;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the delta-V guidance algorithm with a stand-in burn frame, which the
   -- first burn command replaces.
   overriding procedure Init (Self : in out Instance);
   not overriding procedure Destroy (Self : in out Instance);

private

   -- The component class instance record:
   type Instance is new Dv_Guidance.Base_Instance with record
      Alg : Dv_Guidance_Algorithm_Access := null;
   end record;

   ---------------------------------------
   -- Set Up Procedure
   ---------------------------------------
   -- Null method which can be implemented to provide some component
   -- set up code. This method is generally called by the assembly
   -- main.adb after all component initialization and tasks have been started.
   -- Some activities need to only be run once at startup, but cannot be run
   -- safely until everything is up and running, i.e. command registration, initial
   -- data product updates. This procedure should be implemented to do these things
   -- if necessary.
   overriding procedure Set_Up (Self : in out Instance) is null;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time and return the attitude reference it
   -- produces.
   overriding function Tick_T_Service (Self : in out Instance; Arg : in Tick.T) return Att_Ref.T;
   -- Set the burn frame from the commanded delta-V and the preferred direction. The
   -- algorithm validates them and returns Success when the frame is applied, or
   -- Failure when it rejects them, in which case the previous frame is kept, so the
   -- sender can reject its command.
   overriding function Dv_Burn_Frame_T_Service (Self : in out Instance; Arg : in Dv_Burn_Frame.T) return Packed_Command_Execution_Status.T;

end Component.Dv_Guidance.Implementation;
