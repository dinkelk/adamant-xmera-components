--------------------------------------------------------------------------------
-- Dv_Guidance Component Implementation Body
--------------------------------------------------------------------------------

with Att_Ref.C;
with Command_Enums;
with Packed_F32x3.C;
with Packed_F32x3_Record.C;

package body Component.Dv_Guidance.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the delta-V guidance algorithm with a stand-in burn frame, which the
   -- first burn command replaces.
   overriding procedure Init (Self : in out Instance) is
      -- The burn frame arrives through a connector once the assembly is running.
      -- Construct the algorithm with a stand-in frame, the delta-V along the third
      -- inertial axis and the preferred direction along the first, so a tick landing
      -- before the first burn command still produces deterministic output.
      Cmd_Dv_N : constant Packed_F32x3_Record.C.U_C := (Value => [0.0, 0.0, 1.0]);
      R_Hat_Pref_N : constant Packed_F32x3_Record.C.U_C := (Value => [1.0, 0.0, 0.0]);
   begin
      -- Create throws on an invalid configuration. The stand-in frame is a valid one, so
      -- this asserts a property of the constants rather than checking runtime input.
      pragma Assert (Validate_Config (Cmd_Dv_N => Cmd_Dv_N, R_Hat_Pref_N => R_Hat_Pref_N));
      Self.Alg := Create (Cmd_Dv_N => Cmd_Dv_N, R_Hat_Pref_N => R_Hat_Pref_N);
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time and return the attitude reference it
   -- produces.
   overriding function Tick_T_Service (Self : in out Instance; Arg : in Tick.T) return Att_Ref.T is
      Ignore : Tick.T renames Arg;
   begin
      -- The burn frame is inertially fixed, so the algorithm takes no per-tick input
      -- and returns the reference with its rate and acceleration zero.
      return Att_Ref.C.Pack (Update (Self.Alg));
   end Tick_T_Service;

   -- Set the burn frame from the commanded delta-V and the preferred direction. The
   -- algorithm validates them and returns Success when the frame is applied, or
   -- Failure when it rejects them, in which case the previous frame is kept, so the
   -- sender can reject its command.
   overriding function Dv_Burn_Frame_T_Service (Self : in out Instance; Arg : in Dv_Burn_Frame.T) return Packed_Command_Execution_Status.T is
      Cmd_Dv_N : constant Packed_F32x3_Record.C.U_C := (Value => Packed_F32x3.C.Unpack (Arg.Cmd_Dv_N));
      R_Hat_Pref_N : constant Packed_F32x3_Record.C.U_C := (Value => Packed_F32x3.C.Unpack (Arg.R_Hat_Pref_N));
   begin
      -- Set_Config throws on an invalid configuration, so the algorithm's own
      -- non-throwing predicate decides first, and the frame rules live solely in the
      -- algorithm.
      if Validate_Config (Cmd_Dv_N => Cmd_Dv_N, R_Hat_Pref_N => R_Hat_Pref_N) then
         Set_Config (Self.Alg, Cmd_Dv_N => Cmd_Dv_N, R_Hat_Pref_N => R_Hat_Pref_N);
         return (Status => Command_Enums.Command_Execution_Status.Success);
      else
         return (Status => Command_Enums.Command_Execution_Status.Failure);
      end if;
   end Dv_Burn_Frame_T_Service;

end Component.Dv_Guidance.Implementation;
