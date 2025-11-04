--------------------------------------------------------------------------------
-- Stepper_Motor_Controller Component Implementation Body
--------------------------------------------------------------------------------

with Interfaces;
with Sys_Time;
with Hinged_Rigid_Body;
with Hinged_Rigid_Body.C;
with Stepper_Motor_Controller_Output;
with Stepper_Motor_Controller_Output.C;
with Algorithm_Wrapper_Util;

package body Component.Stepper_Motor_Controller.Implementation is

   Nanoseconds_Per_Second : constant Interfaces.Unsigned_64 := 1_000_000_000;
   Subsecond_Divisor      : constant Interfaces.Unsigned_64 := 2 ** 16;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   overriding procedure Init (Self : in out Instance) is
   begin
      Self.Alg := Stepper_Motor_Controller_Algorithm_C.Create;
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      if Self.Alg /= null then
         Stepper_Motor_Controller_Algorithm_C.Destroy (Self.Alg);
         Self.Alg := null;
      end if;
   end Destroy;

   ---------------------------------------
   -- Set Up Procedure
   ---------------------------------------
   overriding procedure Set_Up (Self : in out Instance) is
      use Data_Product_Enums;

      pragma Assert (Self.Alg /= null, "Stepper_Motor_Controller.Set_Up: Algorithm instance not allocated");

      Motor_Ref : Hinged_Rigid_Body.T;
      Status    : constant Data_Dependency_Status.E :=
         Self.Get_Motor_Reference_Angle (
            Stale_Reference => (Seconds => 0, Subseconds => 0),
            Value           => Motor_Ref
         );
   begin
      if Algorithm_Wrapper_Util.Is_Dep_Status_Success (Status) then
         declare
            Motor_Ref_Unpacked : constant Hinged_Rigid_Body.U := Hinged_Rigid_Body.Unpack (Motor_Ref);
         begin
            Stepper_Motor_Controller_Algorithm_C.Set_Theta_Init (Self.Alg, Motor_Ref_Unpacked.Theta);
            Update_Parameters_Action (Self);
         end;
      else
         pragma Annotate (GNATSAS, Intentional, "subp always fails", "missing motor reference during setup");
         pragma Assert (False, "Stepper_Motor_Controller.Set_Up: Failed to fetch Motor_Reference_Angle data dependency");
      end if;
   end Set_Up;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      use Data_Product_Enums;

      Motor_Ref_Time : Sys_Time.T;
      Motor_Ref      : Hinged_Rigid_Body.T;
      Motor_Ref_Status : constant Data_Dependency_Status.E :=
         Self.Get_Motor_Reference_Angle (
            Stale_Reference => Arg.Time,
            Timestamp       => Motor_Ref_Time,
            Value           => Motor_Ref
         );
   begin
      pragma Assert (Self.Alg /= null, "Stepper_Motor_Controller.Tick: Algorithm instance not allocated");

      Self.Update_Parameters;

      if Algorithm_Wrapper_Util.Is_Dep_Status_Success (Motor_Ref_Status) then
         declare
            Motor_Ref_Unpacked : constant Hinged_Rigid_Body.U := Hinged_Rigid_Body.Unpack (Motor_Ref);
            Motor_Ref_C        : aliased Hinged_Rigid_Body.C.U_C := Hinged_Rigid_Body.C.To_C (Motor_Ref_Unpacked);

            Arg_Time_Unpacked : constant Sys_Time.U := Sys_Time.Unpack (Arg.Time);
            Call_Time_Ns : constant Interfaces.Unsigned_64 :=
              Interfaces.Unsigned_64 (Arg_Time_Unpacked.Seconds) * Nanoseconds_Per_Second +
              Interfaces.Unsigned_64 (Arg_Time_Unpacked.Subseconds) * Nanoseconds_Per_Second / Subsecond_Divisor;

            Motor_Ref_Time_Unpacked : constant Sys_Time.U := Sys_Time.Unpack (Motor_Ref_Time);
            Message_Time_Seconds : constant Short_Float :=
              Short_Float (
                Long_Float (Motor_Ref_Time_Unpacked.Seconds) +
                Long_Float (Motor_Ref_Time_Unpacked.Subseconds) / 65536.0);

            Output_C : constant Stepper_Motor_Controller_Output.C.U_C := Stepper_Motor_Controller_Algorithm_C.Update (
               Self.Alg,
               Call_Time                      => Call_Time_Ns,
               Hinged_Rigid_Body_Msg_Time_Written => Message_Time_Seconds,
               Motor_Ref_Angle_In             => Motor_Ref_C'Unchecked_Access
            );

            Output_Ada : constant Stepper_Motor_Controller_Output.U :=
              Stepper_Motor_Controller_Output.C.To_Ada (Output_C);
         begin
            if Output_Ada.Write_Output_Message /= 0 then
               declare
                  Motor_Cmd : constant Motor_Step_Command.T :=
                    Motor_Step_Command.Pack (Output_Ada.Motor_Step_Command_Out);
               begin
                  Self.Motor_Step_Command_T_Send (Motor_Cmd);
               end;
            end if;
         end;
      end if;
   end Tick_T_Recv_Sync;

   overriding procedure Parameter_Update_T_Modify (Self : in out Instance; Arg : in out Parameter_Update.T) is
   begin
      Self.Process_Parameter_Update (Arg);
   end Parameter_Update_T_Modify;

   -----------------------------------------------
   -- Parameter handlers:
   -----------------------------------------------
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
   begin
      pragma Assert (Self.Alg /= null, "Stepper_Motor_Controller.Update_Parameters_Action: Algorithm instance not allocated");

      Stepper_Motor_Controller_Algorithm_C.Set_Theta_Max (Self.Alg, Self.Theta_Max.Value);
      Stepper_Motor_Controller_Algorithm_C.Set_Theta_Min (Self.Alg, Self.Theta_Min.Value);
      Stepper_Motor_Controller_Algorithm_C.Set_Step_Angle (Self.Alg, Self.Step_Angle.Value);
      Stepper_Motor_Controller_Algorithm_C.Set_Step_Time (Self.Alg, Self.Step_Time.Value);
   end Update_Parameters_Action;

   overriding procedure Invalid_Parameter (
      Self               : in out Instance;
      Par                : in Parameter.T;
      Errant_Field_Number : in Unsigned_32;
      Errant_Field       : in Basic_Types.Poly_Type
   ) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "unexpected invalid parameter");
   begin
      pragma Assert (False);
   end Invalid_Parameter;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   overriding function Get_Data_Dependency (
      Self : in out Instance;
      Id   : in Data_Product_Types.Data_Product_Id
   ) return Data_Product_Return.T is
   begin
      return Self.Data_Product_Fetch_T_Request ((Id => Id));
   end Get_Data_Dependency;

   overriding procedure Invalid_Data_Dependency (
      Self : in out Instance;
      Id   : in Data_Product_Types.Data_Product_Id;
      Ret  : in Data_Product_Return.T
   ) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "unexpected invalid data dependency");
   begin
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Stepper_Motor_Controller.Implementation;
