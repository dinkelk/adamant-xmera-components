--------------------------------------------------------------------------------
-- Stepper_Motor_Controller Component Implementation Spec
--------------------------------------------------------------------------------

with Tick;
with Parameter_Update;
with Motor_Step_Command;
with Stepper_Motor_Controller_Algorithm_C; use Stepper_Motor_Controller_Algorithm_C;

-- Stepper motor controller algorithm computes commanded motor steps from the
-- reference angle input.
package Component.Stepper_Motor_Controller.Implementation is

   -- The component class instance record:
   type Instance is new Stepper_Motor_Controller.Base_Instance with private;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the stepper motor controller algorithm.
   overriding procedure Init (Self : in out Instance);
   not overriding procedure Destroy (Self : in out Instance);

private

   -- The component class instance record:
   type Instance is new Stepper_Motor_Controller.Base_Instance with record
      Alg : Stepper_Motor_Controller_Algorithm_Access := null;
   end record;

   ---------------------------------------
   -- Set Up Procedure
   ---------------------------------------
   overriding procedure Set_Up (Self : in out Instance);

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T);
   overriding procedure Parameter_Update_T_Modify (Self : in out Instance; Arg : in out Parameter_Update.T);

   ---------------------------------------
   -- Invoker connector primitives:
   ---------------------------------------
   overriding procedure Motor_Step_Command_T_Send_Dropped (Self : in out Instance; Arg : in Motor_Step_Command.T) is null;

   -----------------------------------------------
   -- Parameter primitives:
   -----------------------------------------------
   overriding procedure Invalid_Parameter (
      Self               : in out Instance;
      Par                : in Parameter.T;
      Errant_Field_Number : in Unsigned_32;
      Errant_Field       : in Basic_Types.Poly_Type
   );

   overriding procedure Update_Parameters_Action (Self : in out Instance);

   overriding function Validate_Parameters (
      Self      : in out Instance;
      Theta_Max : in Packed_F32.U;
      Theta_Min : in Packed_F32.U;
      Step_Angle : in Packed_F32.U;
      Step_Time : in Packed_F32.U
   ) return Parameter_Validation_Status.E is (Parameter_Validation_Status.Valid);

   -----------------------------------------------
   -- Data dependency primitives:
   -----------------------------------------------
   overriding function Get_Data_Dependency (
      Self : in out Instance;
      Id   : in Data_Product_Types.Data_Product_Id
   ) return Data_Product_Return.T;

   overriding procedure Invalid_Data_Dependency (
      Self : in out Instance;
      Id   : in Data_Product_Types.Data_Product_Id;
      Ret  : in Data_Product_Return.T
   );

end Component.Stepper_Motor_Controller.Implementation;
