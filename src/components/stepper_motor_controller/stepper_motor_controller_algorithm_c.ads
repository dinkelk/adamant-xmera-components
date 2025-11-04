pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces; use Interfaces;
with Hinged_Rigid_Body.C;
with Stepper_Motor_Controller_Output.C;

package Stepper_Motor_Controller_Algorithm_C is

   --* Opaque handle for a StepperMotorControllerAlgorithm instance.
   type Stepper_Motor_Controller_Algorithm is limited private;
   type Stepper_Motor_Controller_Algorithm_Access is access all Stepper_Motor_Controller_Algorithm;

   --* @brief Construct a new StepperMotorControllerAlgorithm.
   --* @return Pointer to a newly allocated algorithm instance.
   function Create
     return Stepper_Motor_Controller_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_create";

   --* @brief Destroy a StepperMotorControllerAlgorithm.
   --* @param Self Pointer to the instance to destroy.
   procedure Destroy
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_destroy";

   --* @brief Reset the algorithm state.
   --* @param Self Pointer to the instance.
   procedure Reset
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_reset";

   --* @brief Run the update step to compute motor step commands.
   --* @param Self Pointer to the instance.
   --* @param Call_Time Time stamp for the update in nanoseconds.
   --* @param Hinged_Rigid_Body_Msg_Time_Written Time (seconds) the input message was written.
   --* @param Motor_Ref_Angle_In Pointer to the motor reference angle payload.
   --* @return Stepper motor controller output data.
   function Update
     (Self                           : Stepper_Motor_Controller_Algorithm_Access;
      Call_Time                      : Unsigned_64;
      Hinged_Rigid_Body_Msg_Time_Written : Short_Float;
      Motor_Ref_Angle_In             : Hinged_Rigid_Body.C.U_C_Access)
     return Stepper_Motor_Controller_Output.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_update";

   --* @brief Set the initial motor angle.
   --* @param Self Pointer to the instance.
   --* @param Theta_Init Initial motor angle in radians.
   procedure Set_Theta_Init
     (Self       : Stepper_Motor_Controller_Algorithm_Access;
      Theta_Init : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_setThetaInit";

   --* @brief Get the initial motor angle.
   --* @param Self Pointer to the instance.
   --* @return Current initial motor angle in radians.
   function Get_Theta_Init
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_getThetaInit";

   --* @brief Set the motor upper actuation limit.
   --* @param Self Pointer to the instance.
   --* @param Theta_Max Motor upper actuation limit in radians.
   procedure Set_Theta_Max
     (Self      : Stepper_Motor_Controller_Algorithm_Access;
      Theta_Max : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_setThetaMax";

   --* @brief Get the motor upper actuation limit.
   --* @param Self Pointer to the instance.
   --* @return Current motor upper actuation limit in radians.
   function Get_Theta_Max
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_getThetaMax";

   --* @brief Set the motor lower actuation limit.
   --* @param Self Pointer to the instance.
   --* @param Theta_Min Motor lower actuation limit in radians.
   procedure Set_Theta_Min
     (Self      : Stepper_Motor_Controller_Algorithm_Access;
      Theta_Min : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_setThetaMin";

   --* @brief Get the motor lower actuation limit.
   --* @param Self Pointer to the instance.
   --* @return Current motor lower actuation limit in radians.
   function Get_Theta_Min
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_getThetaMin";

   --* @brief Set the motor step angle.
   --* @param Self Pointer to the instance.
   --* @param Step_Angle Motor step angle in radians.
   procedure Set_Step_Angle
     (Self       : Stepper_Motor_Controller_Algorithm_Access;
      Step_Angle : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_setStepAngle";

   --* @brief Get the motor step angle.
   --* @param Self Pointer to the instance.
   --* @return Current motor step angle in radians.
   function Get_Step_Angle
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_getStepAngle";

   --* @brief Set the motor step time.
   --* @param Self Pointer to the instance.
   --* @param Step_Time Motor step time in seconds.
   procedure Set_Step_Time
     (Self      : Stepper_Motor_Controller_Algorithm_Access;
      Step_Time : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_setStepTime";

   --* @brief Get the motor step time.
   --* @param Self Pointer to the instance.
   --* @return Current motor step time in seconds.
   function Get_Step_Time
     (Self : Stepper_Motor_Controller_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "StepperMotorControllerAlgorithm_getStepTime";

private

   -- Private representation: opaque null record
   type Stepper_Motor_Controller_Algorithm is null record;

end Stepper_Motor_Controller_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
