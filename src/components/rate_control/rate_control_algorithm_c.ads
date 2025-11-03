pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Att_Guid.C;
with Vehicle_Config.C;
with Packed_F32x3_Record.C;

package Rate_Control_Algorithm_C is

   --* Opaque handle for a RateControlAlgorithm instance.
   type Rate_Control_Algorithm is limited private;
   type Rate_Control_Algorithm_Access is access all Rate_Control_Algorithm;

   --* @brief Construct a new RateControlAlgorithm.
   function Create
     return Rate_Control_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_create";

   --* @brief Destroy a RateControlAlgorithm.
   procedure Destroy
     (Self : Rate_Control_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_destroy";

   --* @brief Compute control torque from attitude guidance.
   --* @param Self         The algorithm instance.
   --* @param Att_Guid_In  Pointer to attitude guidance message payload.
   --* @return Command torque message.
   function Update
     (Self        : Rate_Control_Algorithm_Access;
      Att_Guid_In : Att_Guid.C.U_C_Access)
     return Packed_F32x3_Record.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_update";

   --* @brief Set spacecraft inertia from vehicle configuration.
   --* @param Self              The algorithm instance.
   --* @param Vehicle_Config_In Pointer to vehicle config message payload.
   procedure Set_Spacecraft_Inertia
     (Self              : Rate_Control_Algorithm_Access;
      Vehicle_Config_In : Vehicle_Config.C.U_C_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_setSpacecraftInertia";

   --* @brief Set the derivative gain P.
   --* @param Self  The algorithm instance.
   --* @param P     [N*m*s] Rate error feedback gain.
   procedure Set_Derivative_Gain_P
     (Self : Rate_Control_Algorithm_Access;
      P    : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_setDerivativeGainP";

   --* @brief Get the derivative gain P.
   --* @param Self  The algorithm instance.
   --* @return [N*m*s] The current derivative gain.
   function Get_Derivative_Gain_P
     (Self : Rate_Control_Algorithm_Access)
     return Short_Float 
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_getDerivativeGainP";

   --* @brief Set the known external torque about point B.
   --* @param Self                  The algorithm instance.
   --* @param Known_Torque_Pnt_B_B  [N*m] Known external torque in body frame.
   procedure Set_Known_Torque_Pnt_B_B
     (Self                  : Rate_Control_Algorithm_Access;
      Known_Torque_Pnt_B_B : Packed_F32x3_Record.C.U_C)
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_setKnownTorquePntB_B";

   --* @brief Get the known external torque about point B.
   --* @param Self  The algorithm instance.
   --* @return [N*m] The known external torque in body frame.
   function Get_Known_Torque_Pnt_B_B
     (Self : Rate_Control_Algorithm_Access)
     return Packed_F32x3_Record.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "RateControlAlgorithm_getKnownTorquePntB_B";

private

   -- Private representation: opaque null record
   type Rate_Control_Algorithm is null record;

end Rate_Control_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
