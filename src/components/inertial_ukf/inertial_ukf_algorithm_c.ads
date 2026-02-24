pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Packed_F32x3_Record.C;
with Packed_F32x4_Record.C;
with Packed_F32x9_Record.C;

package Inertial_UKF_Algorithm_C is

   --* C-compatible representation of STAttInput_c.
   type ST_Att_Input is record
      Time_Tag      : aliased Short_Float;
      Mrp_Bdy_Inrtl : aliased Packed_F32x3_Record.C.U_C;
      Omega_Bn_B    : aliased Packed_F32x3_Record.C.U_C;
   end record
   with Convention => C_Pass_By_Copy;

   --* C-compatible representation of GyroInput_c.
   type Gyro_Input is record
      Gyro_B : aliased Packed_F32x3_Record.C.U_C;
   end record
   with Convention => C_Pass_By_Copy;

   --* C-compatible representation of RWSpeedsInput_c.
   type RW_Speeds_Input is record
      Wheel_Speeds : aliased Packed_F32x4_Record.C.U_C;
   end record
   with Convention => C_Pass_By_Copy;

   --* C-compatible representation of RWArrayConfigInput_c.
   type RW_Array_Config_Input is record
      Num_RW : aliased Integer_32;
   end record
   with Convention => C_Pass_By_Copy;

   --* C-compatible representation of VehicleConfigInput_c.
   --* ISCPnt_B_B is stored in column-major order to match the Eigen::Matrix3f
   --* memory layout used internally.
   type Vehicle_Config_Input is record
      ISCPnt_B_B : aliased Packed_F32x9_Record.C.U_C;
      Mass_SC    : aliased Short_Float;
   end record
   with Convention => C_Pass_By_Copy;

   --* C-compatible representation of NavAttOutput_c.
   type Nav_Att_Output is record
      Time_Tag        : aliased Long_Float;
      Sigma_Bn        : aliased Packed_F32x3_Record.C.U_C;
      Omega_Bn_B      : aliased Packed_F32x3_Record.C.U_C;
      Veh_Sun_Pnt_Bdy : aliased Packed_F32x3_Record.C.U_C;
   end record
   with Convention => C_Pass_By_Copy;

   --* C-compatible representation of InertialFilterOutput_c.
   type Inertial_Filter_Output is record
      Time_Tag : aliased Long_Float;
      Num_Obs  : aliased Integer_32;
   end record
   with Convention => C_Pass_By_Copy;

   --* Combined output of the inertial UKF algorithm update step.
   type Inertial_UKF_Output is record
      Nav_Att : aliased Nav_Att_Output;
      Filter  : aliased Inertial_Filter_Output;
   end record
   with Convention => C_Pass_By_Copy;

   --* @brief Run the inertial UKF algorithm update step.
   --* @param St_Att     Pointer to star tracker attitude input.
   --* @param Gyro       Pointer to gyro measurement input.
   --* @param Rw_Speeds  Pointer to reaction wheel speeds input.
   --* @param Rw_Config  Pointer to reaction wheel array configuration input.
   --* @param Veh_Config Pointer to vehicle configuration input.
   --* @return Combined navigation attitude and filter output.
   function Update_State
     (St_Att     : access constant ST_Att_Input;
      Gyro       : access constant Gyro_Input;
      Rw_Speeds  : access constant RW_Speeds_Input;
      Rw_Config  : access constant RW_Array_Config_Input;
      Veh_Config : access constant Vehicle_Config_Input)
     return Inertial_UKF_Output
     with Import       => True,
          Convention   => C,
          External_Name => "InertialUKFAlgorithm_updateState";

end Inertial_UKF_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
