pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings     (Off, "-gnatwu");

with Interfaces.C;              use Interfaces; use Interfaces.C;
with Mimu_Majority_Vote_Output.C;
with Packed_F32x3_Record.C;

package Mimu_Majority_Vote_Algorithm_C is

   -- MAX_IMU_VEH_COUNT must match the constexpr in mimuMajorityVoteTypes.h:10
   -- Re-run h2ads if the C header changes to regenerate this binding
   MAX_IMU_VEH_COUNT : constant := 4;

   --* @brief Get the maximum IMU vehicle count constant for validation.
   --* @return The maximum IMU count (MAX_IMU_VEH_COUNT = 4).
   function Get_Max_Imu_Veh_Count
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "MimuMajorityVoteAlgorithm_getMaxImuVehCount";

   -- Runtime validation: ensure Ada constant matches C definition
   pragma Assert (Unsigned_32 (MAX_IMU_VEH_COUNT) = Get_Max_Imu_Veh_Count);

   --* Opaque handle for a MimuMajorityVoteAlgorithm instance.
   type Mimu_Majority_Vote_Algorithm is limited private;
   type Mimu_Majority_Vote_Algorithm_Access is access all Mimu_Majority_Vote_Algorithm;

   --* @brief Construct a new MimuMajorityVoteAlgorithm.
   function Create
     return Mimu_Majority_Vote_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "MimuMajorityVoteAlgorithm_create";

   --* @brief Destroy a MimuMajorityVoteAlgorithm.
   procedure Destroy
     (Self : Mimu_Majority_Vote_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "MimuMajorityVoteAlgorithm_destroy";

   --* @brief Run the majority vote update step.
   --* @param Self           The algorithm instance.
   --* @param Imu_Inputs     Pointer to array of IMU input structs (max MAX_IMU_VEH_COUNT).
   --* @param Number_Of_Imus Number of valid IMU inputs in the array.
   --* @return The computed majority vote output.
   function Update
     (Self           : Mimu_Majority_Vote_Algorithm_Access;
      Imu_Inputs     : Packed_F32x3_Record.C.U_C_Access;
      Number_Of_Imus : Unsigned_32)
     return Mimu_Majority_Vote_Output.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "MimuMajorityVoteAlgorithm_update";

   --* @brief Set the omega threshold for fault detection.
   --* @param Self  The algorithm instance.
   --* @param Value The new omega threshold value (must be positive).
   procedure Set_Omega_Threshold
     (Self  : Mimu_Majority_Vote_Algorithm_Access;
      Value : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "MimuMajorityVoteAlgorithm_setOmegaThreshold";

   --* @brief Get the current omega threshold.
   --* @param Self The algorithm instance.
   --* @return The current omega threshold.
   function Get_Omega_Threshold
     (Self : Mimu_Majority_Vote_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "MimuMajorityVoteAlgorithm_getOmegaThreshold";

private

   -- Private representation: opaque null record
   type Mimu_Majority_Vote_Algorithm is null record;

end Mimu_Majority_Vote_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings     (On, "-gnatwu");
