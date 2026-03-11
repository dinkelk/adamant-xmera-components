pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Packed_F32x3_Record.C;

package Body_Rate_Miscompare_Algorithm_C is

   --* Opaque handle for a BodyRateMiscompareAlgorithm instance.
   type Body_Rate_Miscompare_Algorithm is limited private;
   type Body_Rate_Miscompare_Algorithm_Access is access all Body_Rate_Miscompare_Algorithm;

   --* POD output type matching BodyRateMiscompareOutput_c in C.
   --* Layout: float omega_BN_B[3]; bool bodyRateFaultDetected;
   type Body_Rate_Miscompare_Output_C is record
      Omega_Bn_B : aliased Packed_F32x3_Record.C.U_C;
      Body_Rate_Fault_Detected : aliased C.unsigned_char;
   end record
      with Convention => C_Pass_By_Copy;

   --* @brief Construct a new BodyRateMiscompareAlgorithm.
   function Create
     return Body_Rate_Miscompare_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "BodyRateMiscompareAlgorithm_create";

   --* @brief Destroy a BodyRateMiscompareAlgorithm.
   procedure Destroy
     (Self : Body_Rate_Miscompare_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "BodyRateMiscompareAlgorithm_destroy";

   --* @brief Run the update step.
   --* @param Self      The algorithm instance.
   --* @param Imu_Omega IMU body rate vector (Vector3f_c).
   --* @param St_Omega  Star tracker body rate vector (Vector3f_c).
   --* @return BodyRateMiscompareOutput_c  The computed output.
   function Update
     (Self      : Body_Rate_Miscompare_Algorithm_Access;
      Imu_Omega : Packed_F32x3_Record.C.U_C;
      St_Omega  : Packed_F32x3_Record.C.U_C)
     return Body_Rate_Miscompare_Output_C
     with Import       => True,
          Convention   => C,
          External_Name => "BodyRateMiscompareAlgorithm_update";

   --* @brief Set the body rate threshold.
   --* @param Self  The algorithm instance.
   --* @param Value The new body rate threshold value.
   procedure Set_Body_Rate_Threshold
     (Self  : Body_Rate_Miscompare_Algorithm_Access;
      Value : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "BodyRateMiscompareAlgorithm_setBodyRateThreshold";

   --* @brief Get the current body rate threshold.
   --* @param Self  The algorithm instance.
   --* @return The current body rate threshold.
   function Get_Body_Rate_Threshold
     (Self : Body_Rate_Miscompare_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "BodyRateMiscompareAlgorithm_getBodyRateThreshold";

private

   -- Private representation: opaque null record
   type Body_Rate_Miscompare_Algorithm is null record;

end Body_Rate_Miscompare_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");