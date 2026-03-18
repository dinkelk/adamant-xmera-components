pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings     (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Packed_F32x9_Record.C;

package Average_Mimu_Data_Algorithm_C is

   --* Opaque handle for an AverageMimuDataAlgorithm instance.
   type Average_Mimu_Data_Algorithm is limited private;
   type Average_Mimu_Data_Algorithm_Access is access all Average_Mimu_Data_Algorithm;

   -- MAX_BUF_PKT_C must match the #define in averageMimuDataAlgorithm_c.h:21
   Max_Buf_Pkt : constant := 120;

   --* POD 3-vector matching C Vector3f_c { float data[3]; }
   type Vector3f_C is array (0 .. 2) of aliased Short_Float
      with Convention => C;

   --* Array of Max_Buf_Pkt Vector3f_C elements.
   type Vector3f_C_Array is array (0 .. Max_Buf_Pkt - 1) of aliased Vector3f_C
      with Convention => C;

   --* Array of Max_Buf_Pkt timestamps.
   type Meas_Time_Array is array (0 .. Max_Buf_Pkt - 1) of aliased Unsigned_64
      with Convention => C;

   --* POD input matching C InputPktsData_c.
   type Input_Pkts_Data_C is record
      Meas_Time : Meas_Time_Array;
      Gyro_P    : Vector3f_C_Array;
      Accel_P   : Vector3f_C_Array;
   end record
      with Convention => C_Pass_By_Copy;
   type Input_Pkts_Data_C_Access is access all Input_Pkts_Data_C;

   --* POD output matching C OutputAverageAccelAngleVel_c.
   type Output_Average_Accel_Angle_Vel_C is record
      Accel_B      : Vector3f_C;
      Gyro_Omega_B : Vector3f_C;
   end record
      with Convention => C_Pass_By_Copy;

   --* @brief Get the MAX_BUF_PKT constant for Ada validation.
   --* @return The maximum buffer packet count (MAX_BUF_PKT_C).
   function Get_Max_Buf_Pkt
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_getMaxBufPkt";

   -- Runtime validation: ensure Ada constant matches C definition
   pragma Assert (Unsigned_32 (Max_Buf_Pkt) = Get_Max_Buf_Pkt);

   --* @brief Construct a new AverageMimuDataAlgorithm.
   function Create
     return Average_Mimu_Data_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_create";

   --* @brief Destroy an AverageMimuDataAlgorithm.
   procedure Destroy
     (Self : Average_Mimu_Data_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_destroy";

   --* @brief Run the update step to compute averaged MIMU data.
   --* @param Self  The algorithm instance.
   --* @param Input Pointer to input packets data (120-element buffer).
   --* @return Averaged body-frame accel and angular velocity.
   function Update
     (Self  : Average_Mimu_Data_Algorithm_Access;
      Input : Input_Pkts_Data_C_Access)
     return Output_Average_Accel_Angle_Vel_C
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_update";

   --* @brief Set the averaging window duration.
   --* @param Self   The algorithm instance.
   --* @param Window Averaging window in seconds.
   procedure Set_Averaging_Window
     (Self   : Average_Mimu_Data_Algorithm_Access;
      Window : Short_Float)
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_setAveragingWindow";

   --* @brief Get the current averaging window duration.
   --* @param Self The algorithm instance.
   --* @return The current averaging window in seconds.
   function Get_Averaging_Window
     (Self : Average_Mimu_Data_Algorithm_Access)
     return Short_Float
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_getAveragingWindow";

   --* @brief Set the DCM from platform frame to body frame.
   --* @param Self   The algorithm instance.
   --* @param Dcm_Bp 3x3 rotation matrix in row-major POD format.
   procedure Set_Dcm_Pltf_To_Bdy
     (Self   : Average_Mimu_Data_Algorithm_Access;
      Dcm_Bp : Packed_F32x9_Record.C.U_C)
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_setDcmPltfToBdy";

   --* @brief Get the current DCM from platform frame to body frame.
   --* @param Self The algorithm instance.
   --* @return 3x3 rotation matrix in row-major POD format.
   function Get_Dcm_Pltf_To_Bdy
     (Self : Average_Mimu_Data_Algorithm_Access)
     return Packed_F32x9_Record.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "AverageMimuDataAlgorithm_getDcmPltfToBdy";

private

   -- Private representation: opaque null record
   type Average_Mimu_Data_Algorithm is null record;

end Average_Mimu_Data_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings     (On, "-gnatwu");
