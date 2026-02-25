pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings     (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Packed_F32x3_Record.C;

package Sunline_Srukf_Algorithm_C is

   -- MAX_NUM_CSS must match the #define in sunlineSRuKFAlgorithm_c.h:12
   -- Re-run h2ads if the C header changes to regenerate this binding
   MAX_NUM_CSS : constant := 32;

   --* @brief Get the maximum number of CSS sensors.
   --* @return The maximum CSS count (SUNLINE_SRUKF_MAX_NUM_CSS).
   function Get_Max_Num_Css
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineSRuKFAlgorithm_getMaxNumCss";

   -- Runtime validation: ensure Ada constant matches C definition
   pragma Assert (Unsigned_32 (MAX_NUM_CSS) = Get_Max_Num_Css);

   --* Opaque handle for a SunlineSRuKFAlgorithm instance.
   type Sunline_Srukf_Algorithm is limited private;
   type Sunline_Srukf_Algorithm_Access is access all Sunline_Srukf_Algorithm;

   --* Array type for CSS cosine measurement values.
   type Cos_Values_Array is array (0 .. MAX_NUM_CSS - 1) of aliased Short_Float
     with Convention => C;

   --* C-compatible input structure for the sunline SRuKF algorithm.
   type Sunline_Srukf_Input is record
      Time_Tag        : aliased Long_Float;
      Sigma_BN        : aliased Packed_F32x3_Record.C.U_C;
      Omega_BN_B      : aliased Packed_F32x3_Record.C.U_C;
      Veh_Sun_Pnt_Bdy : aliased Packed_F32x3_Record.C.U_C;
      N_CSS           : aliased Unsigned_32;
      Cos_Values      : aliased Cos_Values_Array;
   end record
   with Convention => C_Pass_By_Copy;

   type Sunline_Srukf_Input_Access is access all Sunline_Srukf_Input;

   --* C-compatible output structure for the sunline SRuKF algorithm.
   type Sunline_Srukf_Output is record
      Time_Tag        : aliased Long_Float;
      Sigma_BN        : aliased Packed_F32x3_Record.C.U_C;
      Omega_BN_B      : aliased Packed_F32x3_Record.C.U_C;
      Veh_Sun_Pnt_Bdy : aliased Packed_F32x3_Record.C.U_C;
   end record
   with Convention => C_Pass_By_Copy;

   --* @brief Construct a new SunlineSRuKFAlgorithm.
   function Create
     return Sunline_Srukf_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineSRuKFAlgorithm_create";

   --* @brief Destroy a SunlineSRuKFAlgorithm.
   procedure Destroy
     (Self : Sunline_Srukf_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineSRuKFAlgorithm_destroy";

   --* @brief Run the sunline SRuKF update step.
   --* @param Self  The algorithm instance.
   --* @param Input Pointer to the input structure (read-only).
   --* @return The computed output.
   function Update_State
     (Self  : Sunline_Srukf_Algorithm_Access;
      Input : Sunline_Srukf_Input_Access)
     return Sunline_Srukf_Output
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineSRuKFAlgorithm_updateState";

private

   -- Private representation: opaque null record
   type Sunline_Srukf_Algorithm is null record;

end Sunline_Srukf_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings     (On, "-gnatwu");
