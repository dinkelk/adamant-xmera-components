pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings     (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Sunline_Srukf_Input.C;
with Sunline_Srukf_Output.C;

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
      Input : Sunline_Srukf_Input.C.U_C_Access)
     return Sunline_Srukf_Output.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineSRuKFAlgorithm_updateState";

private

   -- Private representation: opaque null record
   type Sunline_Srukf_Algorithm is null record;

end Sunline_Srukf_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings     (On, "-gnatwu");
