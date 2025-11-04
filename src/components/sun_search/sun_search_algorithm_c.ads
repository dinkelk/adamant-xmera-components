pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings     (Off, "-gnatwu");

with Interfaces.C;     use Interfaces; use Interfaces.C;
with Att_Guid.C;
with Nav_Att.C;
with Slew_Properties.C;
with Vehicle_Config.C;

package Sun_Search_Algorithm_C is

   -- NUM_SLEWS must match the #define in sunSearchTypes.h:11
   -- Re-run h2ads if the C header changes to regenerate this binding
   NUM_SLEWS : constant := 3;

   --* @brief Get the maximum number of slews constant for validation.
   --* @return The maximum number of slews (NUM_SLEWS).
   function Get_Num_Slews
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_getNumSlews";

   -- Runtime validation: ensure Ada constant matches C definition
   pragma Assert (Unsigned_32 (NUM_SLEWS) = Get_Num_Slews);

   --* Opaque handle for a SunSearchAlgorithm instance.
   type Sun_Search_Algorithm is limited private;
   type Sun_Search_Algorithm_Access is access all Sun_Search_Algorithm;

   --* @brief Construct a new SunSearchAlgorithm.
   function Create
     return Sun_Search_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_create";

   --* @brief Destroy a SunSearchAlgorithm.
   procedure Destroy
     (Self : Sun_Search_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_destroy";

   --* @brief Reset the algorithm state.
   --* @param Self               The algorithm instance.
   --* @param Current_Sim_Nanos  Time stamp for reset in nanoseconds.
   --* @param Vehicle_Config_In  Pointer to vehicle configuration message payload.
   procedure Reset
     (Self               : Sun_Search_Algorithm_Access;
      Current_Sim_Nanos  : Unsigned_64;
      Vehicle_Config_In  : Vehicle_Config.C.U_C_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_reset";

   --* @brief Run the update step to compute guidance message.
   --* @param Self               The algorithm instance.
   --* @param Current_Sim_Nanos  Time stamp for update in nanoseconds.
   --* @param Nav_Att_In         Pointer to navigation attitude message payload.
   --* @return Computed attitude guidance message.
   function Update
     (Self               : Sun_Search_Algorithm_Access;
      Current_Sim_Nanos  : Unsigned_64;
      Nav_Att_In         : Nav_Att.C.U_C_Access)
     return Att_Guid.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_update";

   --* @brief Set the properties of a slew maneuver (adds to internal array).
   --* @param Self                  The algorithm instance.
   --* @param Slew_Properties_Input The slew properties to add.
   procedure Set_Slew_Properties
     (Self                  : Sun_Search_Algorithm_Access;
      Slew_Properties_Input : access constant Slew_Properties.C.U_C)
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_setSlewProperties";

   --* @brief Modify the properties of an existing slew maneuver.
   --* @param Self                  The algorithm instance.
   --* @param Slew_Properties_Input The new slew properties.
   --* @param Index                 Index of the slew maneuver to modify.
   procedure Modify_Slew_Properties
     (Self                  : Sun_Search_Algorithm_Access;
      Slew_Properties_Input : access constant Slew_Properties.C.U_C;
      Index                 : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_modifySlewProperties";

   --* @brief Get the properties of a slew maneuver.
   --* @param Self  The algorithm instance.
   --* @param Index Index of the slew maneuver to retrieve.
   --* @return The properties of the slew maneuver.
   function Get_Slew_Properties
     (Self  : Sun_Search_Algorithm_Access;
      Index : Unsigned_32)
     return Slew_Properties.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "SunSearchAlgorithm_getSlewProperties";

private

   -- Private representation: opaque null record
   type Sun_Search_Algorithm is null record;

end Sun_Search_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings     (On, "-gnatwu");
