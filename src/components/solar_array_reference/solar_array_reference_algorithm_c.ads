pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");
-- Boolean is used at the C boundary to match the shim's C99 bool (_Bool):
-- 1-byte, 0/1 representation, interoperable under Convention => C. Suppress
-- the -gnatwx advisory about using a C "char"-style type for the mapping.
pragma Warnings (Off, "-gnatwx");

with Interfaces; use Interfaces;
with Packed_F32x3_Record.C;
with Solar_Array_Reference_Enums;

package Solar_Array_Reference_Algorithm_C is

   --* Tracking mode. The representation clause pins the literals to the C TrackingMode
   --* values so that 'Enum_Val is a genuine validity gate when converting the parameter
   --* value into this type.
   type Solar_Array_Tracking_Mode is
     (Auto_Track,
      Specified_Angle)
     with Convention => C;
   for Solar_Array_Tracking_Mode use
     (Auto_Track      => 0,
      Specified_Angle => 1);

   --* Convert the component's tracking mode parameter value into the C enumeration
   --* above. The conversion lives here, with the C type, because it is boundary
   --* marshalling: the two enumerations exist separately only because the generated
   --* Adamant enumeration cannot carry Convention => C, and so is sized for Ada rather
   --* than for the C int the shim expects. Going through 'Enum_Rep and 'Enum_Val
   --* honors both representation clauses rather than relying on literal position.
   function To_C (Value : in Solar_Array_Reference_Enums.Tracking_Mode.E)
      return Solar_Array_Tracking_Mode
   is (Solar_Array_Tracking_Mode'Enum_Val (Solar_Array_Reference_Enums.Tracking_Mode.E'Enum_Rep (Value)));

   --* Opaque handle for a SolarArrayReferenceAlgorithm instance.
   type Solar_Array_Reference_Algorithm is limited private;
   type Solar_Array_Reference_Algorithm_Access is access all Solar_Array_Reference_Algorithm;

   --* @brief Report whether a configuration would be accepted by Create/Set_Config.
   --* @param Drive_Axis            [-]   Solar array drive axis in body frame; finite, near unit, orthogonal
   --*                                    to the surface normal.
   --* @param Surface_Normal        [-]   Solar array surface normal at zero rotation; finite, near unit,
   --*                                    orthogonal to the drive axis.
   --* @param Alignment_Threshold   [rad] Alignment threshold between the sun direction and the drive axis;
   --*                                    in [1e-3, pi/2].
   --* @param Tracking_Mode         [-]   Array tracking mode.
   --* @param Specified_Array_Angle [rad] Reference array angle used in the specified angle mode; in [-pi, pi].
   --* @param Offset_Angle          [rad] Offset added to the determined reference angle; in [-pi, pi].
   --* @return True if the configuration is valid. Never throws, so it can guard the
   --* throwing Create/Set_Config from an invalid configuration.
   function Validate_Config
     (Drive_Axis            : access constant Packed_F32x3_Record.C.U_C;
      Surface_Normal        : access constant Packed_F32x3_Record.C.U_C;
      Alignment_Threshold   : Short_Float;
      Tracking_Mode         : Solar_Array_Tracking_Mode;
      Specified_Array_Angle : Short_Float;
      Offset_Angle          : Short_Float)
     return Boolean
     with Import        => True,
          Convention    => C,
          External_Name => "SolarArrayReferenceAlgorithm_validateConfig";

   --* @brief Construct a new SolarArrayReferenceAlgorithm from a configuration.
   --* Validate the values with Validate_Config before calling; throws on invalid input.
   --* The parameters are as for Validate_Config.
   --* @return The new algorithm instance, which must be released with Destroy.
   function Create
     (Drive_Axis            : access constant Packed_F32x3_Record.C.U_C;
      Surface_Normal        : access constant Packed_F32x3_Record.C.U_C;
      Alignment_Threshold   : Short_Float;
      Tracking_Mode         : Solar_Array_Tracking_Mode;
      Specified_Array_Angle : Short_Float;
      Offset_Angle          : Short_Float)
     return Solar_Array_Reference_Algorithm_Access
     with Import        => True,
          Convention    => C,
          External_Name => "SolarArrayReferenceAlgorithm_create";

   --* @brief Destroy a SolarArrayReferenceAlgorithm.
   --* @param Self The algorithm instance to destroy.
   procedure Destroy
     (Self : Solar_Array_Reference_Algorithm_Access)
     with Import        => True,
          Convention    => C,
          External_Name => "SolarArrayReferenceAlgorithm_destroy";

   --* @brief Apply a new configuration (validated; throws on invalid input). The algorithm
   --* holds no runtime state. The parameters are as for Validate_Config.
   --* @param Self The algorithm instance.
   procedure Set_Config
     (Self                  : Solar_Array_Reference_Algorithm_Access;
      Drive_Axis            : access constant Packed_F32x3_Record.C.U_C;
      Surface_Normal        : access constant Packed_F32x3_Record.C.U_C;
      Alignment_Threshold   : Short_Float;
      Tracking_Mode         : Solar_Array_Tracking_Mode;
      Specified_Array_Angle : Short_Float;
      Offset_Angle          : Short_Float)
     with Import        => True,
          Convention    => C,
          External_Name => "SolarArrayReferenceAlgorithm_setConfig";

   --* @brief Compute the reference array angle.
   --* @param Self          The algorithm instance.
   --* @param Sigma_Bn      [-]   Body attitude MRP relative to inertial.
   --* @param Sigma_Rn      [-]   Reference attitude MRP relative to inertial.
   --* @param R_Hat_In_Sb_B [-]   Sun direction in body frame components.
   --* @param Theta         [rad] Current array rotation angle.
   --* @return [rad] Reference array angle wrapped to [-pi, pi].
   function Update
     (Self          : Solar_Array_Reference_Algorithm_Access;
      Sigma_Bn      : Packed_F32x3_Record.C.U_C;
      Sigma_Rn      : Packed_F32x3_Record.C.U_C;
      R_Hat_In_Sb_B : Packed_F32x3_Record.C.U_C;
      Theta         : Short_Float)
     return Short_Float
     with Import        => True,
          Convention    => C,
          External_Name => "SolarArrayReferenceAlgorithm_update";

private

   -- Private representation: opaque null record
   type Solar_Array_Reference_Algorithm is null record;

end Solar_Array_Reference_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
pragma Warnings (On, "-gnatwx");
