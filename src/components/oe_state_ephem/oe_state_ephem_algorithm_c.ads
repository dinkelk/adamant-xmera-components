pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");
-- Boolean is used at the C boundary to match the shim's C99 bool (_Bool):
-- 1-byte, 0/1 representation, interoperable under Convention => C. Suppress
-- the -gnatwx advisory about using a C "char"-style type for the mapping.
pragma Warnings (Off, "-gnatwx");

with Interfaces; use Interfaces;
with Cartesian_State.C;
with Oe_State_Ephem_Enums;
with Oe_Coefficients.C;
with Packed_F64x20.C;
with Oe_Arc.C;
with Oe_Arc_Records.C;

package Oe_State_Ephem_Algorithm_C is

   --* Opaque handle for an OEStateEphemAlgorithm instance.
   type Oe_State_Ephem_Algorithm is limited private;
   type Oe_State_Ephem_Algorithm_Access is access all Oe_State_Ephem_Algorithm;

   --* @brief Get MAX_OE_COEFF, the number of Chebyshev coefficients per
   --* orbital element, for ABI validation.
   --* @return The C-side MAX_OE_COEFF coefficients-per-element count.
   function Get_Max_Oe_Coeff return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_getMaxOeCoeff";

   --* @brief Get MAX_OE_RECORDS, the maximum number of time-segmented arc records.
   --* @return The C-side MAX_OE_RECORDS arc-table bound.
   function Get_Max_Oe_Records return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_getMaxOeRecords";

   --* @brief Get sizeof(ChebyshevFitArc_c) in bits, for per-arc ABI validation.
   --* @return The size of one C-side Chebyshev fit arc, in bits.
   function Get_Fit_Arc_Size_Bits return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_getFitArcSizeBits";

   -- ABI validation: the constant-dimensioned Ada types crossing the FFI boundary
   -- must match the C-side sizing constants, checked at elaboration.
   -- OeCoefficients: double data[MAX_OE_COEFF];
   pragma Assert (Unsigned_32 (Packed_F64x20.Length) = Get_Max_Oe_Coeff);
   pragma Assert (Packed_F64x20.C.U_C'Object_Size = Oe_Coefficients.C.U_C'Object_Size);
   pragma Assert (Unsigned_32 (Oe_Coefficients.C.U_C'Object_Size / Long_Float'Object_Size) = Get_Max_Oe_Coeff);
   -- ChebyshevFitArc_c fitCoefficients[MAX_OE_RECORDS]. The arc array crosses by
   -- reference and the C++ side reads it at fixed offsets, so a layout drift misreads
   -- data rather than failing to compile.
   pragma Assert (Unsigned_32 (Oe_Arc_Records.Length) = Get_Max_Oe_Records);
   -- This size assert is narrower than it looks: it catches a field added or removed,
   -- or a Long_Float narrowed, but not an Anomaly_Flag width change (the seven bytes
   -- of padding that follow absorb any width up to 64 bits) and not a size-preserving
   -- field reorder. Field order is guarded behaviourally by the component tests.
   pragma Assert (Oe_Arc.C.U_C'Object_Size = Get_Fit_Arc_Size_Bits);
   -- Anomaly_Flag pairs with a uint8_t-backed C enum, and the generated Adamant
   -- enumeration carries no size clause -- its 8-bit width is a GNAT default that
   -- nothing in the model pins. Assert it directly, since the size assert above
   -- cannot see it.
   pragma Assert (Oe_State_Ephem_Enums.Anomaly_Type.E'Object_Size = 8);

   --* @brief Report whether a configuration would be accepted by Create/Set_Config.
   --* @param Central_Body_Mu  [m^3/s^2] Central-body gravitational parameter.
   --* @param Number_Of_Arcs   [-] Number of populated arcs.
   --* @param Ephemeris_Time   [s] Ephemeris time offset referenced to J2000.
   --* @param Vehicle_Time     [s] Vehicle clock time offset.
   --* @param Fit_Coefficients Table of MAX_OE_RECORDS Chebyshev fit arcs.
   --* @return True if the configuration is valid. Never throws, so it can guard the
   --* throwing Create/Set_Config from an invalid configuration.
   function Validate_Config
     (Central_Body_Mu  : Long_Float;
      Number_Of_Arcs   : Unsigned_32;
      Ephemeris_Time   : Long_Float;
      Vehicle_Time     : Long_Float;
      Fit_Coefficients : access constant Oe_Arc_Records.C.U_C)
     return Boolean
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_validateConfig";

   --* @brief Construct a new OEStateEphemAlgorithm from a validated configuration.
   --* Validate the values with Validate_Config before calling; throws on invalid input.
   --* @param Central_Body_Mu  [m^3/s^2] Central-body gravitational parameter.
   --* @param Number_Of_Arcs   [-] Number of populated arcs.
   --* @param Ephemeris_Time   [s] Ephemeris time offset referenced to J2000.
   --* @param Vehicle_Time     [s] Vehicle clock time offset.
   --* @param Fit_Coefficients Table of MAX_OE_RECORDS Chebyshev fit arcs.
   --* @return The new algorithm instance, which must be released with Destroy.
   function Create
     (Central_Body_Mu  : Long_Float;
      Number_Of_Arcs   : Unsigned_32;
      Ephemeris_Time   : Long_Float;
      Vehicle_Time     : Long_Float;
      Fit_Coefficients : access constant Oe_Arc_Records.C.U_C)
     return Oe_State_Ephem_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_create";

   --* @brief Destroy an OEStateEphemAlgorithm.
   --* @param Self The algorithm instance to destroy.
   procedure Destroy
     (Self : Oe_State_Ephem_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_destroy";

   --* @brief Apply a new configuration (validated; throws on invalid input).
   --* @param Self             The algorithm instance.
   --* @param Central_Body_Mu  [m^3/s^2] Central-body gravitational parameter.
   --* @param Number_Of_Arcs   [-] Number of populated arcs.
   --* @param Ephemeris_Time   [s] Ephemeris time offset referenced to J2000.
   --* @param Vehicle_Time     [s] Vehicle clock time offset.
   --* @param Fit_Coefficients Table of MAX_OE_RECORDS Chebyshev fit arcs.
   procedure Set_Config
     (Self             : Oe_State_Ephem_Algorithm_Access;
      Central_Body_Mu  : Long_Float;
      Number_Of_Arcs   : Unsigned_32;
      Ephemeris_Time   : Long_Float;
      Vehicle_Time     : Long_Float;
      Fit_Coefficients : access constant Oe_Arc_Records.C.U_C)
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_setConfig";

   --* @brief Run the ephemeris update step.
   --* @param Self      The algorithm instance.
   --* @param Call_Time Vehicle time in nanoseconds.
   --* @return Cartesian state with position and velocity vectors.
   function Update
     (Self      : Oe_State_Ephem_Algorithm_Access;
      Call_Time : Unsigned_64)
     return Cartesian_State.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "OEStateEphemAlgorithm_update";

private

   -- Private representation: opaque null record
   type Oe_State_Ephem_Algorithm is null record;

end Oe_State_Ephem_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
pragma Warnings (On, "-gnatwx");
