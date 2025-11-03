pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Ephemeris.C;
with Nav_Trans.C;

package Ephem_Nav_Converter_Algorithm_C is

   --* Opaque handle for an EphemNavConverterAlgorithm instance.
   type Ephem_Nav_Converter_Algorithm is limited private;
   type Ephem_Nav_Converter_Algorithm_Access is access all Ephem_Nav_Converter_Algorithm;

   --* @brief Construct a new EphemNavConverterAlgorithm.
   function Create
     return Ephem_Nav_Converter_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "EphemNavConverterAlgorithm_create";

   --* @brief Destroy an EphemNavConverterAlgorithm.
   procedure Destroy
     (Self : Ephem_Nav_Converter_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "EphemNavConverterAlgorithm_destroy";

   --* @brief Convert ephemeris message to navigation translation message.
   --* @param Self             The algorithm instance.
   --* @param Call_Time        Timestamp for the update call.
   --* @param Ephemeris_In_Msg Pointer to ephemeris message payload.
   --* @return Navigation translation message.
   function Update
     (Self             : Ephem_Nav_Converter_Algorithm_Access;
      Call_Time        : Unsigned_64;
      Ephemeris_In_Msg : Ephemeris.C.U_C_Access)
     return Nav_Trans.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "EphemNavConverterAlgorithm_update";

private

   -- Private representation: opaque null record
   type Ephem_Nav_Converter_Algorithm is null record;

end Ephem_Nav_Converter_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
