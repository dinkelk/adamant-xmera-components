pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Ephemeris.C;
with Nav_Att.C;
with Nav_Trans.C;

package Sunline_Ephem_Algorithm_C is

   --* Opaque handle for a SunlineEphemAlgorithm instance.
   type Sunline_Ephem_Algorithm is limited private;
   type Sunline_Ephem_Algorithm_Access is access all Sunline_Ephem_Algorithm;

   --* @brief Construct a new SunlineEphemAlgorithm.
   function Create
     return Sunline_Ephem_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineEphemAlgorithm_create";

   --* @brief Destroy a SunlineEphemAlgorithm.
   procedure Destroy
     (Self : Sunline_Ephem_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineEphemAlgorithm_destroy";

   --* @brief Compute ephemeris-based sunline heading in body frame.
   --* @param Self    The algorithm instance.
   --* @param Sun_Pos Pointer to sun ephemeris message payload.
   --* @param Sc_Pos  Pointer to spacecraft position message payload.
   --* @param Sc_Att  Pointer to spacecraft attitude message payload.
   --* @return Navigation message containing sunline direction in body frame.
   function Update
     (Self    : Sunline_Ephem_Algorithm_Access;
      Sun_Pos : Ephemeris.C.U_C_Access;
      Sc_Pos  : Nav_Trans.C.U_C_Access;
      Sc_Att  : Nav_Att.C.U_C_Access)
     return Nav_Att.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineEphemAlgorithm_update";

private

   -- Private representation: opaque null record
   type Sunline_Ephem_Algorithm is null record;

end Sunline_Ephem_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
