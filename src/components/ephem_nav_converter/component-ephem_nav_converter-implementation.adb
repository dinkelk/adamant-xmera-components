--------------------------------------------------------------------------------
-- Ephem_Nav_Converter Component Implementation Body
--------------------------------------------------------------------------------

with Ephemeris.C;
with Nav_Trans.C;
with Algorithm_Wrapper_Util;

package body Component.Ephem_Nav_Converter.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the ephemeris navigation converter algorithm.
   overriding procedure Init (Self : in out Instance) is
   begin
      -- Allocate C++ class on the heap
      Self.Alg := Create;
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      use Data_Product_Enums;
      use Data_Product_Enums.Data_Dependency_Status;
      use Algorithm_Wrapper_Util;

      Eph : Ephemeris.T;
      Eph_Status : constant Data_Dependency_Status.E :=
         Self.Get_Input_Ephemeris (Value => Eph, Stale_Reference => Arg.Time);
   begin
      if Is_Dep_Status_Success (Eph_Status) then
         declare
            Eph_C : aliased Ephemeris.C.U_C := Ephemeris.C.To_C (Ephemeris.Unpack (Eph));
            Nav_Trans_Out : constant Nav_Trans.C.U_C := Update (
               Self.Alg,
               Call_Time => 0, -- Unused by algorithm
               Ephemeris_In_Msg => Eph_C'Unchecked_Access
            );
         begin
            Self.Data_Product_T_Send (Self.Data_Products.Navigation_Translation (
               Arg.Time,
               Nav_Trans.Pack (Nav_Trans.C.To_Ada (Nav_Trans_Out))
            ));
         end;
      end if;
   end Tick_T_Recv_Sync;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Ephem Nav Converter component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Ephem_Nav_Converter.Implementation;
