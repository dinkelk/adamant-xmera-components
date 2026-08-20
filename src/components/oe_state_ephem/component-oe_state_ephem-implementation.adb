--------------------------------------------------------------------------------
-- Oe_State_Ephem Component Implementation Body
--------------------------------------------------------------------------------

with Basic_Types;
with Cartesian_State;
with Cartesian_State.C;
with Interfaces;
with Oe_Arc.C;
with Oe_State_Ephem_Parameter_Table.Validation;
with Parameter_Enums;

use Interfaces;

package body Component.Oe_State_Ephem.Implementation is

   ------------------------------------------------------------------------
   -- Protected staging area
   ------------------------------------------------------------------------

   protected body Staged_Config is

      procedure Stage (Table : in Oe_State_Ephem_Parameter_Table.T; Valid : out Boolean) is
      begin
         Central_Body_Mu := Table.Central_Body_Mu;
         Number_Of_Arcs := Table.Number_Of_Arcs.Value;
         Ephemeris_Time := Table.Ephemeris_Time;
         Vehicle_Time := Table.Vehicle_Clock_Time;
         -- Convert arc-by-arc: the array-level Oe_Arc_Records.C.Unpack returns
         -- the whole ~10 KB C array by value, and the compiler is free to
         -- materialize a full-size stack temporary for the assignment. Per-arc
         -- conversion bounds the transient at one arc (~1 KB).
         for I in Arcs'Range loop
            Arcs (I) := Oe_Arc.C.Unpack (Table.Arcs (I));
         end loop;
         -- Table-level format validation (done by the caller) only checks each
         -- field's type and range as declared in the table YAML; it does not
         -- know the algorithm's semantic constraints (Central_Body_Mu finite
         -- and non-negative, Number_Of_Arcs within MAX_OE_RECORDS, per-arc
         -- Number_Of_Coefficients within MAX_OE_COEFF, and finite times and
         -- active coefficients). Consulting the algorithm's own validator here,
         -- against the exact buffer that will be applied, is what keeps the
         -- throwing Create/Set_Config unreachable from Apply_If_Staged.
         Valid := Validate_Config (
            Central_Body_Mu  => Central_Body_Mu,
            Number_Of_Arcs   => Number_Of_Arcs,
            Ephemeris_Time   => Ephemeris_Time,
            Vehicle_Time     => Vehicle_Time,
            Fit_Coefficients => Arcs'Access);
         Is_Staged := Valid;
      end Stage;

      procedure Apply_If_Staged (Alg : in out Oe_State_Ephem_Algorithm_Access; Applied : out Boolean) is
      begin
         Applied := False;
         if not Is_Staged then
            return;
         end if;
         -- Only configurations Stage validated ever reach this point, so the
         -- throwing paths of Create/Set_Config are unreachable.
         if Alg = null then
            Alg := Create (
               Central_Body_Mu  => Central_Body_Mu,
               Number_Of_Arcs   => Number_Of_Arcs,
               Ephemeris_Time   => Ephemeris_Time,
               Vehicle_Time     => Vehicle_Time,
               Fit_Coefficients => Arcs'Access);
         else
            Set_Config (Alg,
               Central_Body_Mu  => Central_Body_Mu,
               Number_Of_Arcs   => Number_Of_Arcs,
               Ephemeris_Time   => Ephemeris_Time,
               Vehicle_Time     => Vehicle_Time,
               Fit_Coefficients => Arcs'Access);
         end if;
         Is_Staged := False;
         Applied := True;
      end Apply_If_Staged;

   end Staged_Config;

   ------------------------------------------------------------------------
   -- Local Helpers
   ------------------------------------------------------------------------

   -- Read the algorithm's active configuration into the caller's packed table
   -- without large stack temporaries: scalars individually, arcs one at a time
   -- through a ~1 KB C-layout local. The per-arc read-back API exists precisely
   -- so no caller needs a table-sized conversion buffer.
   procedure Read_Table_From_Algorithm (
      Alg : in Oe_State_Ephem_Algorithm_Access;
      Out_T : out Oe_State_Ephem_Parameter_Table.T
   ) is
      Central_Body_Mu : Long_Float;
      Number_Of_Arcs : Unsigned_32;
      Ephemeris_Time : Long_Float;
      Vehicle_Time : Long_Float;
   begin
      Get_Config_Scalars (Alg, Central_Body_Mu, Number_Of_Arcs, Ephemeris_Time, Vehicle_Time);
      Out_T.Ephemeris_Time := Ephemeris_Time;
      Out_T.Vehicle_Clock_Time := Vehicle_Time;
      Out_T.Central_Body_Mu := Central_Body_Mu;
      Out_T.Number_Of_Arcs := (Value => Number_Of_Arcs);
      for I in Out_T.Arcs'Range loop
         declare
            Arc_C : aliased Oe_Arc.C.U_C;
         begin
            Get_Config_Arc (Alg, Unsigned_32 (I), Arc_C'Access);
            Out_T.Arcs (I) := Oe_Arc.C.Pack (Arc_C);
         end;
      end loop;
   end Read_Table_From_Algorithm;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   overriding procedure Init (Self : in out Instance; Default_Table : not null Oe_State_Ephem_Parameter_Table.T_Access) is
      Valid : Boolean := False;
      Applied : Boolean := False;
   begin
      -- Stage and apply the default table through the same protected path used
      -- for uploads, constructing the algorithm on the first apply so any tick
      -- arriving before an uploaded table is received still produces
      -- deterministic output. Default_Table is passed by access to avoid a
      -- large by-value copy on the env task's stack. The default comes from the
      -- assembly rather than the ground, so a configuration the algorithm would
      -- reject is a wiring error: assert instead of reporting.
      Self.Staged_Parameters.Stage (Default_Table.all, Valid);
      pragma Assert (Valid);
      Self.Staged_Parameters.Apply_If_Staged (Self.Alg, Applied);
      pragma Assert (Applied);
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      Applied : Boolean := False;
   begin
      -- Apply any staged parameter table BEFORE running the algorithm so it
      -- operates on the freshest values starting this tick. The staged
      -- configuration was validated when it was uploaded, so applying cannot
      -- fail.
      Self.Staged_Parameters.Apply_If_Staged (Self.Alg, Applied);
      if Applied then
         Self.Event_T_Send_If_Connected (Self.Events.Parameter_Table_Applied (Self.Sys_Time_T_Get));
      end if;

      declare
         Call_Time_Ns : constant Interfaces.Unsigned_64 :=
            Interfaces.Unsigned_64 (Arg.Time.Seconds) * 1_000_000_000 +
            (Interfaces.Unsigned_64 (Arg.Time.Subseconds) * 1_000_000_000) / 65_536;
         Result : constant Cartesian_State.C.U_C := Update (Self.Alg, Call_Time_Ns);
      begin
         Self.Data_Product_T_Send (Self.Data_Products.Ephemeris_State (
            Arg.Time,
            Cartesian_State.Pack (Cartesian_State.C.To_Ada (Result))
         ));
      end;
   end Tick_T_Recv_Sync;

   overriding function Parameters_Memory_Region_T_Service (Self : in out Instance; Arg : in Parameters_Memory_Region.T) return Parameters_Memory_Region_Release.T is
      use Parameter_Enums.Parameter_Table_Operation_Type;
      use Parameter_Enums.Parameter_Table_Update_Status;
      Status : Parameter_Enums.Parameter_Table_Update_Status.E := Success;
   begin
      case Arg.Operation is
         when Set =>
            -- Forwarder hands us a payload-only region. Overlay and validate before use.
            declare
               Bytes : constant Basic_Types.Byte_Array (0 .. Arg.Region.Length - 1)
                  with Import, Convention => Ada, Address => Arg.Region.Address;
               Errant_Field : Interfaces.Unsigned_32 := 0;
            begin
               if not Oe_State_Ephem_Parameter_Table.Validation.Valid (Bytes, Errant_Field) then
                  Self.Event_T_Send_If_Connected (Self.Events.Invalid_Parameter_Table_Format (
                     Self.Sys_Time_T_Get,
                     (Value => Errant_Field)
                  ));
                  Status := Parameter_Error;
               else
                  declare
                     -- Overlay the packed .T directly on the upstream buffer, then
                     -- stage it. Stage also runs the algorithm's own configuration
                     -- validator, so a semantically bad table is rejected here,
                     -- synchronously on the upload, and never reaches the tick.
                     Table_T : constant Oe_State_Ephem_Parameter_Table.T
                        with Import, Convention => Ada, Address => Arg.Region.Address;
                     Valid : Boolean := False;
                  begin
                     Self.Staged_Parameters.Stage (Table_T, Valid);
                     if not Valid then
                        Self.Event_T_Send_If_Connected (Self.Events.Invalid_Parameter_Table_Config (
                           Self.Sys_Time_T_Get
                        ));
                        Status := Parameter_Error;
                     end if;
                  end;
               end if;
            end;

         when Validate =>
            -- Validate is intentionally unsupported for this component.
            Self.Event_T_Send_If_Connected (Self.Events.Validate_Not_Supported (
               Self.Sys_Time_T_Get
            ));
            Status := Parameter_Error;

         when Get_Copy =>
            -- Get_Copy is intentionally unsupported for this component.
            Self.Event_T_Send_If_Connected (Self.Events.Get_Copy_Not_Supported (
               Self.Sys_Time_T_Get
            ));
            Status := Parameter_Error;

         when Get_Pointer =>
            -- Snapshot the algorithm's current configuration into the component's
            -- dedicated Dump_Buffer in-place and expose its address, so a dump
            -- serves the algorithm's actual state (the single source of truth)
            -- rather than a component-side copy. This works fine as long as no
            -- table is being applied while this operation is called; operators
            -- wait for table upload success before trying to dump.
            Read_Table_From_Algorithm (Self.Alg, Self.Dump_Buffer);
            return (
               Region => (
                  Address => Self.Dump_Buffer'Address,
                  Length => Oe_State_Ephem_Parameter_Table.Size_In_Bytes
               ),
               Status => Success
            );
      end case;

      return (Region => Arg.Region, Status => Status);
   end Parameters_Memory_Region_T_Service;

end Component.Oe_State_Ephem.Implementation;
