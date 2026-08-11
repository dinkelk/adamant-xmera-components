--------------------------------------------------------------------------------
-- Oe_State_Ephem Component Implementation Body
--------------------------------------------------------------------------------

with Basic_Types;
with Cartesian_State;
with Cartesian_State.C;
with Interfaces;
with Oe_State_Ephem_Parameter_Table.Validation;
with Parameter_Enums;

use Interfaces;

package body Component.Oe_State_Ephem.Implementation is

   ------------------------------------------------------------------------
   -- Local Helpers
   ------------------------------------------------------------------------
   -- Fill the instance's off-stack C arc-array staging buffer from a table.
   procedure Build_Config_Arcs (Self : in out Instance; Table : in Oe_State_Ephem_Parameter_Table.T) is
   begin
      Self.Config_Arcs := Oe_Arc_Records.C.Unpack (Table.Arcs);
   end Build_Config_Arcs;

   -- Ask the algorithm's own non-throwing predicate whether it would accept a
   -- table, using the C-boundary arc array already staged in Self.Config_Arcs.
   --
   -- Table-level validation only checks each field's type and range as declared in
   -- the table YAML; it does not know the algorithm's semantic constraints
   -- (Central_Body_Mu finite and non-negative, Number_Of_Arcs within MAX_OE_RECORDS,
   -- per-arc Number_Of_Coefficients within MAX_OE_COEFF, and finite times and active
   -- coefficients). Consulting Validate_Config is what keeps an accepted-but-unusable
   -- table out of the throwing Set_Config, where the C++ exception would escape
   -- into Ada.
   function Config_Is_Valid (Self : in out Instance; Table : in Oe_State_Ephem_Parameter_Table.T) return Boolean
   is (Validate_Config (
         Central_Body_Mu  => Table.Central_Body_Mu,
         Number_Of_Arcs   => Table.Number_Of_Arcs.Value,
         Ephemeris_Time   => Table.Ephemeris_Time,
         Vehicle_Time     => Table.Vehicle_Clock_Time,
         Fit_Coefficients => Self.Config_Arcs'Access));

   -- Push a parameter table to the C++ algorithm via one flattened Set_Config, and
   -- remember it as the component's current configuration for Get_Pointer dumps.
   --
   -- Returns False, leaving the algorithm on the configuration it had already
   -- applied, when the table is rejected.
   --
   -- Validation deliberately happens here rather than when the upload is staged: it
   -- needs the C-boundary arc array, and Config_Arcs is a ~10 KB buffer owned by
   -- the tick task. Building it in the Set handler would either race the tick task
   -- or cost a second ~10 KB buffer, so a bad table is instead rejected on the tick
   -- that would have applied it, one tick after the upload is acknowledged.
   function Apply_Table (Self : in out Instance; Table : in Oe_State_Ephem_Parameter_Table.T) return Boolean is
   begin
      Build_Config_Arcs (Self, Table);
      if not Config_Is_Valid (Self, Table) then
         -- Config_Arcs is left holding the rejected table's arcs. That is harmless:
         -- every apply rebuilds it from the table it is about to push, and Init
         -- builds it before the first Create.
         return False;
      end if;
      Set_Config (Self.Alg,
         Central_Body_Mu  => Table.Central_Body_Mu,
         Number_Of_Arcs   => Table.Number_Of_Arcs.Value,
         Ephemeris_Time   => Table.Ephemeris_Time,
         Vehicle_Time     => Table.Vehicle_Clock_Time,
         Fit_Coefficients => Self.Config_Arcs'Access);
      Self.Dump_Buffer := Table;
      return True;
   end Apply_Table;

   -- Copy the staged parameter table into the algorithm, reporting whether it was
   -- accepted. Isolated into a separate subprogram so the ~10 KB
   -- Oe_State_Ephem_Parameter_Table.T lives only on this helper's stack frame,
   -- which is not frequently called.
   function Drain_Staged_To_Algorithm (Self : in out Instance) return Boolean is
      New_Table_T : Oe_State_Ephem_Parameter_Table.T;
   begin
      Self.Staged_Parameters.Copy_From_Staged (New_Table_T);
      return Apply_Table (Self, New_Table_T);
   end Drain_Staged_To_Algorithm;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   overriding procedure Init (Self : in out Instance; Default_Table : not null Oe_State_Ephem_Parameter_Table.T_Access) is
   begin
      -- Build the initial C arc array and construct the algorithm with the default
      -- configuration so any tick arriving before an uploaded table is received still
      -- produces deterministic output. Default_Table is passed by access to avoid a
      -- large by-value copy on the env task's stack.
      Build_Config_Arcs (Self, Default_Table.all);
      -- The default table comes from the assembly rather than the ground, so a
      -- configuration the algorithm would reject is a wiring error: assert instead
      -- of reporting, and keep it out of the throwing Create.
      pragma Assert (Config_Is_Valid (Self, Default_Table.all));
      Self.Alg := Create (
         Central_Body_Mu  => Default_Table.all.Central_Body_Mu,
         Number_Of_Arcs   => Default_Table.all.Number_Of_Arcs.Value,
         Ephemeris_Time   => Default_Table.all.Ephemeris_Time,
         Vehicle_Time     => Default_Table.all.Vehicle_Clock_Time,
         Fit_Coefficients => Self.Config_Arcs'Access);
      Self.Dump_Buffer := Default_Table.all;
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
   begin
      -- Apply the staged parameter table BEFORE running the algorithm so it operates
      -- on the freshest values starting this tick, but only when a new table is staged.
      if Self.Staged_Parameters.Is_Staged then
         if Drain_Staged_To_Algorithm (Self) then
            Self.Event_T_Send_If_Connected (Self.Events.Parameter_Table_Applied (Self.Sys_Time_T_Get));
         else
            -- The algorithm refused the uploaded configuration. It keeps running on
            -- the previously applied table; report so the rejection is visible in
            -- telemetry rather than silently discarded.
            Self.Event_T_Send_If_Connected (Self.Events.Invalid_Parameter_Table_Config (Self.Sys_Time_T_Get));
         end if;
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
                     -- Overlay the packed .T directly on the upstream buffer, then stage it.
                     Table_T : constant Oe_State_Ephem_Parameter_Table.T
                        with Import, Convention => Ada, Address => Arg.Region.Address;
                  begin
                     Self.Staged_Parameters.Stage (Table_T);
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
            -- Expose the component's stored copy of the last-applied parameter table.
            -- The flattened shim has no getters, so the component is the source of
            -- truth for the current configuration. Valid as long as no table update
            -- is in flight when this is called (operators dump after upload success).
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
