--------------------------------------------------------------------------------
-- Oe_State_Ephem Component Implementation Spec
--------------------------------------------------------------------------------

-- Includes:
with Tick;
with Parameters_Memory_Region;
with Oe_State_Ephem_Parameter_Table;
with Oe_State_Ephem_Algorithm_C; use Oe_State_Ephem_Algorithm_C;
with Oe_Arc_Records.C;
with Interfaces;

-- Orbital element state ephemeris algorithm. Computes spacecraft Cartesian
-- state (position and velocity) from Chebyshev polynomial fits of classical
-- orbital elements. The algorithm's configuration (central body gravitational
-- parameter and per-arc Chebyshev coefficients) is delivered as a single
-- Oe_State_Ephem_Parameter_Table payload via a Parameter_Table_Forwarder
-- upstream; the component validates the bytes and the configuration when the
-- upload arrives, stages it in the C-boundary layout, and applies it to the
-- algorithm on the next tick.
package Component.Oe_State_Ephem.Implementation is

   -- The component class instance record:
   type Instance is new Oe_State_Ephem.Base_Instance with private;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the algorithm with a default parameter table. The component
   -- applies the default to the C++ algorithm immediately so that ticks
   -- arriving before any uploaded table is received still produce
   -- deterministic output.
   --
   -- Init Parameters:
   -- Default_Table : Oe_State_Ephem_Parameter_Table.T_Access - Pointer to a
   -- packed parameter table applied to the algorithm at startup. The
   -- component derefs it once during Init to push values into the C++
   -- algorithm; passing by access avoids any large by-value copy on the
   -- env task's stack at Init_Components time. Default_Table is overridden
   -- by any subsequent successful Set delivered via the parameter-region
   -- pathway.
   --
   overriding procedure Init (Self : in out Instance; Default_Table : not null Oe_State_Ephem_Parameter_Table.T_Access);
   not overriding procedure Destroy (Self : in out Instance);

private

   -- Staging area for parameter tables, held directly in the C-boundary layout
   -- that Create/Set_Config consume by reference. The Service handler (forwarder
   -- task) converts a format-valid upload arc-by-arc into this buffer and
   -- consults the algorithm's own configuration validator right there, so a
   -- rejected table is reported synchronously on the upload and only tables the
   -- algorithm will accept are ever marked staged; the tick task then applies
   -- the staged configuration, which cannot fail. Staging in C layout means the
   -- component carries exactly one staging buffer (no packed staged copy plus a
   -- separate conversion buffer) and the applying tick performs no large copies
   -- or conversions: staging, validation, and apply all act on this buffer
   -- under its lock.
   protected type Staged_Config is
      -- Convert Table into the internal C-layout buffer and validate it via the
      -- algorithm's configuration validator. Marks the buffer staged (and
      -- reports Valid => True) only when the algorithm would accept it, which is
      -- what keeps the throwing Create/Set_Config unreachable from
      -- Apply_If_Staged. A rejected table reports Valid => False and leaves
      -- nothing staged, including any earlier staged-but-unapplied table (the
      -- buffer is single and latest-wins).
      procedure Stage (Table : in Oe_State_Ephem_Parameter_Table.T; Valid : out Boolean);
      -- Push the staged configuration to the algorithm and clear the staged
      -- flag: Create when Alg is still null (first apply, from Init), Set_Config
      -- afterwards. No-op with Applied => False when nothing is staged.
      procedure Apply_If_Staged (Alg : in out Oe_State_Ephem_Algorithm_Access; Applied : out Boolean);
   private
      Central_Body_Mu : Long_Float := 0.0;
      Number_Of_Arcs : Interfaces.Unsigned_32 := 0;
      Ephemeris_Time : Long_Float := 0.0;
      Vehicle_Time : Long_Float := 0.0;
      -- Written by Stage before Is_Staged is ever set; unread until then.
      Arcs : aliased Oe_Arc_Records.C.U_C;
      Is_Staged : Boolean := False;
   end Staged_Config;

   -- The component class instance record:
   type Instance is new Oe_State_Ephem.Base_Instance with record
      Alg : Oe_State_Ephem_Algorithm_Access := null;
      -- The single staging buffer (see Staged_Config above).
      Staged_Parameters : Staged_Config;
      -- Scratch for Get_Pointer dumps: filled from the algorithm's actual
      -- configuration on each dump request, so the algorithm remains the single
      -- source of truth and the component keeps no copy of the applied table.
      Dump_Buffer : Oe_State_Ephem_Parameter_Table.T;
   end record;

   ---------------------------------------
   -- Set Up Procedure
   ---------------------------------------
   overriding procedure Set_Up (Self : in out Instance) is null;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time. Also applies the staged parameter
   -- table (if any) to the algorithm before evaluating.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T);
   -- Inbound parameter table memory region from an upstream
   -- Parameter_Table_Forwarder; returns the operation status (Success,
   -- Parameter_Error, etc) synchronously. The forwarder has already stripped
   -- the parameter table header; the region contains only the
   -- Oe_State_Ephem_Parameter_Table payload bytes.
   overriding function Parameters_Memory_Region_T_Service (Self : in out Instance; Arg : in Parameters_Memory_Region.T) return Parameters_Memory_Region_Release.T;

   ---------------------------------------
   -- Invoker connector primitives:
   ---------------------------------------
   -- This procedure is called when a Data_Product_T_Send message is dropped due to a full queue.
   overriding procedure Data_Product_T_Send_Dropped (Self : in out Instance; Arg : in Data_Product.T) is null;
   -- This procedure is called when a Event_T_Send message is dropped due to a full queue.
   overriding procedure Event_T_Send_Dropped (Self : in out Instance; Arg : in Event.T) is null;

end Component.Oe_State_Ephem.Implementation;
