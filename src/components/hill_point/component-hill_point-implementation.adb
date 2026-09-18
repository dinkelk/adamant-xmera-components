--------------------------------------------------------------------------------
-- Hill_Point Component Implementation Body
--------------------------------------------------------------------------------

with Att_Ref;
with Att_Ref.C;
with Cartesian_State;
with Cartesian_State.C;

package body Component.Hill_Point.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the Hill point algorithm.
   overriding procedure Init (Self : in out Instance) is
   begin
      -- Allocate the C++ class on the heap. The algorithm has no configuration.
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

      -- Grab data dependencies:
      --
      -- Data_Dependency_Status.E can be Success, Not_Available, Error, or Stale.
      -- Both states are published fresh by the ephemeris components earlier in the
      -- same tick, so any other status indicates that this component is not wired
      -- up correctly in the algorithm execution order. That should never happen, so
      -- we assert.
      Spacecraft : Cartesian_State.T;
      Spacecraft_Status : constant Data_Dependency_Status.E :=
         Self.Get_Spacecraft_State (Value => Spacecraft, Stale_Reference => Arg.Time);
      pragma Assert (Spacecraft_Status = Success);
      Primary_Body : Cartesian_State.T;
      Primary_Body_Status : constant Data_Dependency_Status.E :=
         Self.Get_Primary_Body_State (Value => Primary_Body, Stale_Reference => Arg.Time);
      pragma Assert (Primary_Body_Status = Success);

      -- Convert to C types. Each state is converted once because it feeds two of
      -- the algorithm's arguments below.
      Spacecraft_C : constant Cartesian_State.C.U_C :=
         Cartesian_State.C.Unpack (Spacecraft);
      Primary_Body_C : constant Cartesian_State.C.U_C :=
         Cartesian_State.C.Unpack (Primary_Body);

      -- Call the algorithm. The vectors cross by value.
      Reference : constant Att_Ref.C.U_C := Update (
         Self.Alg,
         R_Bn_N => (Value => Spacecraft_C.Position),
         V_Bn_N => (Value => Spacecraft_C.Velocity),
         R_Pn_N => (Value => Primary_Body_C.Position),
         V_Pn_N => (Value => Primary_Body_C.Velocity)
      );
   begin
      -- Send out data product:
      Self.Data_Product_T_Send (Self.Data_Products.Attitude_Reference (
         Arg.Time,
         Att_Ref.C.Pack (Reference)
      ));
   end Tick_T_Recv_Sync;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Hill Point component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Hill_Point.Implementation;
