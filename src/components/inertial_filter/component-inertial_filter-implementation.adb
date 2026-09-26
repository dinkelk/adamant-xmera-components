--------------------------------------------------------------------------------
-- Inertial_Filter Component Implementation Body
--------------------------------------------------------------------------------

with Inertial_Filter_Output.C;
with Inertial_Filter_Rate_Data.C;
with Inertial_Filter_St_Att_Data.C;
with Inertial_Filter_State_Matrix.C;
with Inertial_Filter_State_Vector.C;
with Nav_Att_Output;
with Packed_F32x3;
with Packed_F64x3.C;
with Packed_F64x6.C;
with Packed_F64x36.C;
with St_Att;

package body Component.Inertial_Filter.Implementation is

   -- The parts of the configuration the shim takes by pointer, held together so a
   -- caller can pass 'Access of each field. Create_Filter and Validate_Parameters
   -- both marshal the same values, so it is assembled in one place.
   type Pointer_Config is record
      Process_Noise : aliased Inertial_Filter_State_Matrix.C.U_C;
      Initial_State : aliased Inertial_Filter_State_Vector.C.U_C;
      Initial_Covariance : aliased Inertial_Filter_State_Matrix.C.U_C;
   end record;

   -- Expand a diagonal into the row major N x N matrix the shim takes. The
   -- parameters carry only the diagonals, since the full matrices do not fit in a
   -- parameter.
   function Diagonal_Matrix (Diagonal : in Packed_F64x6.U) return Packed_F64x36.U is
      Matrix : Packed_F64x36.U := [others => 0.0];
   begin
      for I in Diagonal'Range loop
         Matrix (I * Packed_F64x6.Length + I) := Diagonal (I);
      end loop;
      return Matrix;
   end Diagonal_Matrix;

   -- Marshal the pointer arguments of the configuration.
   function To_Pointer_Config (
      Process_Noise_Diagonal : in Packed_F64x6.U;
      Initial_State : in Packed_F64x6.U;
      Initial_Covariance_Diagonal : in Packed_F64x6.U
   ) return Pointer_Config is
      (Process_Noise => (Value => Packed_F64x36.C.To_C (Diagonal_Matrix (Process_Noise_Diagonal))),
       Initial_State => (Value => Packed_F64x6.C.To_C (Initial_State)),
       Initial_Covariance => (Value => Packed_F64x36.C.To_C (Diagonal_Matrix (Initial_Covariance_Diagonal))));

   -- Build the filter from the component's current parameters. The values were
   -- checked at staging by Validate_Parameters, so Create does not throw.
   procedure Create_Filter (Self : in out Instance) is
      Cfg : aliased constant Pointer_Config := To_Pointer_Config (Self.Process_Noise_Diagonal, Self.Initial_State, Self.Initial_Covariance_Diagonal);
   begin
      Self.Alg := Create (
         Alpha                      => Self.Alpha.Value,
         Beta                       => Self.Beta.Value,
         Process_Noise              => Cfg.Process_Noise'Access,
         Initial_State              => Cfg.Initial_State'Access,
         Initial_Covariance         => Cfg.Initial_Covariance'Access,
         St_Measurement_Noise_Std   => Self.St_Measurement_Noise_Std.Value,
         Rate_Measurement_Noise_Std => Self.Rate_Measurement_Noise_Std.Value);
   end Create_Filter;

   -- Widen a single precision vector from a data product to the double precision
   -- the shim takes. The vector is unpacked first, so each element is read whole
   -- before it is converted.
   function To_C (Vector : in Packed_F32x3.T) return Packed_F64x3.C.U_C is
      Unpacked : constant Packed_F32x3.U := Packed_F32x3.Unpack (Vector);
   begin
      return Packed_F64x3.C.To_C ([for I in Unpacked'Range => Long_Float (Unpacked (I))]);
   end To_C;

   -- System time in nanoseconds, the unit of the star tracker time tag, so the two are
   -- compared and differenced exactly.
   Ns_Per_Second : constant Unsigned_64 := 1_000_000_000;
   function To_Nanoseconds (Time : in Sys_Time.T) return Unsigned_64 is
      (Unsigned_64 (Time.Seconds) * Ns_Per_Second + (Unsigned_64 (Time.Subseconds) * Ns_Per_Second) / Unsigned_64 (Sys_Time.Subseconds_Type'Modulus));

   -- Seconds since the filter's time base.
   function Filter_Seconds (Self : in Instance; Ns : in Unsigned_64) return Long_Float is
      (Long_Float (Ns - Self.Epoch_Ns) * 1.0E-9);

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the inertial filter with the default parameter values, which seed
   -- the filter state and covariance. The diagnostics packet is off until its period
   -- is commanded.
   overriding procedure Init (Self : in out Instance) is
   begin
      -- Create throws on an invalid configuration, so the parameter defaults must form a
      -- valid one. The generated Assert_Valid_Parameter_Defaults checks them at startup
      -- and in unit test set up.
      Create_Filter (Self);
      -- The protected counter starts with a period of one, so turn the packet off
      -- until it is commanded.
      Self.Diagnostics_Counter.Set_Period_And_Reset_Count (0);
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the filter up to the current time, folding in a fresh star tracker reading
   -- when there is one.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      use Data_Product_Enums;
      use Data_Product_Enums.Data_Dependency_Status;

      -- Grab data dependencies:
      --
      -- Data_Dependency_Status.E can be Success, Not_Available, Error, or Stale.
      -- The star tracker attitude is produced earlier in the same tick, so any other
      -- status indicates that this component is not wired up correctly in the
      -- algorithm execution order. That should never happen, so we assert.
      Star_Tracker : St_Att.T;
      Star_Tracker_Status : constant Data_Dependency_Status.E :=
         Self.Get_Star_Tracker_Attitude (Value => Star_Tracker, Stale_Reference => Arg.Time);
      pragma Assert (Star_Tracker_Status = Success);

      Tick_Ns : constant Unsigned_64 := To_Nanoseconds (Arg.Time);
   begin
      -- Apply any pending parameter update, which may rebuild the filter:
      Self.Update_Parameters;

      -- A clock that stepped back cannot be followed: the filter's pending measurements
      -- and time anchor are dropped, and its time base restarts below, as after a
      -- measurements reset.
      if Tick_Ns < Self.Epoch_Ns then
         Re_Initialize_Except_Persistent_States (Self.Alg);
         Self.Restart_Time_Base := True;
      end if;

      -- A freshly built or reset filter starts its time at this tick. Readings from
      -- before the restart are then not new to the filter.
      if Self.Restart_Time_Base then
         Self.Epoch_Ns := Tick_Ns;
         Self.Restart_Time_Base := False;
         Self.Last_St_Time_Tag := 0;
      end if;

      declare
         -- The product is refreshed only when the star tracker delivers, so it is a new
         -- measurement only when its time tag has advanced. It must also lie within the
         -- filter's time base and not be after the tick, since the filter only advances
         -- to the tick; a tag from the future waits rather than shutting out the
         -- readings that follow. A time tag of zero tells the filter there is no new
         -- reading of that kind; a fresh one is fed as both the attitude and the rate
         -- measurement, as the algorithm's own adapter does.
         Fresh : constant Boolean := Star_Tracker.Time_Tag > Self.Last_St_Time_Tag
            and then Star_Tracker.Time_Tag > Self.Epoch_Ns
            and then Star_Tracker.Time_Tag <= Tick_Ns;
         Time_Tag : constant Long_Float := (if Fresh then Filter_Seconds (Self, Star_Tracker.Time_Tag) else 0.0);
         St_Att_Data : aliased constant Inertial_Filter_St_Att_Data.C.U_C :=
            (Time_Tag => Time_Tag, Sigma_Bn => To_C (Star_Tracker.Sigma_Bn));
         Rate_Data : aliased constant Inertial_Filter_Rate_Data.C.U_C :=
            (Time_Tag => Time_Tag, Rate => To_C (Star_Tracker.Omega_Bn_B));
      begin
         if Fresh then
            Self.Last_St_Time_Tag := Star_Tracker.Time_Tag;
         end if;

         declare
            -- Advance the filter and take its snapshot. The estimate is the first three
            -- states, the attitude MRP, and the next three, the body rate.
            Output : constant Inertial_Filter_Output.C.U_C := Update (
               Self.Alg,
               Current_Seconds => Filter_Seconds (Self, Tick_Ns),
               St_Att          => St_Att_Data'Access,
               Rate            => Rate_Data'Access);
         begin
            -- Publish the estimate for the downstream algorithms, narrowed to the single
            -- precision they consume, and stamped in system seconds. This filter does
            -- not estimate the sun direction.
            Self.Data_Product_T_Send (Self.Data_Products.Attitude_Estimate (
               Arg.Time,
               Nav_Att_Output.Pack ((
                  Time_Tag        => Long_Float (Tick_Ns) * 1.0E-9,
                  Sigma_Bn        => [for I in 0 .. 2 => Short_Float (Output.State (I))],
                  Omega_Bn_B      => [for I in 0 .. 2 => Short_Float (Output.State (I + 3))],
                  Veh_Sun_Pnt_Bdy => [0.0, 0.0, 0.0]))
            ));

            -- The full snapshot goes out as a packet every commanded number of ticks.
            Self.Diagnostics_Counter.Increment_Count;
            if Self.Diagnostics_Counter.Is_Count_At_Period then
               Self.Packet_T_Send_If_Connected (Self.Packets.Filter_Diagnostics (Arg.Time, Inertial_Filter_Output.C.Pack (Output)));
            end if;
         end;
      end;
   end Tick_T_Recv_Sync;

   -- Re-seed the filter state and covariance from the configured initial values and
   -- clear the pending measurements and residuals. TODO verify with the GNC team
   -- whether this reset is needed at all.
   overriding procedure Reset_Estimate_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      Ignore : Tick.T renames Arg;
   begin
      Re_Initialize (Self.Alg);
      Self.Restart_Time_Base := True;
   end Reset_Estimate_Tick_T_Recv_Sync;

   -- Clear the pending measurements and residuals, keeping the filter state and
   -- covariance.
   overriding procedure Reset_Measurements_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      Ignore : Tick.T renames Arg;
   begin
      -- Clearing the pending measurements also drops the filter's time anchor, so its
      -- time base restarts as well.
      Re_Initialize_Except_Persistent_States (Self.Alg);
      Self.Restart_Time_Base := True;
   end Reset_Measurements_Tick_T_Recv_Sync;

   -- The parameter update connector.
   overriding procedure Parameter_Update_T_Modify (Self : in out Instance; Arg : in out Parameter_Update.T) is
   begin
      -- Process the parameter update, staging or fetching parameters as requested.
      Self.Process_Parameter_Update (Arg);
   end Parameter_Update_T_Modify;

   -- This is the command receive connector.
   overriding procedure Command_T_Recv_Sync (Self : in out Instance; Arg : in Command.T) is
      -- Execute the command:
      Stat : constant Command_Response_Status.E := Self.Execute_Command (Arg);
   begin
      -- Send the return status:
      Self.Command_Response_T_Send_If_Connected ((Source_Id => Arg.Header.Source_Id, Registration_Id => Self.Command_Reg_Id, Command_Id => Arg.Header.Id, Status => Stat));
   end Command_T_Recv_Sync;

   -----------------------------------------------
   -- Command handler primitives:
   -----------------------------------------------
   -- Description:
   --    Commands for the Inertial Filter component.
   -- Set the period of the diagnostics packet in ticks. Zero turns the packet off.
   overriding function Set_Diagnostics_Packet_Period (Self : in out Instance; Arg : in Packed_U16.T) return Command_Execution_Status.E is
      use Command_Execution_Status;
   begin
      Self.Diagnostics_Counter.Set_Period_And_Reset_Count (Arg.Value);
      Self.Event_T_Send_If_Connected (Self.Events.Diagnostics_Packet_Period_Set (Self.Sys_Time_T_Get, Arg));
      return Success;
   end Set_Diagnostics_Packet_Period;

   -- Invalid command handler. This procedure is called when a command's arguments are found to be invalid:
   overriding procedure Invalid_Command (Self : in out Instance; Cmd : in Command.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type) is
   begin
      -- A malformed command can arrive from the ground, so report it rather than assert:
      Self.Event_T_Send_If_Connected (Self.Events.Invalid_Command_Received (
         Self.Sys_Time_T_Get,
         (Id => Cmd.Header.Id, Errant_Field_Number => Errant_Field_Number, Errant_Field => Errant_Field)
      ));
   end Invalid_Command;

   -----------------------------------------------
   -- Parameter handlers:
   -----------------------------------------------
   -- Description:
   --    Parameters for the Inertial Filter component.
   -- This procedure is called when the parameters of a component have been updated. The
   -- shim has no setConfig, so the only way to apply a new configuration is to build a
   -- new filter, which restarts the estimate from the configured seed.
   -- TODO replace with Set_Config once the inertialFilter C shim provides one, as the
   -- sunlineFilter shim does, so a parameter update keeps the current estimate.
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
   begin
      Destroy (Self.Alg);
      Create_Filter (Self);
      Self.Restart_Time_Base := True;
   end Update_Parameters_Action;

   -- Validate a staged parameter set before it is applied by asking the algorithm's own
   -- non-throwing Validate_Config predicate, so the config rules live solely in the
   -- algorithm. Rejecting an invalid update here at staging keeps it from reaching the
   -- throwing Create across the FFI boundary.
   overriding function Validate_Parameters (
      Self : in out Instance;
      Alpha : in Packed_F64.U;
      Beta : in Packed_F64.U;
      Process_Noise_Diagonal : in Packed_F64x6.U;
      Initial_State : in Packed_F64x6.U;
      Initial_Covariance_Diagonal : in Packed_F64x6.U;
      St_Measurement_Noise_Std : in Packed_F64.U;
      Rate_Measurement_Noise_Std : in Packed_F64.U
   ) return Parameter_Validation_Status.E is
      Ignore : Instance renames Self;
      -- Filled in below, inside the handled part of the function, so that a conversion
      -- that raises is caught here.
      Cfg : aliased Pointer_Config;
   begin
      Cfg := To_Pointer_Config (Process_Noise_Diagonal, Initial_State, Initial_Covariance_Diagonal);
      if Validate_Config (
         Alpha                      => Alpha.Value,
         Beta                       => Beta.Value,
         Process_Noise              => Cfg.Process_Noise'Access,
         Initial_State              => Cfg.Initial_State'Access,
         Initial_Covariance         => Cfg.Initial_Covariance'Access,
         St_Measurement_Noise_Std   => St_Measurement_Noise_Std.Value,
         Rate_Measurement_Noise_Std => Rate_Measurement_Noise_Std.Value)
      then
         return Parameter_Validation_Status.Valid;
      else
         return Parameter_Validation_Status.Invalid;
      end if;
   exception
      -- Reachable, and the parameter rejection test covers it: float staging accepts a
      -- non-finite value, and marshalling it above raises. Rejecting the set here keeps
      -- that from unwinding into the Parameters component.
      when Constraint_Error =>
         return Parameter_Validation_Status.Invalid;
   end Validate_Parameters;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Inertial Filter component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Inertial_Filter.Implementation;
