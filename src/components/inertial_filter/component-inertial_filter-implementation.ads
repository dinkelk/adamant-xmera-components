--------------------------------------------------------------------------------
-- Inertial_Filter Component Implementation Spec
--------------------------------------------------------------------------------

-- Includes:
with Tick;
with Parameter_Update;
with Command;
with Protected_Variables;
with Inertial_Filter_Algorithm_C; use Inertial_Filter_Algorithm_C;

-- Inertial attitude filter. Estimates the body attitude and body rate from the
-- star tracker attitude and rate with a square root unscented Kalman filter, and
-- publishes the estimate for the guidance and control algorithms. A diagnostics
-- packet with the full filter state, covariance, and residuals is produced at a
-- commanded period. The filter keeps its own time base, which restarts at the tick
-- after it is built or reset, so the star tracker time tag and the tick time must
-- be on the same clock. Wraps the InertialFilterAlgorithm C++ algorithm via its C
-- shim.
package Component.Inertial_Filter.Implementation is

   -- The component class instance record:
   type Instance is new Inertial_Filter.Base_Instance with private;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the inertial filter with the default parameter values, which seed
   -- the filter state and covariance. The diagnostics packet is off until its period
   -- is commanded.
   overriding procedure Init (Self : in out Instance);
   not overriding procedure Destroy (Self : in out Instance);

private

   -- The diagnostics packet period is set by command, from another task than the
   -- tick, so the counter is protected.
   package Packet_Period_Counter is new Protected_Variables.Generic_Protected_Periodic_Counter (Unsigned_16);

   -- The component class instance record:
   type Instance is new Inertial_Filter.Base_Instance with record
      Alg : Inertial_Filter_Algorithm_Access := null;
      -- The filter keeps time in seconds from an origin at zero: it anchors there when
      -- built or reset and propagates from the anchor to the current time. Handing it
      -- system time would make that first propagation span the whole mission, so the
      -- component gives it a time base of its own, in nanoseconds of system time,
      -- which restarts at the next tick after the filter is built or reset.
      Epoch_Ns : Unsigned_64 := 0;
      Restart_Time_Base : Boolean := True;
      -- Time tag of the last star tracker reading fed to the filter. A reading is fed
      -- only when its time tag has advanced past this one, so a product that has not
      -- been refreshed since the last tick is not applied twice.
      Last_St_Time_Tag : Unsigned_64 := 0;
      -- Counts ticks toward the next diagnostics packet.
      Diagnostics_Counter : Packet_Period_Counter.Counter;
   end record;

   ---------------------------------------
   -- Set Up Procedure
   ---------------------------------------
   -- Null method which can be implemented to provide some component
   -- set up code. This method is generally called by the assembly
   -- main.adb after all component initialization and tasks have been started.
   -- Some activities need to only be run once at startup, but cannot be run
   -- safely until everything is up and running, i.e. command registration, initial
   -- data product updates. This procedure should be implemented to do these things
   -- if necessary.
   overriding procedure Set_Up (Self : in out Instance) is null;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the filter up to the current time, folding in a fresh star tracker reading
   -- when there is one.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T);
   -- Re-seed the filter state and covariance from the configured initial values and
   -- clear the pending measurements and residuals. TODO verify with the GNC team
   -- whether this reset is needed at all.
   overriding procedure Reset_Estimate_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T);
   -- Clear the pending measurements and residuals, keeping the filter state and
   -- covariance.
   overriding procedure Reset_Measurements_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T);
   -- The parameter update connector.
   overriding procedure Parameter_Update_T_Modify (Self : in out Instance; Arg : in out Parameter_Update.T);
   -- This is the command receive connector.
   overriding procedure Command_T_Recv_Sync (Self : in out Instance; Arg : in Command.T);

   ---------------------------------------
   -- Invoker connector primitives:
   ---------------------------------------
   -- This procedure is called when a Data_Product_T_Send message is dropped due to a full queue.
   overriding procedure Data_Product_T_Send_Dropped (Self : in out Instance; Arg : in Data_Product.T) is null;
   -- This procedure is called when a Command_Response_T_Send message is dropped due to a full queue.
   overriding procedure Command_Response_T_Send_Dropped (Self : in out Instance; Arg : in Command_Response.T) is null;
   -- This procedure is called when a Packet_T_Send message is dropped due to a full queue.
   overriding procedure Packet_T_Send_Dropped (Self : in out Instance; Arg : in Packet.T) is null;
   -- This procedure is called when a Event_T_Send message is dropped due to a full queue.
   overriding procedure Event_T_Send_Dropped (Self : in out Instance; Arg : in Event.T) is null;

   -----------------------------------------------
   -- Command handler primitives:
   -----------------------------------------------
   -- Description:
   --    Commands for the Inertial Filter component.
   -- Set the period of the diagnostics packet in ticks. Zero turns the packet off.
   overriding function Set_Diagnostics_Packet_Period (Self : in out Instance; Arg : in Packed_U16.T) return Command_Execution_Status.E;

   -- Invalid command handler. This procedure is called when a command's arguments are found to be invalid:
   overriding procedure Invalid_Command (Self : in out Instance; Cmd : in Command.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type);

   -----------------------------------------------
   -- Parameter primitives:
   -----------------------------------------------
   -- Description:
   --    Parameters for the Inertial Filter component. The process noise and the initial
   --    covariance are given by their diagonals, since the full matrices do not fit in
   --    a parameter. TODO verify with the GNC team that diagonal matrices are enough.

   -- Invalid parameter handler. This procedure is called when a parameter's type is found to be invalid:
   -- Null: the staging code rejects the value and returns an error status to the Parameters
   -- component, which reports the offending parameter ID to the ground. That is sufficient, and
   -- we avoid adding per-component event overhead to these algorithm components.
   overriding procedure Invalid_Parameter (Self : in out Instance; Par : in Parameter.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type) is null;
   -- This procedure is called when the parameters of a component have been updated. The default implementation of this
   -- subprogram in the implementation package is a null procedure. However, this procedure can, and should be implemented if
   -- something special needs to happen after a parameter update. Examples of this might be copying certain parameters to
   -- hardware registers, or performing other special functionality that only needs to be performed after parameters have
   -- been updated.
   overriding procedure Update_Parameters_Action (Self : in out Instance);
   -- This function is called when the parameter operation type is "Validate". The default implementation of this
   -- subprogram in the implementation package is a function that returns "Valid". However, this function can, and should be
   -- overridden if something special needs to happen to further validate a parameter. Examples of this might be validation of
   -- certain parameters beyond individual type ranges, or performing other special functionality that only needs to be
   -- performed after parameters have been validated. Note that range checking is performed during staging, and does not need
   -- to be implemented here. This function is also called through Assert_Valid_Parameter_Defaults from Set_Id_Bases and from
   -- unit test setup, before the component is connected or initialized, to check the compiled-in default parameter values. The
   -- implementation must therefore be a pure function of the passed-in parameter values, with no dependence on Init state and
   -- no connector invocations.
   overriding function Validate_Parameters (
      Self : in out Instance;
      Alpha : in Packed_F64.U;
      Beta : in Packed_F64.U;
      Process_Noise_Diagonal : in Packed_F64x6.U;
      Initial_State : in Packed_F64x6.U;
      Initial_Covariance_Diagonal : in Packed_F64x6.U;
      St_Measurement_Noise_Std : in Packed_F64.U;
      Rate_Measurement_Noise_Std : in Packed_F64.U
   ) return Parameter_Validation_Status.E;

   -----------------------------------------------
   -- Data dependency primitives:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Inertial Filter component.
   -- Function which retrieves a data dependency.
   -- The default implementation is to simply call the Data_Product_Fetch_T_Request connector. Change the implementation if this component
   -- needs to do something different.
   overriding function Get_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id) return Data_Product_Return.T is (Self.Data_Product_Fetch_T_Request ((Id => Id)));

   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T);

end Component.Inertial_Filter.Implementation;
