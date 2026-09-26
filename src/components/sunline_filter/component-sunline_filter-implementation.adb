--------------------------------------------------------------------------------
-- Sunline_Filter Component Implementation Body
--------------------------------------------------------------------------------

with Css_Sensor_Values;
with Nav_Att_Output;
with Packed_F32x24;
with Packed_F32x3;
with Packed_F64x3.C;
with Packed_F64x7.C;
with Packed_F64x8;
with Packed_F64x8.C;
with Packed_F64x24.C;
with Packed_F64x49.C;
with Sunline_Filter_Css_Data.C;
with Sunline_Filter_Css_Fit_Residuals;
with Sunline_Filter_Css_Matrix.C;
with Sunline_Filter_Css_Residuals.C;
with Sunline_Filter_Css_Vector.C;
with Sunline_Filter_Output.C;
with Sunline_Filter_Rate_Data.C;
with Sunline_Filter_Rate_Fit_Residuals;
with Sunline_Filter_Rate_Residuals.C;
with Sunline_Filter_State;
with Sunline_Filter_State_Matrix.C;
with Sunline_Filter_State_Vector.C;
with Sys_Time.Arithmetic;

package body Component.Sunline_Filter.Implementation is

   -- The parts of the configuration the shim takes by pointer, held together so a
   -- caller can pass 'Access of each field. Init, Apply_Config, and
   -- Validate_Parameters all marshal the same values, so it is assembled in one place.
   type Pointer_Config is record
      Process_Noise : aliased Sunline_Filter_State_Matrix.C.U_C;
      Initial_State : aliased Sunline_Filter_State_Vector.C.U_C;
      Initial_Covariance : aliased Sunline_Filter_State_Matrix.C.U_C;
      Css_N_Hat : aliased Sunline_Filter_Css_Matrix.C.U_C;
      Css_Scale_Factor : aliased Sunline_Filter_Css_Vector.C.U_C;
   end record;

   -- Expand a diagonal into the row major N x N matrix the shim takes. The
   -- parameters carry only the diagonals, since the full matrices do not fit in a
   -- parameter.
   function Diagonal_Matrix (Diagonal : in Packed_F64x7.U) return Packed_F64x49.U is
      Matrix : Packed_F64x49.U := [others => 0.0];
   begin
      for I in Diagonal'Range loop
         Matrix (I * Packed_F64x7.Length + I) := Diagonal (I);
      end loop;
      return Matrix;
   end Diagonal_Matrix;

   -- Marshal the pointer arguments of the configuration. The boresight table is a
   -- single precision parameter, since the double one would not fit in the parameter
   -- buffer, and is widened here.
   -- TODO: Widening a parameter to feed the algorithm is a mismatch. Either the
   -- parameter buffer grows so the table can be double precision, or the algorithm
   -- takes the table in single precision, and then this conversion goes away.
   function To_Pointer_Config (
      Process_Noise_Diagonal : in Packed_F64x7.U;
      Initial_State : in Packed_F64x7.U;
      Initial_Covariance_Diagonal : in Packed_F64x7.U;
      Css_N_Hat_B : in Packed_F32x24.U;
      Css_Scale_Factor : in Packed_F64x8.U
   ) return Pointer_Config is
      (Process_Noise => (Value => Packed_F64x49.C.To_C (Diagonal_Matrix (Process_Noise_Diagonal))),
       Initial_State => (Value => Packed_F64x7.C.To_C (Initial_State)),
       Initial_Covariance => (Value => Packed_F64x49.C.To_C (Diagonal_Matrix (Initial_Covariance_Diagonal))),
       Css_N_Hat => (Value => Packed_F64x24.C.To_C ([for I in Css_N_Hat_B'Range => Long_Float (Css_N_Hat_B (I))])),
       Css_Scale_Factor => (Value => Packed_F64x8.C.To_C (Css_Scale_Factor)));

   -- Time is kept in nanoseconds, the unit of the tick's call time, and the product
   -- timestamps are converted to it, so measurement times are compared and
   -- differenced exactly. This converts back to system time, for reporting a
   -- timestamp kept in nanoseconds.
   Ns_Per_Second : constant Unsigned_64 := 1_000_000_000;
   function To_Sys_Time (Ns : in Unsigned_64) return Sys_Time.U is
      (Seconds => Unsigned_32 (Ns / Ns_Per_Second),
       Subseconds => Sys_Time.Subseconds_Type (((Ns mod Ns_Per_Second) * Unsigned_64 (Sys_Time.Subseconds_Type'Modulus)) / Ns_Per_Second));

   -- Seconds since the filter's time base.
   function Filter_Seconds (Self : in Instance; Ns : in Unsigned_64) return Long_Float is
      (Long_Float (Ns - Self.Epoch_Ns) * 1.0E-9);

   -- The residuals of each measurement kind as published. The measured values are
   -- left out, since the sensor products already carry them. The sun sensor residuals
   -- before and after the update are separate products, since together they would
   -- not fit in one.
   function To_Css_Fit_Residuals (Residuals : in Sunline_Filter_Css_Residuals.C.U_C; Fit : in Packed_F64x8.C.U_C) return Sunline_Filter_Css_Fit_Residuals.T is
      (Sunline_Filter_Css_Fit_Residuals.Pack ((
         Valid                => Boolean (Residuals.Valid),
         Number_Of_Active_Css => Residuals.Number_Of_Active_Css,
         Residuals            => Packed_F64x8.C.To_Ada (Fit))));
   function To_Rate_Fit_Residuals (Residuals : in Sunline_Filter_Rate_Residuals.C.U_C) return Sunline_Filter_Rate_Fit_Residuals.T is
      (Sunline_Filter_Rate_Fit_Residuals.Pack ((
         Valid    => Boolean (Residuals.Valid),
         Pre_Fit  => Packed_F64x3.C.To_Ada (Residuals.Pre_Fit),
         Post_Fit => Packed_F64x3.C.To_Ada (Residuals.Post_Fit))));

   -- Whether a product stamped at Ns is a new measurement: it must be newer than the
   -- last one consumed, lie within the filter's time base, and not be after the tick,
   -- since the filter only advances to the tick. Last is advanced to it only when it
   -- is fed, so a stamp from the future waits rather than shutting out the readings
   -- that follow.
   function Is_Fresh (Self : in Instance; Ns : in Unsigned_64; Tick_Ns : in Unsigned_64; Last : in out Unsigned_64) return Boolean is
   begin
      if Ns > Last and then Ns > Self.Epoch_Ns and then Ns <= Tick_Ns then
         Last := Ns;
         return True;
      end if;
      return False;
   end Is_Fresh;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the sunline filter with the default parameter values, which seed
   -- the filter state and covariance.
   overriding procedure Init (Self : in out Instance) is
      Cfg : aliased constant Pointer_Config := To_Pointer_Config (Self.Process_Noise_Diagonal, Self.Initial_State, Self.Initial_Covariance_Diagonal, Self.Css_N_Hat_B, Self.Css_Scale_Factor);
   begin
      -- Create throws on an invalid configuration, so the parameter defaults must form a
      -- valid one. The generated Assert_Valid_Parameter_Defaults checks them at startup
      -- and in unit test set up.
      Self.Alg := Create (
         Alpha                      => Self.Alpha.Value,
         Beta                       => Self.Beta.Value,
         Process_Noise              => Cfg.Process_Noise'Access,
         Initial_State              => Cfg.Initial_State'Access,
         Initial_Covariance         => Cfg.Initial_Covariance'Access,
         Bias_Lower_Bound           => Self.Bias_Lower_Bound.Value,
         Bias_Upper_Bound           => Self.Bias_Upper_Bound.Value,
         Css_N_Hat                  => Cfg.Css_N_Hat'Access,
         Css_Scale_Factor           => Cfg.Css_Scale_Factor'Access,
         Number_Of_Css              => Self.Number_Of_Css.Value,
         Sensor_Threshold           => Self.Sensor_Threshold.Value,
         Css_Measurement_Noise_Std  => Self.Css_Measurement_Noise_Std.Value,
         Gyro_Measurement_Noise_Std => Self.Gyro_Measurement_Noise_Std.Value);
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the filter up to the tick's call time, folding in the coarse sun sensor and
   -- body rate readings that are new since the last tick.
   overriding procedure Algorithm_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Algorithm_Tick.T) is
      use Data_Product_Enums;
      use Data_Product_Enums.Data_Dependency_Status;

      -- Grab data dependencies:
      --
      -- Data_Dependency_Status.E can be Success, Not_Available, Error, or Stale.
      -- Both products are produced earlier in the same tick, so any other status
      -- indicates that this component is not wired up correctly in the algorithm
      -- execution order. That should never happen, so we assert. The timestamp of
      -- each product is the time of its measurement.
      Body_Rate : Packed_F32x3.T;
      Body_Rate_Time : Sys_Time.T;
      Body_Rate_Status : constant Data_Dependency_Status.E :=
         Self.Get_Body_Rate (Value => Body_Rate, Timestamp => Body_Rate_Time, Stale_Reference => Arg.Current_Tick.Time);
      pragma Assert (Body_Rate_Status = Success);
      Css : Css_Sensor_Values.T;
      Css_Time : Sys_Time.T;
      Css_Status : constant Data_Dependency_Status.E :=
         Self.Get_Css_Cosines (Value => Css, Timestamp => Css_Time, Stale_Reference => Arg.Current_Tick.Time);
      pragma Assert (Css_Status = Success);

      Tick_Ns : Unsigned_64 renames Arg.Call_Time;

      -- Widen the single precision body rate to the double precision the shim takes.
      -- The vector is unpacked first, so each element is read whole before it is
      -- converted.
      -- TODO: The producer publishes single precision and the algorithm takes double.
      -- One side should change so the conversion goes away.
      function To_C (Vector : in Packed_F32x3.T) return Packed_F64x3.C.U_C is
         Unpacked : constant Packed_F32x3.U := Packed_F32x3.Unpack (Vector);
      begin
         return Packed_F64x3.C.To_C ([for I in Unpacked'Range => Long_Float (Unpacked (I))]);
      end To_C;
   begin
      -- Apply any pending parameter update:
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
         Self.Last_Rate_Time_Ns := 0;
         Self.Last_Css_Time_Ns := 0;
      end if;

      declare
         -- A product is a new measurement only when its timestamp has advanced since
         -- the last one consumed. A time tag of zero tells the filter there is no new
         -- reading of that kind.
         Rate_Fresh : constant Boolean := Is_Fresh (Self, Sys_Time.Arithmetic.To_Nanoseconds (Body_Rate_Time), Tick_Ns, Self.Last_Rate_Time_Ns);
         Css_Fresh : constant Boolean := Is_Fresh (Self, Sys_Time.Arithmetic.To_Nanoseconds (Css_Time), Tick_Ns, Self.Last_Css_Time_Ns);
         Rate_Data : aliased constant Sunline_Filter_Rate_Data.C.U_C :=
            (Time_Tag => (if Rate_Fresh then Filter_Seconds (Self, Self.Last_Rate_Time_Ns) else 0.0),
             Rate => To_C (Body_Rate));
         Css_Data : aliased constant Sunline_Filter_Css_Data.C.U_C :=
            (Time_Tag => (if Css_Fresh then Filter_Seconds (Self, Self.Last_Css_Time_Ns) else 0.0),
             Cos_Values => Packed_F64x8.C.Unpack (Css.Data));

         -- Advance the filter and take its snapshot. The estimate is the first three
         -- states, the sun direction, and the next three, the body rate.
         Output : constant Sunline_Filter_Output.C.U_C := Update (
            Self.Alg,
            Current_Seconds => Filter_Seconds (Self, Tick_Ns),
            Css_Data        => Css_Data'Access,
            Rate_Data       => Rate_Data'Access);
      begin
         -- Publish the estimate for the downstream algorithms, narrowed to the single
         -- precision they consume, and stamped in system seconds.
         Self.Data_Product_T_Send (Self.Data_Products.Sun_Direction_Estimate (
            Arg.Current_Tick.Time,
            Nav_Att_Output.Pack ((
               Time_Tag        => Long_Float (Tick_Ns) * 1.0E-9,
               -- The estimate shares the navigation attitude output type with the
               -- inertial filter, but this filter estimates only the sun direction and
               -- the body rate. It has no attitude state, so the attitude field is
               -- always zero and consumers must not read it from this product.
               Sigma_Bn        => [0.0, 0.0, 0.0],
               Omega_Bn_B      => [for I in 0 .. 2 => Short_Float (Output.Filter_State.State (I + 3))],
               Veh_Sun_Pnt_Bdy => [for I in 0 .. 2 => Short_Float (Output.Filter_State.State (I))]))
         ));

         -- Publish the rest of the snapshot for the ground. The covariance is reduced to
         -- its diagonal, which is all that is needed, and the times of the last readings
         -- applied go along so the time of each last measurement update is known once
         -- the product is sampled into a packet. The residuals carry whether their
         -- measurement fired this tick for the same reason.
         Self.Data_Product_T_Send (Self.Data_Products.Filter_State (Arg.Current_Tick.Time, Sunline_Filter_State.Pack ((
            State                      => Packed_F64x7.C.To_Ada (Output.Filter_State.State),
            Covariance_Diagonal        => [for I in 0 .. Packed_F64x7.Length - 1 => Output.Filter_State.Covariance (I * Packed_F64x7.Length + I)],
            Last_Css_Measurement_Time  => To_Sys_Time (Self.Last_Css_Time_Ns),
            Last_Rate_Measurement_Time => To_Sys_Time (Self.Last_Rate_Time_Ns)))));
         Self.Data_Product_T_Send (Self.Data_Products.Css_Pre_Fit_Residuals (Arg.Current_Tick.Time, To_Css_Fit_Residuals (Output.Css_Residuals, Output.Css_Residuals.Pre_Fit)));
         Self.Data_Product_T_Send (Self.Data_Products.Css_Post_Fit_Residuals (Arg.Current_Tick.Time, To_Css_Fit_Residuals (Output.Css_Residuals, Output.Css_Residuals.Post_Fit)));
         Self.Data_Product_T_Send (Self.Data_Products.Rate_Residuals (Arg.Current_Tick.Time, To_Rate_Fit_Residuals (Output.Rate_Residuals)));
      end;
   end Algorithm_Tick_T_Recv_Sync;

   -- Re-seed the filter state and covariance from the configured initial values and
   -- clear the pending measurements and residuals. The filter's time base restarts at
   -- the next tick. The assembly fires this on every entry to the navigation state,
   -- where the filter is first put into use.
   overriding procedure Reset_Estimate_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      Ignore : Tick.T renames Arg;
   begin
      Re_Initialize (Self.Alg);
      Self.Restart_Time_Base := True;
   end Reset_Estimate_Tick_T_Recv_Sync;

   -- Clear the pending measurements and residuals, keeping the filter state and
   -- covariance. The filter's time base restarts at the next tick. The assembly fires
   -- this on every transition between pointing states.
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

   -----------------------------------------------
   -- Parameter handlers:
   -----------------------------------------------
   -- Description:
   --    Parameters for the Sunline Filter component.
   -- This procedure is called when the parameters of a component have been updated. In this
   -- case we push the whole configuration into the C algorithm, which keeps the current
   -- estimate. The values were checked at staging by Validate_Parameters, so Set_Config
   -- does not throw.
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
      Cfg : aliased constant Pointer_Config := To_Pointer_Config (Self.Process_Noise_Diagonal, Self.Initial_State, Self.Initial_Covariance_Diagonal, Self.Css_N_Hat_B, Self.Css_Scale_Factor);
   begin
      Set_Config (
         Self.Alg,
         Alpha                      => Self.Alpha.Value,
         Beta                       => Self.Beta.Value,
         Process_Noise              => Cfg.Process_Noise'Access,
         Initial_State              => Cfg.Initial_State'Access,
         Initial_Covariance         => Cfg.Initial_Covariance'Access,
         Bias_Lower_Bound           => Self.Bias_Lower_Bound.Value,
         Bias_Upper_Bound           => Self.Bias_Upper_Bound.Value,
         Css_N_Hat                  => Cfg.Css_N_Hat'Access,
         Css_Scale_Factor           => Cfg.Css_Scale_Factor'Access,
         Number_Of_Css              => Self.Number_Of_Css.Value,
         Sensor_Threshold           => Self.Sensor_Threshold.Value,
         Css_Measurement_Noise_Std  => Self.Css_Measurement_Noise_Std.Value,
         Gyro_Measurement_Noise_Std => Self.Gyro_Measurement_Noise_Std.Value);
   end Update_Parameters_Action;

   -- Validate a staged parameter set before it is applied by asking the algorithm's own
   -- non-throwing Validate_Config predicate, so the config rules live solely in the
   -- algorithm. Rejecting an invalid update here at staging keeps it from reaching the
   -- throwing Set_Config across the FFI boundary.
   overriding function Validate_Parameters (
      Self : in out Instance;
      Alpha : in Packed_F64.U;
      Beta : in Packed_F64.U;
      Process_Noise_Diagonal : in Packed_F64x7.U;
      Initial_State : in Packed_F64x7.U;
      Initial_Covariance_Diagonal : in Packed_F64x7.U;
      Bias_Lower_Bound : in Packed_F64.U;
      Bias_Upper_Bound : in Packed_F64.U;
      Css_N_Hat_B : in Packed_F32x24.U;
      Css_Scale_Factor : in Packed_F64x8.U;
      Number_Of_Css : in Packed_U32.U;
      Sensor_Threshold : in Packed_F64.U;
      Css_Measurement_Noise_Std : in Packed_F64.U;
      Gyro_Measurement_Noise_Std : in Packed_F64.U
   ) return Parameter_Validation_Status.E is
      Ignore : Instance renames Self;
      -- Filled in below, inside the handled part of the function, so that a conversion
      -- that raises is caught here.
      Cfg : aliased Pointer_Config;
   begin
      Cfg := To_Pointer_Config (Process_Noise_Diagonal, Initial_State, Initial_Covariance_Diagonal, Css_N_Hat_B, Css_Scale_Factor);
      if Validate_Config (
         Alpha                      => Alpha.Value,
         Beta                       => Beta.Value,
         Process_Noise              => Cfg.Process_Noise'Access,
         Initial_State              => Cfg.Initial_State'Access,
         Initial_Covariance         => Cfg.Initial_Covariance'Access,
         Bias_Lower_Bound           => Bias_Lower_Bound.Value,
         Bias_Upper_Bound           => Bias_Upper_Bound.Value,
         Css_N_Hat                  => Cfg.Css_N_Hat'Access,
         Css_Scale_Factor           => Cfg.Css_Scale_Factor'Access,
         Number_Of_Css              => Number_Of_Css.Value,
         Sensor_Threshold           => Sensor_Threshold.Value,
         Css_Measurement_Noise_Std  => Css_Measurement_Noise_Std.Value,
         Gyro_Measurement_Noise_Std => Gyro_Measurement_Noise_Std.Value)
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
   --    Data dependencies for the Sunline Filter component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Sunline_Filter.Implementation;
