--------------------------------------------------------------------------------
-- Dv_Accumulation Component Implementation Body
--------------------------------------------------------------------------------

with Averaged_Imu_Data;
with Packed_F32x3.C;

package body Component.Dv_Accumulation.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the delta-V accumulation algorithm with the control period and a zero
   -- accelerometer bias.
   --
   -- Init Parameters:
   -- Control_Period : Basic_Types.Positive_Short_Float - [s] Time between two
   -- algorithm updates, the integration step of each sample, so it must equal the
   -- rate at which the component is scheduled. Fixed by the assembly, so it is not a
   -- runtime parameter. The type keeps it greater than zero.
   --
   overriding procedure Init (Self : in out Instance; Control_Period : in Basic_Types.Positive_Short_Float) is
   begin
      -- Create throws on an invalid configuration. The control period's type keeps it
      -- positive, and it is finite, so this asserts a property of the type rather than
      -- checking runtime input.
      pragma Assert (Validate_Config (Control_Period => Control_Period));
      Self.Alg := Create (Control_Period => Control_Period);
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Integrate the acceleration of this tick and publish the accumulated delta-V.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      use Data_Product_Enums;
      use Data_Product_Enums.Data_Dependency_Status;

      -- Grab data dependencies:
      --
      -- Data_Dependency_Status.E can be Success, Not_Available, Error, or Stale.
      -- The IMU data is produced every tick, earlier in the same tick, so any other
      -- status indicates that this component is not wired up correctly in the
      -- algorithm execution order. That should never happen, so we assert.
      Imu : Averaged_Imu_Data.T;
      Imu_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_Body_Data (Value => Imu, Stale_Reference => Arg.Time);
      pragma Assert (Imu_Status = Success);
   begin
      -- Integrate the sample, less the commanded bias, and publish the accumulated
      -- delta-V.
      Self.Data_Product_T_Send (Self.Data_Products.Accumulated_Delta_V (
         Arg.Time,
         Packed_F32x3.C.Pack (Update (
            Self.Alg,
            R_Ddot_No_Gravity_Bn_B => (Value => Packed_F32x3.C.Unpack (Imu.Accel_Body)),
            Accel_Bias_B           => Self.Accel_Bias_B).Value)
      ));
   end Tick_T_Recv_Sync;

   -- Zero the accumulated delta-V and start a new accumulation window at the next
   -- tick. The assembly fires this when a burn is set up, before the burn executor is
   -- started.
   overriding procedure Reset_Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      Ignore : Tick.T renames Arg;
   begin
      -- Zeroes the accumulator and restarts the window only; the bias is untouched.
      Re_Initialize (Self.Alg);
   end Reset_Tick_T_Recv_Sync;

   -- Set the accelerometer bias in body frame components, the offset the measured
   -- acceleration contains, which is subtracted from each sample from the next tick
   -- on. The sender is required to send a finite vector.
   overriding procedure Accel_Bias_T_Recv_Sync (Self : in out Instance; Arg : in Packed_F32x3_Record.T) is
   begin
      -- The algorithm takes the bias with each sample, so it is converted once, here.
      Self.Accel_Bias_B := Packed_F32x3_Record.C.Unpack (Arg);
   end Accel_Bias_T_Recv_Sync;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Dv Accumulation component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Dv_Accumulation.Implementation;
