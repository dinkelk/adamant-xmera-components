--------------------------------------------------------------------------------
-- Mimu_Majority_Vote Component Implementation Body
--------------------------------------------------------------------------------

with Packed_F32x3_Record.C;
with Mimu_Majority_Vote_Output.C;
with Algorithm_Wrapper_Util;

package body Component.Mimu_Majority_Vote.Implementation is

   -- Array type for passing IMU inputs to the C algorithm.
   type Imu_Input_Array is array (Natural range 0 .. MAX_IMU_VEH_COUNT - 1) of aliased Packed_F32x3_Record.C.U_C;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the MIMU majority vote algorithm.
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

      -- Grab data dependencies for all 4 IMU angular velocity inputs:
      Imu_1 : Packed_F32x3_Record.T;
      Imu_1_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_1_Ang_Vel_Body (Value => Imu_1, Stale_Reference => Arg.Time);
      Imu_2 : Packed_F32x3_Record.T;
      Imu_2_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_2_Ang_Vel_Body (Value => Imu_2, Stale_Reference => Arg.Time);
      Imu_3 : Packed_F32x3_Record.T;
      Imu_3_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_3_Ang_Vel_Body (Value => Imu_3, Stale_Reference => Arg.Time);
      Imu_4 : Packed_F32x3_Record.T;
      Imu_4_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_4_Ang_Vel_Body (Value => Imu_4, Stale_Reference => Arg.Time);
   begin
      -- Update the parameters:
      Self.Update_Parameters;

      -- Check all dependencies are available:
      if Is_Dep_Status_Success (Imu_1_Status) and then
         Is_Dep_Status_Success (Imu_2_Status) and then
         Is_Dep_Status_Success (Imu_3_Status) and then
         Is_Dep_Status_Success (Imu_4_Status)
      then
         declare
            -- Convert Ada packed types to C types and build input array:
            Imu_Inputs : aliased Imu_Input_Array :=
               [Packed_F32x3_Record.C.To_C (Packed_F32x3_Record.Unpack (Imu_1)),
                Packed_F32x3_Record.C.To_C (Packed_F32x3_Record.Unpack (Imu_2)),
                Packed_F32x3_Record.C.To_C (Packed_F32x3_Record.Unpack (Imu_3)),
                Packed_F32x3_Record.C.To_C (Packed_F32x3_Record.Unpack (Imu_4))];

            -- Call the C algorithm:
            Result : constant Mimu_Majority_Vote_Output.C.U_C := Update (
               Self.Alg,
               Imu_Inputs     => Imu_Inputs (Imu_Inputs'First)'Unchecked_Access,
               Number_Of_Imus => MAX_IMU_VEH_COUNT
            );
         begin
            -- Send out data product:
            Self.Data_Product_T_Send (Self.Data_Products.Majority_Vote_Result (
               Arg.Time,
               Mimu_Majority_Vote_Output.Pack (Mimu_Majority_Vote_Output.C.To_Ada (Result))
            ));
         end;
      end if;
   end Tick_T_Recv_Sync;

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
   --    Parameters for the MIMU Majority Vote component
   -- This procedure is called when the parameters of a component have been updated.
   -- Apply the omega threshold parameter to the C algorithm.
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
   begin
      Set_Omega_Threshold (Self.Alg, Self.Omega_Threshold.Value);
   end Update_Parameters_Action;

   -- Invalid Parameter handler. This procedure is called when a parameter's type is found to be invalid:
   overriding procedure Invalid_Parameter (Self : in out Instance; Par : in Parameter.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the parameters should be invalid in this case.
      pragma Assert (False);
   end Invalid_Parameter;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the MIMU Majority Vote component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Mimu_Majority_Vote.Implementation;
