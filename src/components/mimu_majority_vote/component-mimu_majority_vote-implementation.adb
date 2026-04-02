--------------------------------------------------------------------------------
-- Mimu_Majority_Vote Component Implementation Body
--------------------------------------------------------------------------------

with Packed_F32x3_Record.C;
with Mimu_Majority_Vote_Output.C;
with Algorithm_Wrapper_Util;

package body Component.Mimu_Majority_Vote.Implementation is

   -- Number of active IMUs passed to the majority vote algorithm. The algorithm
   -- compares angular velocity inputs, detects outliers above a threshold, and
   -- recomputes the average excluding faulted inputs. This wrapper constrains
   -- the number of active IMUs to 3, although the algorithm can support more.
   Num_Active_Imus : constant := MAX_IMU_VEH_COUNT - 1;
   pragma Compile_Time_Error (Num_Active_Imus /= 3,
      "This wrapper requires exactly 3 active IMUs");

   -- Array type for passing IMU inputs to the C algorithm.
   type Imu_Input_Array is array (Natural range 0 .. Num_Active_Imus - 1) of aliased Packed_F32x3_Record.C.U_C;

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

      -- Grab data dependencies for the 3 active IMU body data inputs:
      Imu_1 : Averaged_Imu_Data.T;
      Imu_1_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_1_Body (Value => Imu_1, Stale_Reference => Arg.Time);
      Imu_2 : Averaged_Imu_Data.T;
      Imu_2_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_2_Body (Value => Imu_2, Stale_Reference => Arg.Time);
      Imu_3 : Averaged_Imu_Data.T;
      Imu_3_Status : constant Data_Dependency_Status.E :=
         Self.Get_Imu_3_Body (Value => Imu_3, Stale_Reference => Arg.Time);
   begin
      -- Update the parameters:
      Self.Update_Parameters;

      -- Check all dependencies are available. If any IMU data is stale,
      -- skip the update entirely — no output is produced this tick.
      -- TODO: Verify stale data policy with algorithm owner. If the
      -- algorithm can tolerate stale inputs, consider always calling
      -- Update and asserting statuses instead.
      if Is_Dep_Status_Success (Imu_1_Status) and then
         Is_Dep_Status_Success (Imu_2_Status) and then
         Is_Dep_Status_Success (Imu_3_Status)
      then
         declare
            -- Extract angular velocity from each IMU and build C input array:
            Imu_Inputs : aliased Imu_Input_Array :=
               [Packed_F32x3_Record.C.Unpack ((Value => Imu_1.Ang_Vel_Body)),
                Packed_F32x3_Record.C.Unpack ((Value => Imu_2.Ang_Vel_Body)),
                Packed_F32x3_Record.C.Unpack ((Value => Imu_3.Ang_Vel_Body))];

            -- Call the C algorithm:
            Result : constant Mimu_Majority_Vote_Output.C.U_C := Update (
               Self.Alg,
               Imu_Inputs     => Imu_Inputs (Imu_Inputs'First)'Access,
               Number_Of_Imus => Num_Active_Imus
            );
            Packed_Result : constant Mimu_Majority_Vote_Output.T :=
               Mimu_Majority_Vote_Output.C.Pack (Result);
         begin
            -- Publish full result with fault status:
            Self.Data_Product_T_Send (Self.Data_Products.Majority_Vote_Result (
               Arg.Time,
               Packed_Result
            ));

            -- Publish fault-excluded averaged angular velocity:
            Self.Data_Product_T_Send (Self.Data_Products.Voted_Ang_Vel_Body (
               Arg.Time,
               (Value => Packed_Result.Avg_Ang_Vel_Body)
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
   begin
      -- Throw event:
      Self.Event_T_Send_If_Connected (Self.Events.Invalid_Parameter_Received (
         Self.Sys_Time_T_Get,
         (Id => Par.Header.Id, Errant_Field_Number => Errant_Field_Number, Errant_Field => Errant_Field)
      ));
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
