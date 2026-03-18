--------------------------------------------------------------------------------
-- Average_Mimu_Data Component Implementation Body
--------------------------------------------------------------------------------

with Mimu_Data_Field_Sample_10;
with Averaged_Imu_Data;
with Packed_F32x9.C;

package body Component.Average_Mimu_Data.Implementation is

   -- Inter-sample period in nanoseconds (10 ms):
   Sample_Period_Ns : constant Interfaces.Unsigned_64 := 10_000_000;

   -- Number of raw samples per packet:
   Num_Samples : constant := 10;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the AverageMimuData algorithm.
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
   -- Receive raw MIMU data packet and buffer for later processing.
   overriding procedure Mimu_Raw_Packet_T_Recv_Sync (Self : in out Instance; Arg : in Mimu_Raw_Packet.T) is
   begin
      Self.Update_Parameters;

      if Self.Packet_Count >= Max_Buffered_Packets then
         -- Buffer full, drop the incoming packet.
         Self.Event_T_Send_If_Connected (Self.Events.Packet_Buffer_Overflow (Self.Sys_Time_T_Get));
      else
         Self.Packets (Self.Packet_Count) := Arg;
         Self.Packet_Count := Self.Packet_Count + 1;
      end if;
   end Mimu_Raw_Packet_T_Recv_Sync;

   -- Tick that triggers the averaging algorithm over buffered samples and publishes
   -- the result.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      Ignore : Tick.T renames Arg;
   begin
      -- Nothing to process if no packets have been buffered:
      if Self.Packet_Count = 0 then
         return;
      end if;

      declare
         -- Build 120-element InputPktsData_c. Zero-initialized so unused
         -- slots have measTime=0, which the time-window filter excludes.
         Input : aliased Input_Pkts_Data_C := (
            Meas_Time => [others => 0],
            Gyro_P    => [others => [others => 0.0]],
            Accel_P   => [others => [others => 0.0]]
         );
         Offset : Natural := 0;
      begin
         -- Convert all buffered raw packets into the algorithm input buffer:
         for P in 0 .. Self.Packet_Count - 1 loop
            declare
               Pkt : Mimu_Raw_Packet.T renames Self.Packets (P);

               Base_Time_Ns : constant Interfaces.Unsigned_64 :=
                  Interfaces.Unsigned_64 (Pkt.Timestamp.Seconds) * 1_000_000_000 +
                  Interfaces.Unsigned_64 (Pkt.Timestamp.Subseconds) * 1_000_000_000 / 65_536;

               Samples : constant Mimu_Data_Field_Sample_10.U :=
                  Mimu_Data_Field_Sample_10.Unpack (Pkt.Samples);
            begin
               for I in 0 .. Num_Samples - 1 loop
                  Input.Meas_Time (Offset + I) := Base_Time_Ns + Interfaces.Unsigned_64 (I) * Sample_Period_Ns;
                  Input.Gyro_P (Offset + I) := [
                     Short_Float (Samples (I).Merged_Gyro_Rates.X_Measurement) * Self.Gyro_Scale.Value,
                     Short_Float (Samples (I).Merged_Gyro_Rates.Y_Measurement) * Self.Gyro_Scale.Value,
                     Short_Float (Samples (I).Merged_Gyro_Rates.Z_Measurement) * Self.Gyro_Scale.Value
                  ];
                  Input.Accel_P (Offset + I) := [
                     Short_Float (Samples (I).Merged_Accelerations.X_Measurement) * Self.Accel_Scale.Value,
                     Short_Float (Samples (I).Merged_Accelerations.Y_Measurement) * Self.Accel_Scale.Value,
                     Short_Float (Samples (I).Merged_Accelerations.Z_Measurement) * Self.Accel_Scale.Value
                  ];
               end loop;
               Offset := Offset + Num_Samples;
            end;
         end loop;

         declare
            Output : constant Output_Average_Accel_Angle_Vel_C :=
               Update (Self.Alg, Input'Unchecked_Access);
         begin
            Self.Data_Product_T_Send (Self.Data_Products.Imu_Body_Data (
               Self.Sys_Time_T_Get,
               Averaged_Imu_Data.Pack ((
                  Accel_Body   => [Output.Accel_B (0),
                                   Output.Accel_B (1),
                                   Output.Accel_B (2)],
                  Ang_Vel_Body => [Output.Gyro_Omega_B (0),
                                   Output.Gyro_Omega_B (1),
                                   Output.Gyro_Omega_B (2)]
               ))
            ));
         end;
      end;

      -- Reset buffer for next cycle:
      Self.Packet_Count := 0;
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
   --    Parameters for the Average Mimu Data component
   -- Apply parameter values to the C++ algorithm when parameters change.
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
   begin
      -- Set the averaging window:
      Set_Averaging_Window (Self.Alg, Self.Time_Delta.Value);
      -- Set the platform-to-body DCM:
      Set_Dcm_Pltf_To_Bdy (Self.Alg, (Value => Packed_F32x9.C.To_C (Self.Dcm_Pltf_To_Bdy)));
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

end Component.Average_Mimu_Data.Implementation;
