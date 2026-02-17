--------------------------------------------------------------------------------
-- Mimu_Majority_Vote Component Implementation Body
--------------------------------------------------------------------------------

package body Component.Mimu_Majority_Vote.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the MIMU majority vote algorithm.
   overriding procedure Init (Self : in out Instance) is
      -- TODO declarations
   begin
      null; -- TODO statements
   end Init;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      -- TODO declarations
   begin
      null; -- TODO statements
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
   -- Invalid Parameter handler. This procedure is called when a parameter's type is found to be invalid:
   overriding procedure Invalid_Parameter (Self : in out Instance; Par : in Parameter.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type) is
   begin
      -- TODO: Perform action to handle an invalid parameter.
      -- Example:
      -- -- Throw event:
      -- Self.Event_T_Send_If_Connected (Self.Events.Invalid_Parameter_Received (
      --    Self.Sys_Time_T_Get,
      --    (Id => Par.Header.Id, Errant_Field_Number => Errant_Field_Number, Errant_Field => Errant_Field)
      -- ));
      null;
   end Invalid_Parameter;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the MIMU Majority Vote component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
   begin
      -- TODO: Perform action to handle an invalid data dependency.
      -- Example:
      -- -- Throw event:
      -- Self.Event_T_Send_If_Connected (Self.Events.Invalid_Data_Dependency_Received (
      --    Self.Sys_Time_T_Get,
      --    (Id => Id, Request_Status => Ret.The_Status, Header => Ret.The_Data_Product.Header)
      -- ));
      null;
   end Invalid_Data_Dependency;

end Component.Mimu_Majority_Vote.Implementation;
