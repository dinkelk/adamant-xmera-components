--------------------------------------------------------------------------------
-- Nav_Aggregate Component Implementation Spec
--------------------------------------------------------------------------------

-- Includes:
with Tick;
with Nav_Aggregate_Algorithm_C; use Nav_Aggregate_Algorithm_C;

-- Navigation aggregation algorithm combines multiple navigation message sources
-- into single aggregated attitude and translational navigation messages.
package Component.Nav_Aggregate.Implementation is

   -- The component class instance record:
   type Instance is new Nav_Aggregate.Base_Instance with private;

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the navigation aggregation algorithm with configuration indices.
   --
   -- Init Parameters:
   -- Att_Time_Idx : Interfaces.Unsigned_32 - Index of message to use for attitude
   -- time tag
   -- Trans_Time_Idx : Interfaces.Unsigned_32 - Index of message to use for
   -- translation time tag
   -- Att_Idx : Interfaces.Unsigned_32 - Index of message to use for inertial MRP
   -- attitude
   -- Rate_Idx : Interfaces.Unsigned_32 - Index of message to use for attitude rate
   -- Pos_Idx : Interfaces.Unsigned_32 - Index of message to use for inertial
   -- position
   -- Vel_Idx : Interfaces.Unsigned_32 - Index of message to use for inertial
   -- velocity
   -- Dv_Idx : Interfaces.Unsigned_32 - Index of message to use for accumulated DV
   -- Sun_Idx : Interfaces.Unsigned_32 - Index of message to use for sun pointing
   -- vector
   -- Att_Msg_Count : Interfaces.Unsigned_32 - Total number of attitude messages
   -- available as inputs
   -- Trans_Msg_Count : Interfaces.Unsigned_32 - Total number of translation messages
   -- available as inputs
   --
   overriding procedure Init (Self : in out Instance; Att_Time_Idx : in Interfaces.Unsigned_32; Trans_Time_Idx : in Interfaces.Unsigned_32; Att_Idx : in Interfaces.Unsigned_32; Rate_Idx : in Interfaces.Unsigned_32; Pos_Idx : in Interfaces.Unsigned_32; Vel_Idx : in Interfaces.Unsigned_32; Dv_Idx : in Interfaces.Unsigned_32; Sun_Idx : in Interfaces.Unsigned_32; Att_Msg_Count : in Interfaces.Unsigned_32; Trans_Msg_Count : in Interfaces.Unsigned_32);
   not overriding procedure Destroy (Self : in out Instance);

private

   -- The component class instance record:
   type Instance is new Nav_Aggregate.Base_Instance with record
      Alg : Nav_Aggregate_Algorithm_Access := null;
      Att_Msg_Count : Interfaces.Unsigned_32 := 0;
      Trans_Msg_Count : Interfaces.Unsigned_32 := 0;
   end record;

   ---------------------------------------
   -- Set Up Procedure
   ---------------------------------------
   -- Null method which can be implemented to provide some component
   -- set up code. This method is generally called by the assembly
   -- main.adb after all component initialization and tasks have been started.
   -- Some activities need to only be run once at startup, but cannot be run
   -- safely until everything is up and running, ie. command registration, initial
   -- data product updates. This procedure should be implemented to do these things
   -- if necessary.
   overriding procedure Set_Up (Self : in out Instance) is null;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the aggregation algorithm up to the current time.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T);

   ---------------------------------------
   -- Invoker connector primitives:
   ---------------------------------------
   -- This procedure is called when a Data_Product_T_Send message is dropped due to a full queue.
   overriding procedure Data_Product_T_Send_Dropped (Self : in out Instance; Arg : in Data_Product.T) is null;

   -----------------------------------------------
   -- Data dependency primitives:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Nav Aggregate component.
   -- Function which retrieves a data dependency.
   -- The default implementation is to simply call the Data_Product_Fetch_T_Request connector. Change the implementation if this component
   -- needs to do something different.
   overriding function Get_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id) return Data_Product_Return.T is (Self.Data_Product_Fetch_T_Request ((Id => Id)));

   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T);

end Component.Nav_Aggregate.Implementation;
