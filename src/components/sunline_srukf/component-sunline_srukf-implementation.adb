--------------------------------------------------------------------------------
-- Sunline_Srukf Component Implementation Body
--------------------------------------------------------------------------------

with Nav_Att.C;
with Sunline_Srukf_Input.C;
with Sunline_Srukf_Output.C;
with Sunline_Srukf_Algorithm_C; use Sunline_Srukf_Algorithm_C;
with Algorithm_Wrapper_Util;

package body Component.Sunline_Srukf.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the sunline SRuKF algorithm.
   overriding procedure Init (Self : in out Instance) is
      pragma Unreferenced (Self);
   begin
      -- Stateless algorithm, nothing to initialize.
      null;
   end Init;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Run the algorithm up to the current time.
   overriding procedure Tick_T_Recv_Sync (Self : in out Instance; Arg : in Tick.T) is
      use Data_Product_Enums;
      use Data_Product_Enums.Data_Dependency_Status;
      use Algorithm_Wrapper_Util;

      -- Grab data dependencies:
      Sc_Att : Nav_Att.T;
      Sc_Att_Status : constant Data_Dependency_Status.E :=
         Self.Get_Spacecraft_Attitude (Value => Sc_Att, Stale_Reference => Arg.Time);
   begin
      if Is_Dep_Status_Success (Sc_Att_Status) then
         -- Call algorithm:
         declare
            Sc_Att_C : constant Nav_Att.C.U_C := Nav_Att.C.To_C (Nav_Att.Unpack (Sc_Att));

            -- Build algorithm input from nav att data:
            Input_C : aliased Sunline_Srukf_Input.C.U_C := (
               Time_Tag        => Sc_Att_C.Time_Tag,
               Sigma_Bn        => Sc_Att_C.Sigma_Bn,
               Omega_Bn_B      => Sc_Att_C.Omega_Bn_B,
               Veh_Sun_Pnt_Bdy => Sc_Att_C.Veh_Sun_Pnt_Bdy,
               N_Css           => 0,
               Cos_Values      => [others => 0.0]
            );

            -- Call the C algorithm:
            Output_C : constant Sunline_Srukf_Output.C.U_C := Update_State (
               Input => Input_C'Unchecked_Access
            );
         begin
            -- Send out data product:
            Self.Data_Product_T_Send (Self.Data_Products.Sunline_Srukf_State (
               Arg.Time,
               Sunline_Srukf_Output.Pack (Sunline_Srukf_Output.C.To_Ada (Output_C))
            ));
         end;
      end if;
   end Tick_T_Recv_Sync;

   -----------------------------------------------
   -- Data dependency handlers:
   -----------------------------------------------
   -- Description:
   --    Data dependencies for the Sunline SRuKF component.
   -- Invalid data dependency handler. This procedure is called when a data dependency's id or length are found to be invalid:
   overriding procedure Invalid_Data_Dependency (Self : in out Instance; Id : in Data_Product_Types.Data_Product_Id; Ret : in Data_Product_Return.T) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the data dependencies should be invalid in this case.
      pragma Assert (False);
   end Invalid_Data_Dependency;

end Component.Sunline_Srukf.Implementation;
