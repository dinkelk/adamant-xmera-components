--------------------------------------------------------------------------------
-- Dv_Guidance Component Tester Body
--------------------------------------------------------------------------------

package body Component.Dv_Guidance.Implementation.Tester is

   ---------------------------------------
   -- Test initialization functions:
   ---------------------------------------
   procedure Connect (Self : in out Instance) is
   begin
      Self.Attach_Tick_T_Request (To_Component => Self.Component_Instance'Unchecked_Access, Hook => Self.Component_Instance.Tick_T_Service_Access);
      Self.Attach_Dv_Burn_Frame_T_Request (To_Component => Self.Component_Instance'Unchecked_Access, Hook => Self.Component_Instance.Dv_Burn_Frame_T_Service_Access);
   end Connect;

end Component.Dv_Guidance.Implementation.Tester;
