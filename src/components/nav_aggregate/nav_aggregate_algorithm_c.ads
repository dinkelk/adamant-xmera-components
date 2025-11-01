pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Nav_Att.C;
with Nav_Trans.C;

package Nav_Aggregate_Algorithm_C is

   -- MIT License
   -- *
   -- * Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
   -- * University of Colorado at Boulder
   -- *
   -- * Permission is hereby granted, free of charge, to any person
   -- * obtaining a copy of this software and associated documentation
   -- * files (the "Software"), to deal in the Software without restriction,
   -- * including without limitation the rights to use, copy, modify, merge,
   -- * publish, distribute, sublicense, and/or sell copies of the Software,
   -- * and to permit persons to whom the Software is furnished to do so,
   -- * subject to the following conditions:
   -- *
   -- * The above copyright notice and this permission notice shall be
   -- * included in all copies or substantial portions of the Software.
   -- *
   -- * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   -- * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   -- * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   -- * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
   -- * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   -- * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   -- * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   -- * SOFTWARE.
   --

   -- MAX_AGG_NAV_MSG must match the #define in navAggregateAlgorithm_c.h:13
   -- Re-run h2ads if the C header changes to regenerate this binding
   MAX_AGG_NAV_MSG : constant := 10;

   --* @brief Get the maximum aggregate navigation message count.
   --* @return The maximum message count (MAX_AGG_NAV_MSG).
   function Get_Max_Agg_Nav_Msg
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getMaxAggNavMsg";

   -- Runtime validation: ensure Ada constant matches C definition
   pragma Assert (Unsigned_32 (MAX_AGG_NAV_MSG) = Get_Max_Agg_Nav_Msg);

   --* Opaque handle for a NavAggregateAlgorithm instance.
   type Nav_Aggregate_Algorithm is limited private;
   type Nav_Aggregate_Algorithm_Access is access all Nav_Aggregate_Algorithm;

   --* Structure containing the attitude and translational navigation output messages.
   type Aggregate_Output is record
      Nav_Att_Out   : aliased Nav_Att.C.U_C;
      Nav_Trans_Out : aliased Nav_Trans.C.U_C;
   end record
   with Convention => C_Pass_By_Copy;

   --* @brief Construct a new NavAggregateAlgorithm.
   function Create
     return Nav_Aggregate_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_create";

   --* @brief Destroy a NavAggregateAlgorithm.
   procedure Destroy
     (Self : Nav_Aggregate_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_destroy";

   --* @brief Run the update step.
   --* @param Self                Pointer to the instance.
   --* @param Att_Msgs_Payloads   Pointer to array of attitude navigation message payloads.
   --* @param Trans_Msgs_Payloads Pointer to array of translational navigation message payloads.
   --* @return The computed output messages.
   function Update
     (Self                : Nav_Aggregate_Algorithm_Access;
      Att_Msgs_Payloads   : Nav_Att.C.U_C_Access;
      Trans_Msgs_Payloads : Nav_Trans.C.U_C_Access)
     return Aggregate_Output
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_update";

   --* @brief Set the attitude time index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new attitude time index to set.
   procedure Set_Att_Time_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setAttTimeIdx";

   --* @brief Get the current attitude time index.
   --* @param Self Pointer to the instance.
   --* @return The current attitude time index.
   function Get_Att_Time_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getAttTimeIdx";

   --* @brief Set the translation time index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new translation time index to set.
   procedure Set_Trans_Time_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setTransTimeIdx";

   --* @brief Get the current translation time index.
   --* @param Self Pointer to the instance.
   --* @return The current translation time index.
   function Get_Trans_Time_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getTransTimeIdx";

   --* @brief Set the attitude index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new attitude index to set.
   procedure Set_Att_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setAttIdx";

   --* @brief Get the current attitude index.
   --* @param Self Pointer to the instance.
   --* @return The current attitude index.
   function Get_Att_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getAttIdx";

   --* @brief Set the rate index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new rate index to set.
   procedure Set_Rate_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setRateIdx";

   --* @brief Get the current rate index.
   --* @param Self Pointer to the instance.
   --* @return The current rate index.
   function Get_Rate_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getRateIdx";

   --* @brief Set the position index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new position index to set.
   procedure Set_Pos_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setPosIdx";

   --* @brief Get the current position index.
   --* @param Self Pointer to the instance.
   --* @return The current position index.
   function Get_Pos_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getPosIdx";

   --* @brief Set the velocity index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new velocity index to set.
   procedure Set_Vel_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setVelIdx";

   --* @brief Get the current velocity index.
   --* @param Self Pointer to the instance.
   --* @return The current velocity index.
   function Get_Vel_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getVelIdx";

   --* @brief Set the accumulated DV index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new accumulated DV index to set.
   procedure Set_Dv_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setDvIdx";

   --* @brief Get the current accumulated DV index.
   --* @param Self Pointer to the instance.
   --* @return The current accumulated DV index.
   function Get_Dv_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getDvIdx";

   --* @brief Set the sun index.
   --* @param Self Pointer to the instance.
   --* @param Idx  The new sun index to set.
   procedure Set_Sun_Idx
     (Self : Nav_Aggregate_Algorithm_Access;
      Idx  : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setSunIdx";

   --* @brief Get the current sun index.
   --* @param Self Pointer to the instance.
   --* @return The current sun index.
   function Get_Sun_Idx
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getSunIdx";

   --* @brief Set the attitude message count.
   --* @param Self      Pointer to the instance.
   --* @param Msg_Count The new attitude message count to set.
   procedure Set_Att_Msg_Count
     (Self      : Nav_Aggregate_Algorithm_Access;
      Msg_Count : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setAttMsgCount";

   --* @brief Get the current attitude message count.
   --* @param Self Pointer to the instance.
   --* @return The current attitude message count.
   function Get_Att_Msg_Count
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getAttMsgCount";

   --* @brief Set the translational message count.
   --* @param Self      Pointer to the instance.
   --* @param Msg_Count The new translational message count to set.
   procedure Set_Trans_Msg_Count
     (Self      : Nav_Aggregate_Algorithm_Access;
      Msg_Count : Unsigned_32)
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_setTransMsgCount";

   --* @brief Get the current translational message count.
   --* @param Self Pointer to the instance.
   --* @return The current translational message count.
   function Get_Trans_Msg_Count
     (Self : Nav_Aggregate_Algorithm_Access)
     return Unsigned_32
     with Import       => True,
          Convention   => C,
          External_Name => "NavAggregateAlgorithm_getTransMsgCount";

private

   -- Private representation: opaque null record
   type Nav_Aggregate_Algorithm is null record;

end Nav_Aggregate_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
