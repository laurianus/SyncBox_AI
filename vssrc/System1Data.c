/*
 * Id:        System1Data.c
 *
 * Function:  VS System Data Source File.
 *
 * Generated: Sat Jan 04 00:01:30 2020
 *
 * Coder 6, 3, 2, 1841
 * 
 * This is an automatically generated file. It will be overwritten by the Coder.
 * 
 * DO NOT EDIT THE FILE!
 */


#include "System1SEMLibB.h"


#if (VS_CODER_GUID != 0X01b841080L)
#error The generated file does not match the SEMTypes.h header file.
#endif


#include "System1Data.h"


#include "System1Action.h"


#include <stdarg.h>


/*
 * VS System External Variable Definitions.
 */
VS_INT device_Kind;

VS_INT device_Status;


/*
 * Action Expression Functions.
 */
VS_VOID System1VSAction_34 (VS_VOID)
{
  device_Status = Status_PowerEnable;
}
VS_VOID System1VSAction_35 (VS_VOID)
{
  device_Kind = Slave_Device;
}
VS_VOID System1VSAction_36 (VS_VOID)
{
  device_Status = Status_Idle;
}
VS_VOID System1VSAction_37 (VS_VOID)
{
  device_Status = Status_Play;
}
VS_VOID System1VSAction_38 (VS_VOID)
{
  device_Status = Status_Pause;
}
VS_VOID System1VSAction_39 (VS_VOID)
{
  device_Kind = Master_Device;
}


/*
 * Action Expression Pointer Table.
 */
VS_ACTIONEXPR_TYPE const System1VSAction[40] = 
{
  ARM_DMX,
  Audio_Pause,
  Audio_Play_Pause_Idle,
  Audio_Play_Resume,
  Audio_Stop,
  DELAY_STAT_MSG,
  DISABLE_PWR_BUTTON,
  Dev_Valid_Timer_Rst,
  Dev_Valid_Timer_Set,
  ENABLE_PWR_BUTTON,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  RST_ARM_DMX,
  RST_DEV_Time,
  RST_DMX_Timer,
  STOP_TIMECODE,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  TimeCode_Gen_start,
  TimeCode_Gen_stop,
  Vol_Down,
  Vol_Up,
  clearPWEN,
  reset_DMX_Buf,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  sendMSPause,
  sendMSPowerDisable,
  sendMSPowerEnable,
  sendMSStart,
  sendMSStop,
  sendSMStatusInterface,
  send_433_start_Master_msg,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  setPWEN,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  System1VSAction_34,
  System1VSAction_35,
  System1VSAction_36,
  System1VSAction_37,
  System1VSAction_38,
  System1VSAction_39
};
