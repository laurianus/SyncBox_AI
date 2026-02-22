#ifndef _visualSTATE_SYSTEM1ACTION_H
#define _visualSTATE_SYSTEM1ACTION_H

/*
 * Id:        System1Action.h
 *
 * Function:  VS System Action Expression Pointer Table Header File.
 *
 * Generated: Sat Jan 04 00:01:30 2020
 *
 * Coder 6, 3, 2, 1841
 * 
 * This is an automatically generated file. It will be overwritten by the Coder.
 * 
 * DO NOT EDIT THE FILE!
 */


#include "System1SEMBDef.h"


#if (VS_CODER_GUID != 0X01b841080L)
#error The generated file does not match the SEMTypes.h header file.
#endif


/*
 * Action Function Prototypes.
 */
extern VS_VOID ARM_DMX (VS_VOID);
extern VS_VOID Audio_Pause (VS_VOID);
extern VS_VOID Audio_Play_Pause_Idle (VS_VOID);
extern VS_VOID Audio_Play_Resume (VS_VOID);
extern VS_VOID Audio_Stop (VS_VOID);
extern VS_VOID DELAY_STAT_MSG (VS_VOID);
extern VS_VOID DISABLE_PWR_BUTTON (VS_VOID);
extern VS_VOID Dev_Valid_Timer_Rst (VS_VOID);
extern VS_VOID Dev_Valid_Timer_Set (VS_VOID);
extern VS_VOID ENABLE_PWR_BUTTON (VS_VOID);
extern VS_VOID RESET_PING_TIMER (VS_VOID);
extern VS_VOID RST_ARM_DMX (VS_VOID);
extern VS_VOID RST_DEV_Time (VS_VOID);
extern VS_VOID RST_DMX_Timer (VS_VOID);
extern VS_VOID STOP_TIMECODE (VS_VOID);
extern VS_VOID Set_crit_errA (VS_VOID);
extern VS_VOID State_update (VS_VOID);
extern VS_VOID TimeCode_Gen_start (VS_VOID);
extern VS_VOID TimeCode_Gen_stop (VS_VOID);
extern VS_VOID Vol_Down (VS_VOID);
extern VS_VOID Vol_Up (VS_VOID);
extern VS_VOID clearPWEN (VS_VOID);
extern VS_VOID reset_DMX_Buf (VS_VOID);
extern VS_VOID rst_Last_Fired (VS_VOID);
extern VS_VOID sendMSPause (VS_VOID);
extern VS_VOID sendMSPowerDisable (VS_VOID);
extern VS_VOID sendMSPowerEnable (VS_VOID);
extern VS_VOID sendMSStart (VS_VOID);
extern VS_VOID sendMSStop (VS_VOID);
extern VS_VOID sendSMStatusInterface (VS_VOID);
extern VS_VOID send_433_start_Master_msg (VS_VOID);
extern VS_VOID setJoinStatus (VS_VOID);
extern VS_VOID setPWEN (VS_VOID);
extern VS_VOID set_refresh_lcdAC (VS_VOID);


/*
 * Action Expression Function Prototypes.
 */
extern VS_VOID System1VSAction_34 (VS_VOID);

extern VS_VOID System1VSAction_35 (VS_VOID);

extern VS_VOID System1VSAction_36 (VS_VOID);

extern VS_VOID System1VSAction_37 (VS_VOID);

extern VS_VOID System1VSAction_38 (VS_VOID);

extern VS_VOID System1VSAction_39 (VS_VOID);


/*
 * Action Expression Pointer Table.
 */
extern VS_ACTIONEXPR_TYPE const System1VSAction[40];


#endif /* ifndef _visualSTATE_SYSTEM1ACTION_H */
