#ifndef _visualSTATE_SYSTEM1ACTION_H
#define _visualSTATE_SYSTEM1ACTION_H

/*
 * Id:        System1Action.h
 *
 * Function:  VS System Action Expression Pointer Table Header File.
 *
 * Generated: Sat Jan 04 13:25:23 2020
 *
 * Coder 6, 3, 2, 1841
 * 
 * This is an automatically generated file. It will be overwritten by the Coder.
 * 
 * DO NOT EDIT THE FILE!
 */


#include "System1SEMBDef.h"


#if (VS_CODER_GUID != 0X000567a93L)
#error The generated file does not match the SEMTypes.h header file.
#endif


/*
 * Action Function Prototypes.
 */
extern VS_VOID ARM_OFF (VS_VOID);
extern VS_VOID ARM_ON (VS_VOID);
extern VS_VOID Bat_Voltage (VS_VOID);
extern VS_VOID COL_TST (VS_VOID);
extern VS_VOID DIS_PWR_ctrl (VS_VOID);
extern VS_VOID DIS_R433_X (VS_VOID);
extern VS_VOID DMX_init (VS_VOID);
extern VS_VOID EN_PWR_ctrl (VS_VOID);
extern VS_VOID InitDevice2 (VS_VOID);
extern VS_VOID Line_TST (VS_VOID);
extern VS_VOID Load_Config (VS_VOID);
extern VS_VOID MTX_0FB_ON (VS_VOID);
extern VS_VOID MTX_FB_OFF (VS_VOID);
extern VS_VOID MTX_FIRE_OFF (VS_VOID);
extern VS_VOID MTX_FIRE_ON (VS_VOID);
extern VS_VOID MTX_TST (VS_VOID);
extern VS_VOID MTX_TST_OFF (VS_VOID);
extern VS_VOID MTX_TST_ON (VS_VOID);
extern VS_VOID MTX_xFB_ON (VS_VOID);
extern VS_VOID Manual_Activation_Event (VS_VOID);
extern VS_VOID Master_Set (VS_VOID);
extern VS_VOID Pause_State (VS_VOID);
extern VS_VOID Play_State (VS_VOID);
extern VS_VOID Prog_Finish (VS_VOID);
extern VS_VOID Read_MTX_0V (VS_VOID);
extern VS_VOID Reset_Start_Prg (VS_VOID);
extern VS_VOID Rst_Crew_ctrl (VS_VOID);
extern VS_VOID Rst_ManualTime (VS_VOID);
extern VS_VOID Rst_Pause_State (VS_VOID);
extern VS_VOID Rst_Play_State (VS_VOID);
extern VS_VOID Rst_Selected_Column (VS_VOID);
extern VS_VOID Rst_Selected_Line (VS_VOID);
extern VS_VOID Rst_cur_ch_ASD (VS_VOID);
extern VS_VOID SetTimer2 (VS_VOID);
extern VS_VOID Set_DMX_R (VS_VOID);
extern VS_VOID Set_Pause_Firing (VS_VOID);
extern VS_VOID Set_addressx (VS_VOID);
extern VS_VOID Start_BUZ (VS_VOID);
extern VS_VOID Stop_BUZ (VS_VOID);
extern VS_VOID TC_init (VS_VOID);
extern VS_VOID USB_VOLTAGE_TEST (VS_VOID);
extern VS_VOID VI_BAT_TST (VS_VOID);
extern VS_VOID activeColumn (VS_VOID);
extern VS_VOID activeLine (VS_VOID);
extern VS_VOID addSlaveToList (VS_VOID);
extern VS_VOID checkProgram (VS_VOID);
extern VS_VOID check_ST (VS_VOID);
extern VS_VOID clearActivedTimeMartix (VS_VOID);
extern VS_VOID clearGetElements (VS_VOID);
extern VS_VOID clearJoinStatus (VS_VOID);
extern VS_VOID clearMissedCount (VS_VOID);
extern VS_VOID clearPWEN (VS_VOID);
extern VS_VOID clearPingSlaves (VS_VOID);
extern VS_VOID clearReportSlaves (VS_VOID);
extern VS_VOID clearSlavePingedX (VS_VOID);
extern VS_VOID clearStatusMatrixA (VS_VOID);
extern VS_VOID clearStatusMatrixB (VS_VOID);
extern VS_VOID clearSyncCounter (VS_VOID);
extern VS_VOID clearTimeMatrix (VS_VOID);
extern VS_VOID deactiveColumn (VS_VOID);
extern VS_VOID deactiveLine (VS_VOID);
extern VS_VOID incMissedCount (VS_VOID);
extern VS_VOID incMissedCountFake (VS_VOID);
extern VS_VOID incMissedPingCount (VS_VOID);
extern VS_VOID incSyncMissedCount (VS_VOID);
extern VS_VOID initMissedPings (VS_VOID);
extern VS_VOID initMonitorPingTimer (VS_VOID);
extern VS_VOID initPeriodicGetMatrixTimer (VS_VOID);
extern VS_VOID initResetTimer (VS_VOID);
extern VS_VOID initScheduleTimer (VS_VOID);
extern VS_VOID initSelfTimer_Idle (VS_VOID);
extern VS_VOID initSelfTimer_PowerEnable (VS_VOID);
extern VS_VOID initSeqTimer (VS_VOID);
extern VS_VOID initSyncTimer (VS_VOID);
extern VS_VOID initTimer1 (VS_VOID);
extern VS_VOID initTimer1_ID (VS_VOID);
extern VS_VOID initTimer2 (VS_VOID);
extern VS_VOID initTimer2_10ms (VS_VOID);
extern VS_VOID initTimer2_1ms (VS_VOID);
extern VS_VOID initTimer3 (VS_VOID);
extern VS_VOID initTimer4 (VS_VOID);
extern VS_VOID initTimer4_1000ms (VS_VOID);
extern VS_VOID initTimer4_200ms (VS_VOID);
extern VS_VOID initTimer5 (VS_VOID);
extern VS_VOID initTimer5_10ms (VS_VOID);
extern VS_VOID initTimer5_1ms (VS_VOID);
extern VS_VOID initTimer5_3ms (VS_VOID);
extern VS_VOID initTimer5_5ms (VS_VOID);
extern VS_VOID initTimer6 (VS_VOID);
extern VS_VOID initTimer7 (VS_VOID);
extern VS_VOID initTimer7_10s (VS_VOID);
extern VS_VOID initTimer8 (VS_VOID);
extern VS_INT loadingData (VS_VOID);
extern VS_VOID pauseSysTimer (VS_VOID);
extern VS_VOID readMatrixAADC (VS_VOID);
extern VS_VOID readMatrixAxADC (VS_VOID);
extern VS_VOID readMatrixBADC (VS_VOID);
extern VS_VOID read_sPin (VS_VOID);
extern VS_VOID resetFiringTimers (VS_VOID);
extern VS_VOID resetSysTimer (VS_VOID);
extern VS_VOID reset_DMX_Buf (VS_VOID);
extern VS_VOID reset_SMPTEtime (VS_VOID);
extern VS_VOID resumeSysTimer (VS_VOID);
extern VS_VOID rst_Display_M (VS_VOID);
extern VS_VOID saveLines (VS_VOID);
extern VS_VOID save_flash_D (VS_VOID);
extern VS_VOID sendMSFinishProgramming (VS_VOID);
extern VS_VOID sendMSJoinBroadcast (VS_VOID);
extern VS_VOID sendMSManualActivation (VS_VOID);
extern VS_VOID sendMSPause (VS_VOID);
extern VS_VOID sendMSPingBC (VS_VOID);
extern VS_VOID sendMSPowerDisable (VS_VOID);
extern VS_VOID sendMSPowerEnable (VS_VOID);
extern VS_VOID sendMSProgramming (VS_VOID);
extern VS_VOID sendMSStart (VS_VOID);
extern VS_VOID sendMSStop (VS_VOID);
extern VS_VOID sendMSSync (VS_VOID);
extern VS_VOID sendPCError (VS_VOID);
extern VS_VOID sendPCSlaveInterfaceStatus (VS_VOID);
extern VS_VOID sendPCSlaveMatrixA (VS_VOID);
extern VS_VOID sendPCSlaveMatrixB (VS_VOID);
extern VS_VOID sendPCStatusFinished (VS_VOID);
extern VS_VOID sendPCStatusInterface (VS_VOID);
extern VS_VOID sendPCStatusInterfaceSA (VS_VOID);
extern VS_VOID sendPCStatusMatrixA (VS_VOID);
extern VS_VOID sendPCStatusMatrixB (VS_VOID);
extern VS_VOID sendSMPingB (VS_VOID);
extern VS_VOID sendSMPingBP (VS_VOID);
extern VS_VOID sendSMStatusInterface (VS_VOID);
extern VS_VOID sendSM_PRG_Answer (VS_VOID);
extern VS_VOID setJoinStatus (VS_VOID);
extern VS_VOID setMaster (VS_VOID);
extern VS_VOID setPCMessageTask (VS_VOID);
extern VS_VOID setPWEN (VS_VOID);
extern VS_VOID setProgrammingMS (VS_VOID);
extern VS_VOID setProgrammingPC (VS_VOID);
extern VS_VOID setSlave (VS_VOID);
extern VS_VOID setSlavePinged (VS_VOID);
extern VS_VOID setSlaveReported (VS_VOID);
extern VS_VOID setTOsalve (VS_VOID);
extern VS_VOID setTimer4 (VS_VOID);
extern VS_VOID set_Fire_Type (VS_VOID);
extern VS_VOID set_LastFire (VS_VOID);
extern VS_VOID startSysTimer (VS_VOID);
extern VS_VOID system_save_Timer (VS_VOID);
extern VS_VOID vLightOff (VS_VOID);
extern VS_VOID vLightOn (VS_VOID);


/*
 * Action Expression Function Prototypes.
 */
extern VS_VOID System1VSAction_92 (VS_VOID);

extern VS_VOID System1VSAction_147 (VS_VOID);

extern VS_VOID System1VSAction_148 (VS_VOID);

extern VS_VOID System1VSAction_149 (VS_VOID);

extern VS_VOID System1VSAction_150 (VS_VOID);

extern VS_VOID System1VSAction_151 (VS_VOID);

extern VS_VOID System1VSAction_152 (VS_VOID);

extern VS_VOID System1VSAction_153 (VS_VOID);

extern VS_VOID System1VSAction_154 (VS_VOID);

extern VS_VOID System1VSAction_155 (VS_VOID);

extern VS_VOID System1VSAction_156 (VS_VOID);

extern VS_VOID System1VSAction_157 (VS_VOID);

extern VS_VOID System1VSAction_158 (VS_VOID);

extern VS_VOID System1VSAction_159 (VS_VOID);

extern VS_VOID System1VSAction_160 (VS_VOID);

extern VS_VOID System1VSAction_161 (VS_VOID);

extern VS_VOID System1VSAction_162 (VS_VOID);

extern VS_VOID System1VSAction_163 (VS_VOID);

extern VS_VOID System1VSAction_164 (VS_VOID);

extern VS_VOID System1VSAction_165 (VS_VOID);

extern VS_VOID System1VSAction_166 (VS_VOID);

extern VS_VOID System1VSAction_167 (VS_VOID);

extern VS_VOID System1VSAction_168 (VS_VOID);

extern VS_VOID System1VSAction_169 (VS_VOID);

extern VS_VOID System1VSAction_170 (VS_VOID);

extern VS_VOID System1VSAction_171 (VS_VOID);

extern VS_VOID System1VSAction_172 (VS_VOID);

extern VS_VOID System1VSAction_173 (VS_VOID);

extern VS_VOID System1VSAction_174 (VS_VOID);

extern VS_VOID System1VSAction_175 (VS_VOID);

extern VS_VOID System1VSAction_176 (VS_VOID);

extern VS_VOID System1VSAction_177 (VS_VOID);

extern VS_VOID System1VSAction_178 (VS_VOID);

extern VS_VOID System1VSAction_179 (VS_VOID);

extern VS_VOID System1VSAction_180 (VS_VOID);

extern VS_VOID System1VSAction_181 (VS_VOID);

extern VS_VOID System1VSAction_182 (VS_VOID);

extern VS_VOID System1VSAction_183 (VS_VOID);

extern VS_VOID System1VSAction_184 (VS_VOID);

extern VS_VOID System1VSAction_185 (VS_VOID);

extern VS_VOID System1VSAction_186 (VS_VOID);

extern VS_VOID System1VSAction_187 (VS_VOID);

extern VS_VOID System1VSAction_188 (VS_VOID);

extern VS_VOID System1VSAction_189 (VS_VOID);

extern VS_VOID System1VSAction_190 (VS_VOID);

extern VS_VOID System1VSAction_191 (VS_VOID);

extern VS_VOID System1VSAction_192 (VS_VOID);

extern VS_VOID System1VSAction_193 (VS_VOID);

extern VS_VOID System1VSAction_194 (VS_VOID);

extern VS_VOID System1VSAction_195 (VS_VOID);

extern VS_VOID System1VSAction_196 (VS_VOID);

extern VS_VOID System1VSAction_197 (VS_VOID);

extern VS_VOID System1VSAction_198 (VS_VOID);

extern VS_VOID System1VSAction_199 (VS_VOID);

extern VS_VOID System1VSAction_200 (VS_VOID);

extern VS_VOID System1VSAction_201 (VS_VOID);

extern VS_VOID System1VSAction_202 (VS_VOID);

extern VS_VOID System1VSAction_203 (VS_VOID);

extern VS_VOID System1VSAction_204 (VS_VOID);

extern VS_VOID System1VSAction_205 (VS_VOID);

extern VS_VOID System1VSAction_206 (VS_VOID);

extern VS_VOID System1VSAction_207 (VS_VOID);

extern VS_VOID System1VSAction_208 (VS_VOID);

extern VS_VOID System1VSAction_209 (VS_VOID);

extern VS_VOID System1VSAction_210 (VS_VOID);

extern VS_VOID System1VSAction_211 (VS_VOID);

extern VS_VOID System1VSAction_212 (VS_VOID);

extern VS_VOID System1VSAction_213 (VS_VOID);

extern VS_VOID System1VSAction_214 (VS_VOID);

extern VS_VOID System1VSAction_215 (VS_VOID);

extern VS_VOID System1VSAction_216 (VS_VOID);

extern VS_VOID System1VSAction_217 (VS_VOID);


/*
 * Action Expression Pointer Table.
 */
extern VS_ACTIONEXPR_TYPE const System1VSAction[218];


#endif /* ifndef _visualSTATE_SYSTEM1ACTION_H */
