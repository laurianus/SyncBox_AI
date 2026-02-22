/*
 * Id:        System1Data.c
 *
 * Function:  VS System Data Source File.
 *
 * Generated: Sat Jan 04 13:25:23 2020
 *
 * Coder 6, 3, 2, 1841
 * 
 * This is an automatically generated file. It will be overwritten by the Coder.
 * 
 * DO NOT EDIT THE FILE!
 */


#include "System1SEMLibB.h"


#if (VS_CODER_GUID != 0X000567a93L)
#error The generated file does not match the SEMTypes.h header file.
#endif


#include "System1Data.h"


#include "System1Action.h"


#include <stdarg.h>


/*
 * VS System External Variable Definitions.
 */
VS_BOOL Can_be_Slave;

VS_UINT8 ET_Active;

VS_BOOL Ext1Status;

VS_UINT8 Fired_cues_NM;

VS_UINT8 IS_F1_CON;

VS_BOOL RTimeToLearnV;

VS_INT RepeatCount;

VS_INT cue_skip;

VS_INT device_Kind;

VS_INT device_Status;

VS_BOOL isDisEn;

VS_BOOL isEraseR;

VS_INT isFiring;

VS_INT isGathering;

VS_INT isGetMatrix;

VS_BOOL isISsent;

VS_INT isJoinEnable;

VS_BOOL isLearnR;

VS_BOOL isLearnValid;

VS_BOOL isMSGTimerValid;

VS_UINT8 isMSGTimerValidX;

VS_INT isMonitorPingValid;

VS_INT isPCMasterTask;

VS_INT isPeriodicGetMatrixTimerValid;

VS_BOOL isProgramming;

VS_INT isReporting;

VS_INT isResetTimerValid;

VS_INT isScheduleTimerValid;

VS_INT isSelfTested;

VS_INT isSelfTimerValid;

VS_UINT8 isSeq;

VS_UINT8 isSetaddress;

VS_INT isSeveralLines;

VS_INT isTimeMatrixUpdate;

VS_BOOL isTimer1Valid;

VS_BOOL isTimer2Valid;

VS_BOOL isTimer3Valid;

VS_INT isTimer4Valid;

VS_BOOL isTimer5Valid;

VS_INT isTimer6Valid;

VS_INT isTimer7Valid;

VS_INT isTimer8Valid;

VS_UINT8 is_F1_ARM;

VS_INT missedPingCount;

VS_BOOL was_LTC_Started;


/*
 * Guard Expression Functions.
 */
static VS_BOOL System1VSGuard_0 (VS_VOID)
{
  return (VS_BOOL)(isPCMasterTask == 0);
}
static VS_BOOL System1VSGuard_1 (VS_VOID)
{
  return (VS_BOOL)(isPCMasterTask == 0);
}
static VS_BOOL System1VSGuard_2 (VS_VOID)
{
  return (VS_BOOL)(isPCMasterTask == 0);
}


/*
 * Guard Expression Pointer Table.
 */
VS_GUARDEXPR_TYPE const System1VSGuard[3] = 
{
  System1VSGuard_0,
  System1VSGuard_1,
  System1VSGuard_2
};


/*
 * Action Expression Functions.
 */
VS_VOID System1VSAction_92 (VS_VOID)
{
  loadingData();
}
VS_VOID System1VSAction_147 (VS_VOID)
{
  isMSGTimerValid = 1;
}
VS_VOID System1VSAction_148 (VS_VOID)
{
  isPeriodicGetMatrixTimerValid = 1;
}
VS_VOID System1VSAction_149 (VS_VOID)
{
  isTimer2Valid = 1;
}
VS_VOID System1VSAction_150 (VS_VOID)
{
  isTimer1Valid = 1;
}
VS_VOID System1VSAction_151 (VS_VOID)
{
  IS_F1_CON = 1;
}
VS_VOID System1VSAction_152 (VS_VOID)
{
  isTimer1Valid = 0;
}
VS_VOID System1VSAction_153 (VS_VOID)
{
  isSelfTimerValid = 1;
}
VS_VOID System1VSAction_154 (VS_VOID)
{
  isPeriodicGetMatrixTimerValid = 0;
}
VS_VOID System1VSAction_155 (VS_VOID)
{
  is_F1_ARM = 1;
}
VS_VOID System1VSAction_156 (VS_VOID)
{
  was_LTC_Started = 1;
}
VS_VOID System1VSAction_157 (VS_VOID)
{
  isGetMatrix = 1;
}
VS_VOID System1VSAction_158 (VS_VOID)
{
  isTimer5Valid = 1;
}
VS_VOID System1VSAction_159 (VS_VOID)
{
  isSeq = 0;
}
VS_VOID System1VSAction_160 (VS_VOID)
{
  isGetMatrix = 0;
}
VS_VOID System1VSAction_161 (VS_VOID)
{
  isTimer5Valid = 0;
}
VS_VOID System1VSAction_162 (VS_VOID)
{
  isFiring = 0;
}
VS_VOID System1VSAction_163 (VS_VOID)
{
  isSeveralLines = 1;
}
VS_VOID System1VSAction_164 (VS_VOID)
{
  isFiring = 1;
}
VS_VOID System1VSAction_165 (VS_VOID)
{
  ++Fired_cues_NM;
}
VS_VOID System1VSAction_166 (VS_VOID)
{
  isSeveralLines = 0;
}
VS_VOID System1VSAction_167 (VS_VOID)
{
  was_LTC_Started = 0;
}
VS_VOID System1VSAction_168 (VS_VOID)
{
  Fired_cues_NM = 0;
}
VS_VOID System1VSAction_169 (VS_VOID)
{
  RTimeToLearnV = 0;
}
VS_VOID System1VSAction_170 (VS_VOID)
{
  device_Kind = Unknown_Device;
}
VS_VOID System1VSAction_171 (VS_VOID)
{
  device_Status = Status_Init;
}
VS_VOID System1VSAction_172 (VS_VOID)
{
  isPCMasterTask = 0;
}
VS_VOID System1VSAction_173 (VS_VOID)
{
  isTimer2Valid = 0;
}
VS_VOID System1VSAction_174 (VS_VOID)
{
  isTimer3Valid = 0;
}
VS_VOID System1VSAction_175 (VS_VOID)
{
  isTimer4Valid = 0;
}
VS_VOID System1VSAction_176 (VS_VOID)
{
  isTimer7Valid = 0;
}
VS_VOID System1VSAction_177 (VS_VOID)
{
  isMonitorPingValid = 0;
}
VS_VOID System1VSAction_178 (VS_VOID)
{
  missedPingCount = 0;
}
VS_VOID System1VSAction_179 (VS_VOID)
{
  isResetTimerValid = 1;
}
VS_VOID System1VSAction_180 (VS_VOID)
{
  isResetTimerValid = 0;
}
VS_VOID System1VSAction_181 (VS_VOID)
{
  isSetaddress = 0;
}
VS_VOID System1VSAction_182 (VS_VOID)
{
  isSetaddress = 1;
}
VS_VOID System1VSAction_183 (VS_VOID)
{
  Can_be_Slave = 1;
}
VS_VOID System1VSAction_184 (VS_VOID)
{
  Ext1Status = 0;
}
VS_VOID System1VSAction_185 (VS_VOID)
{
  isSelfTimerValid = 0;
}
VS_VOID System1VSAction_186 (VS_VOID)
{
  cue_skip = 0;
}
VS_VOID System1VSAction_187 (VS_VOID)
{
  Can_be_Slave = 0;
}
VS_VOID System1VSAction_188 (VS_VOID)
{
  isSelfTested = 0;
}
VS_VOID System1VSAction_189 (VS_VOID)
{
  isISsent = 0;
}
VS_VOID System1VSAction_190 (VS_VOID)
{
  isDisEn = 0;
}
VS_VOID System1VSAction_191 (VS_VOID)
{
  isSelfTested = 1;
}
VS_VOID System1VSAction_192 (VS_VOID)
{
  isScheduleTimerValid = 0;
}
VS_VOID System1VSAction_193 (VS_VOID)
{
  device_Status = Status_Error;
}
VS_VOID System1VSAction_194 (VS_VOID)
{
  device_Kind = Master_Device;
}
VS_VOID System1VSAction_195 (VS_VOID)
{
  device_Status = Status_Create;
}
VS_VOID System1VSAction_196 (VS_VOID)
{
  isJoinEnable = 1;
}
VS_VOID System1VSAction_197 (VS_VOID)
{
  isJoinEnable = 0;
}
VS_VOID System1VSAction_198 (VS_VOID)
{
  isTimer4Valid = 1;
}
VS_VOID System1VSAction_199 (VS_VOID)
{
  isReporting = 1;
}
VS_VOID System1VSAction_200 (VS_VOID)
{
  isReporting = 0;
}
VS_VOID System1VSAction_201 (VS_VOID)
{
  is_F1_ARM = 0;
}
VS_VOID System1VSAction_202 (VS_VOID)
{
  device_Status = Status_Idle;
}
VS_VOID System1VSAction_203 (VS_VOID)
{
  isTimer7Valid = 1;
}
VS_VOID System1VSAction_204 (VS_VOID)
{
  isGathering = 0;
}
VS_VOID System1VSAction_205 (VS_VOID)
{
  isPCMasterTask = 1;
}
VS_VOID System1VSAction_206 (VS_VOID)
{
  isTimer6Valid = 1;
}
VS_VOID System1VSAction_207 (VS_VOID)
{
  device_Status = Status_PowerEnable;
}
VS_VOID System1VSAction_208 (VS_VOID)
{
  isPCMasterTask = 2;
}
VS_VOID System1VSAction_209 (VS_VOID)
{
  isTimer3Valid = 1;
}
VS_VOID System1VSAction_210 (VS_VOID)
{
  device_Status = Status_Play;
}
VS_VOID System1VSAction_211 (VS_VOID)
{
  isTimeMatrixUpdate = 1;
}
VS_VOID System1VSAction_212 (VS_VOID)
{
  device_Status = Status_Pause;
}
VS_VOID System1VSAction_213 (VS_VOID)
{
  isMSGTimerValidX = 0;
}
VS_VOID System1VSAction_214 (VS_VOID)
{
  isProgramming = 0;
}
VS_VOID System1VSAction_215 (VS_VOID)
{
  device_Kind = Slave_Device;
}
VS_VOID System1VSAction_216 (VS_VOID)
{
  isMonitorPingValid = 1;
}
VS_VOID System1VSAction_217 (VS_VOID)
{
  isTimeMatrixUpdate = 0;
}


/*
 * Action Expression Pointer Table.
 */
VS_ACTIONEXPR_TYPE const System1VSAction[218] = 
{
  ARM_OFF,
  ARM_ON,
  Bat_Voltage,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  DIS_PWR_ctrl,
  DIS_R433_X,
  DMX_init,
  EN_PWR_ctrl,
  InitDevice2,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  Load_Config,
  MTX_0FB_ON,
  MTX_FB_OFF,
  MTX_FIRE_OFF,
  MTX_FIRE_ON,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  MTX_TST_OFF,
  MTX_TST_ON,
  MTX_xFB_ON,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  Master_Set,
  Pause_State,
  Play_State,
  Prog_Finish,
  Read_MTX_0V,
  Reset_Start_Prg,
  Rst_Crew_ctrl,
  Rst_ManualTime,
  Rst_Pause_State,
  Rst_Play_State,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  Rst_cur_ch_ASD,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  Set_DMX_R,
  Set_Pause_Firing,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  Start_BUZ,
  Stop_BUZ,
  TC_init,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  activeColumn,
  activeLine,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  checkProgram,
  check_ST,
  clearActivedTimeMartix,
  clearGetElements,
  clearJoinStatus,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  clearPWEN,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  clearReportSlaves,
  clearSlavePingedX,
  clearStatusMatrixA,
  clearStatusMatrixB,
  clearSyncCounter,
  clearTimeMatrix,
  deactiveColumn,
  deactiveLine,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  incMissedPingCount,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  initMissedPings,
  initMonitorPingTimer,
  initPeriodicGetMatrixTimer,
  initResetTimer,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  initSelfTimer_Idle,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  initSeqTimer,
  initSyncTimer,
  initTimer1,
  initTimer1_ID,
  initTimer2,
  initTimer2_10ms,
  initTimer2_1ms,
  initTimer3,
  initTimer4,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  initTimer5,
  initTimer5_10ms,
  initTimer5_1ms,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  initTimer5_5ms,
  initTimer6,
  initTimer7,
  initTimer7_10s,
  initTimer8,
  System1VSAction_92,
  pauseSysTimer,
  readMatrixAADC,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  readMatrixBADC,
  read_sPin,
  resetFiringTimers,
  resetSysTimer,
  reset_DMX_Buf,
  reset_SMPTEtime,
  resumeSysTimer,
  rst_Display_M,
  saveLines,
  save_flash_D,
  sendMSFinishProgramming,
  sendMSJoinBroadcast,
  sendMSManualActivation,
  sendMSPause,
  sendMSPingBC,
  sendMSPowerDisable,
  sendMSPowerEnable,
  sendMSProgramming,
  sendMSStart,
  sendMSStop,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  sendPCError,
  sendPCSlaveInterfaceStatus,
  sendPCSlaveMatrixA,
  sendPCSlaveMatrixB,
  sendPCStatusFinished,
  sendPCStatusInterface,
  sendPCStatusInterfaceSA,
  sendPCStatusMatrixA,
  sendPCStatusMatrixB,
  sendSMPingB,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  sendSM_PRG_Answer,
  setJoinStatus,
  setMaster,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  setPWEN,
  setProgrammingMS,
  setProgrammingPC,
  setSlave,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  setSlaveReported,
  setTOsalve,
  setTimer4,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  set_LastFire,
  startSysTimer,
  system_save_Timer,
  (VS_ACTIONEXPR_TYPE) NULL /* declared action function is never used */,
  vLightOn,
  System1VSAction_147,
  System1VSAction_148,
  System1VSAction_149,
  System1VSAction_150,
  System1VSAction_151,
  System1VSAction_152,
  System1VSAction_153,
  System1VSAction_154,
  System1VSAction_155,
  System1VSAction_156,
  System1VSAction_157,
  System1VSAction_158,
  System1VSAction_159,
  System1VSAction_160,
  System1VSAction_161,
  System1VSAction_162,
  System1VSAction_163,
  System1VSAction_164,
  System1VSAction_165,
  System1VSAction_166,
  System1VSAction_167,
  System1VSAction_168,
  System1VSAction_169,
  System1VSAction_170,
  System1VSAction_171,
  System1VSAction_172,
  System1VSAction_173,
  System1VSAction_174,
  System1VSAction_175,
  System1VSAction_176,
  System1VSAction_177,
  System1VSAction_178,
  System1VSAction_179,
  System1VSAction_180,
  System1VSAction_181,
  System1VSAction_182,
  System1VSAction_183,
  System1VSAction_184,
  System1VSAction_185,
  System1VSAction_186,
  System1VSAction_187,
  System1VSAction_188,
  System1VSAction_189,
  System1VSAction_190,
  System1VSAction_191,
  System1VSAction_192,
  System1VSAction_193,
  System1VSAction_194,
  System1VSAction_195,
  System1VSAction_196,
  System1VSAction_197,
  System1VSAction_198,
  System1VSAction_199,
  System1VSAction_200,
  System1VSAction_201,
  System1VSAction_202,
  System1VSAction_203,
  System1VSAction_204,
  System1VSAction_205,
  System1VSAction_206,
  System1VSAction_207,
  System1VSAction_208,
  System1VSAction_209,
  System1VSAction_210,
  System1VSAction_211,
  System1VSAction_212,
  System1VSAction_213,
  System1VSAction_214,
  System1VSAction_215,
  System1VSAction_216,
  System1VSAction_217
};
