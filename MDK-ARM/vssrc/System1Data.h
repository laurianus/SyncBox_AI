#ifndef _visualSTATE_SYSTEM1DATA_H
#define _visualSTATE_SYSTEM1DATA_H

/*
 * Id:        System1Data.h
 *
 * Function:  VS System Data Header File.
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
 * Event Identifier Definitions.
 */
#define SE_RESET                         0X000u  /*   0 */
#define ev_ActiveTimeMatrix              0X001u  /*   1 */
#define ev_AnotherElement                0X002u  /*   2 */
#define ev_AnotherReport                 0X003u  /*   3 */
#define ev_But_Disarm                    0X004u  /*   4 */
#define ev_But_ID_SET                    0X005u  /*   5 */
#define ev_EndElements                   0X006u  /*   6 */
#define ev_EndLine                       0X007u  /*   7 */
#define ev_EndReport                     0X008u  /*   8 */
#define ev_EndSlave                      0X009u  /*   9 */
#define ev_EraseR                        0X00Au  /*  10 */
#define ev_EraseRExit                    0X00Bu  /*  11 */
#define ev_ErrorPCMsgDecode              0X00Cu  /*  12 */
#define ev_Error_CANDecode               0X00Du  /*  13 */
#define ev_Error_CANInit                 0X00Eu  /*  14 */
#define ev_Error_CANRx                   0X00Fu  /*  15 */
#define ev_Error_CANTx                   0X010u  /*  16 */
#define ev_Error_USBDecode               0X011u  /*  17 */
#define ev_Error_USBInit                 0X012u  /*  18 */
#define ev_Error_USBReceive              0X013u  /*  19 */
#define ev_Error_USBSend                 0X014u  /*  20 */
#define ev_Error_WirelessDecode          0X015u  /*  21 */
#define ev_Error_WirelessInit            0X016u  /*  22 */
#define ev_External_trigger              0X017u  /*  23 */
#define ev_F1_ARM                        0X018u  /*  24 */
#define ev_F1_PLAY                       0X019u  /*  25 */
#define ev_F1_PWR_Dis                    0X01Au  /*  26 */
#define ev_F1_Ping                       0X01Bu  /*  27 */
#define ev_GPS_Start                     0X01Cu  /*  28 */
#define ev_GetStatusTimeOver             0X01Du  /*  29 */
#define ev_LearnR                        0X01Eu  /*  30 */
#define ev_LostPing                      0X01Fu  /*  31 */
#define ev_MS_BroadcastJoin              0X020u  /*  32 */
#define ev_MS_BroadcastJoinX             0X021u  /*  33 */
#define ev_MS_CANTerminal                0X022u  /*  34 */
#define ev_MS_ET_Arm                     0X023u  /*  35 */
#define ev_MS_FinishProgramming          0X024u  /*  36 */
#define ev_MS_InterfaceProgramming       0X025u  /*  37 */
#define ev_MS_ManualActivation           0X026u  /*  38 */
#define ev_MS_Pause                      0X027u  /*  39 */
#define ev_MS_Ping                       0X028u  /*  40 */
#define ev_MS_PowerDisable               0X029u  /*  41 */
#define ev_MS_PowerEnable                0X02Au  /*  42 */
#define ev_MS_Start                      0X02Bu  /*  43 */
#define ev_MS_Stop                       0X02Cu  /*  44 */
#define ev_MS_WirelessPower              0X02Du  /*  45 */
#define ev_M_S_Event                     0X02Eu  /*  46 */
#define ev_MasterMsg_Error               0X02Fu  /*  47 */
#define ev_Master_ARM                    0X030u  /*  48 */
#define ev_MessageSntT                   0X031u  /*  49 */
#define ev_MonitorPingOver               0X032u  /*  50 */
#define ev_NextLine                      0X033u  /*  51 */
#define ev_PCMsg_CANTerminal             0X034u  /*  52 */
#define ev_PCMsg_FinishProgramming       0X035u  /*  53 */
#define ev_PCMsg_FinishProgrammingSelf   0X036u  /*  54 */
#define ev_PCMsg_Init                    0X037u  /*  55 */
#define ev_PCMsg_InitDMX                 0X038u  /*  56 */
#define ev_PCMsg_JoinDisable             0X039u  /*  57 */
#define ev_PCMsg_JoinEnable              0X03Au  /*  58 */
#define ev_PCMsg_ManualActivation        0X03Bu  /*  59 */
#define ev_PCMsg_ManualActivationSelf    0X03Cu  /*  60 */
#define ev_PCMsg_Pause                   0X03Du  /*  61 */
#define ev_PCMsg_Ping                    0X03Eu  /*  62 */
#define ev_PCMsg_PowerDisable            0X03Fu  /*  63 */
#define ev_PCMsg_PowerEnable             0X040u  /*  64 */
#define ev_PCMsg_Programming             0X041u  /*  65 */
#define ev_PCMsg_ProgrammingSelf         0X042u  /*  66 */
#define ev_PCMsg_Start                   0X043u  /*  67 */
#define ev_PCMsg_Stop                    0X044u  /*  68 */
#define ev_PCMsg_WirelessPower           0X045u  /*  69 */
#define ev_PPIN_Active                   0X046u  /*  70 */
#define ev_PPIN_ActiveXXX                0X047u  /*  71 */
#define ev_PPIN_ActiveYYY                0X048u  /*  72 */
#define ev_PauseTimerS                   0X049u  /*  73 */
#define ev_PeriodicGetting               0X04Au  /*  74 */
#define ev_PlayFinish                    0X04Bu  /*  75 */
#define ev_SMPTEstart                    0X04Cu  /*  76 */
#define ev_SMPTEstartP                   0X04Du  /*  77 */
#define ev_SM_AB_Ping                    0X04Eu  /*  78 */
#define ev_SM_InterfaceStatus            0X04Fu  /*  79 */
#define ev_SM_InterfaceStatusPrg         0X050u  /*  80 */
#define ev_SM_InterfaceStatusX           0X051u  /*  81 */
#define ev_SM_StatusMatrixA              0X052u  /*  82 */
#define ev_SM_StatusMatrixB              0X053u  /*  83 */
#define ev_ST_Error                      0X054u  /*  84 */
#define ev_ST_Success                    0X055u  /*  85 */
#define ev_ST_SuccessNP                  0X056u  /*  86 */
#define ev_ScheduleTimeOver              0X057u  /*  87 */
#define ev_SelfPIN_Active                0X058u  /*  88 */
#define ev_SelfPIN_ActiveXXX             0X059u  /*  89 */
#define ev_SelfTimerOver                 0X05Au  /*  90 */
#define ev_SendSMPingB                   0X05Bu  /*  91 */
#define ev_Send_IS                       0X05Cu  /*  92 */
#define ev_Set_Master                    0X05Du  /*  93 */
#define ev_Step_mode                     0X05Eu  /*  94 */
#define ev_SyncMissed                    0X05Fu  /*  95 */
#define ev_SyncMsg                       0X060u  /*  96 */
#define ev_SyncTimer                     0X061u  /*  97 */
#define ev_Sync_msg_R                    0X062u  /*  98 */
#define ev_Timer1Over                    0X063u  /*  99 */
#define ev_Timer2Over                    0X064u  /* 100 */
#define ev_Timer3Over                    0X065u  /* 101 */
#define ev_Timer5Over                    0X066u  /* 102 */
#define ev_Timer6Over                    0X067u  /* 103 */
#define ev_Timer7Over                    0X068u  /* 104 */
#define ev_Timer8Over                    0X069u  /* 105 */
#define ev_USBConnected                  0X06Au  /* 106 */
#define ev_USBDisconnected               0X06Bu  /* 107 */
#define ev_wasFired                      0X06Cu  /* 108 */


/*
 * Constants.
 */
#define Master_Device                    ((VS_INT) 2)
#define RepeatLimit                      ((VS_INT) 1)
#define Self_Device                      ((VS_INT) 3)
#define Slave_Device                     ((VS_INT) 1)
#define Status_Create                    ((VS_INT) 1)
#define Status_Error                     ((VS_INT) 9)
#define Status_Idle                      ((VS_INT) 2)
#define Status_Init                      ((VS_INT) 0)
#define Status_Manual                    ((VS_INT) 6)
#define Status_Pause                     ((VS_INT) 5)
#define Status_Play                      ((VS_INT) 4)
#define Status_PowerEnable               ((VS_INT) 3)
#define Unknown_Device                   ((VS_INT) 0)


/*
 * VS System External Variable Declarations.
 */
extern VS_BOOL Can_be_Slave;

extern VS_UINT8 ET_Active;

extern VS_BOOL Ext1Status;

extern VS_UINT8 Fired_cues_NM;

extern VS_UINT8 IS_F1_CON;

extern VS_BOOL RTimeToLearnV;

extern VS_INT RepeatCount;

extern VS_INT cue_skip;

extern VS_INT device_Kind;

extern VS_INT device_Status;

extern VS_BOOL isDisEn;

extern VS_BOOL isEraseR;

extern VS_INT isFiring;

extern VS_INT isGathering;

extern VS_INT isGetMatrix;

extern VS_BOOL isISsent;

extern VS_INT isJoinEnable;

extern VS_BOOL isLearnR;

extern VS_BOOL isLearnValid;

extern VS_BOOL isMSGTimerValid;

extern VS_UINT8 isMSGTimerValidX;

extern VS_INT isMonitorPingValid;

extern VS_INT isPCMasterTask;

extern VS_INT isPeriodicGetMatrixTimerValid;

extern VS_BOOL isProgramming;

extern VS_INT isReporting;

extern VS_INT isResetTimerValid;

extern VS_INT isScheduleTimerValid;

extern VS_INT isSelfTested;

extern VS_INT isSelfTimerValid;

extern VS_UINT8 isSeq;

extern VS_UINT8 isSetaddress;

extern VS_INT isSeveralLines;

extern VS_INT isTimeMatrixUpdate;

extern VS_BOOL isTimer1Valid;

extern VS_BOOL isTimer2Valid;

extern VS_BOOL isTimer3Valid;

extern VS_INT isTimer4Valid;

extern VS_BOOL isTimer5Valid;

extern VS_INT isTimer6Valid;

extern VS_INT isTimer7Valid;

extern VS_INT isTimer8Valid;

extern VS_UINT8 is_F1_ARM;

extern VS_INT missedPingCount;

extern VS_BOOL was_LTC_Started;


#endif /* ifndef _visualSTATE_SYSTEM1DATA_H */
