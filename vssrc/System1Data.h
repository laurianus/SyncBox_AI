#ifndef _visualSTATE_SYSTEM1DATA_H
#define _visualSTATE_SYSTEM1DATA_H

/*
 * Id:        System1Data.h
 *
 * Function:  VS System Data Header File.
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
 * Event Identifier Definitions.
 */
#define SE_RESET                         0X000u  /*   0 */
#define ev_Arm                           0X001u  /*   1 */
#define ev_BB_Long_Press                 0X002u  /*   2 */
#define ev_BB_Short_Press                0X003u  /*   3 */
#define ev_Disarm                        0X004u  /*   4 */
#define ev_GB_Long_Press                 0X005u  /*   5 */
#define ev_GB_Short_Press                0X006u  /*   6 */
#define ev_GB_Short_PressX               0X007u  /*   7 */
#define ev_MS_Ping                       0X008u  /*   8 */
#define ev_MS_ToMaster                   0X009u  /*   9 */
#define ev_MS_ToTEST                     0X00Au  /*  10 */
#define ev_NetworkJoin                   0X00Bu  /*  11 */
#define ev_Pause                         0X00Cu  /*  12 */
#define ev_Resume                        0X00Du  /*  13 */
#define ev_SMPTEstart                    0X00Eu  /*  14 */
#define ev_SMPTEstartP                   0X00Fu  /*  15 */
#define ev_Start                         0X010u  /*  16 */
#define ev_Stop                          0X011u  /*  17 */
#define ev_SyncMsg                       0X012u  /*  18 */


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
extern VS_INT device_Kind;

extern VS_INT device_Status;


#endif /* ifndef _visualSTATE_SYSTEM1DATA_H */
