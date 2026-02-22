#ifndef _visualSTATE_SYSTEM1SEMBDEF_H
#define _visualSTATE_SYSTEM1SEMBDEF_H

/*
 * Id:        System1SEMBDef.h
 *
 * Function:  SEM Defines Header File.
 *
 * Generated: Sat Jan 04 00:01:30 2020
 *
 * Coder 6, 3, 2, 1841
 * 
 * This is an automatically generated file. It will be overwritten by the Coder.
 * 
 * DO NOT EDIT THE FILE!
 */


#include "System1SEMTypes.h"


#if (VS_CODER_GUID != 0X01b841080L)
#error The generated file does not match the SEMTypes.h header file.
#endif


/*
 * Number of Identifiers.
 */
#define VS_NOF_ACTION_EXPRESSIONS        0X028u  /*  40 */
#define VS_NOF_ACTION_FUNCTIONS          0X022u  /*  34 */
#define VS_NOF_EVENT_GROUPS              0X000u  /*   0 */
#define VS_NOF_EVENTS                    0X013u  /*  19 */
#define VS_NOF_EXTERNAL_VARIABLES        0X002u  /*   2 */
#define VS_NOF_GUARD_EXPRESSIONS         0X000u  /*   0 */
#define VS_NOF_INSTANCES                 0X001u  /*   1 */
#define VS_NOF_INTERNAL_VARIABLES        0X000u  /*   0 */
#define VS_NOF_SIGNALS                   0X000u  /*   0 */
#define VS_NOF_STATE_MACHINES            0X001u  /*   1 */
#define VS_NOF_STATES                    0X009u  /*   9 */


/*
 * Undefined State.
 */
#define STATE_UNDEFINED                  0X0FFu  /* 255 */


/*
 * Undefined Event.
 */
#define EVENT_UNDEFINED                  0X0FFu  /* 255 */


/*
 * Undefined Event Group.
 */
#define EVENT_GROUP_UNDEFINED            0X0FFu  /* 255 */


/*
 * Event Termination ID.
 */
#define EVENT_TERMINATION_ID             0X0FFu  /* 255 */


/*
 * Action Expression Termination ID.
 */
#define ACTION_EXPRESSION_TERMINATION_ID 0X0FFu  /* 255 */
#define ACTION_FPT_NAME System1VSAction


/*
 * Functional expression type definitions
 */
typedef VS_BOOL (* VS_GUARDEXPR_TYPE) (VS_VOID);
typedef VS_VOID (* VS_ACTIONEXPR_TYPE) (VS_VOID);


/*
 * SEM Library Datatype Definition.
 */
typedef struct SEMDATASystem1
{
  VS_UINT8                                      Status;
  VS_UINT8                                      State;
  VS_UINT8                                      DIt;
  VS_UINT8                                      nAction;
  SEM_EVENT_TYPE                                EventNo;
  SEM_RULE_INDEX_TYPE                           _iRI;
  SEM_RULE_TABLE_INDEX_TYPE                     iFirstR;
  SEM_RULE_TABLE_INDEX_TYPE                     iLastR;
  SEM_STATE_TYPE                                CSV[VS_NOF_STATE_MACHINES];
  SEM_STATE_TYPE                                WSV[VS_NOF_STATE_MACHINES];
} SEMDATASystem1;


/*
 * VS System Datatype Definition.
 */
typedef struct
{
  VS_UINT8       SMI[0X009];
  VS_UINT8       RD[0X0188];
  VS_UINT16      RI[0X0002e];
  VS_UINT8       RTI[0X014];
} VSDATASystem1;


/*
 * Data External Declaration.
 */
extern VSDATASystem1 const System1;

extern SEMDATASystem1 SEMSystem1;


/*
 * Action Expression Collection Macro.
 */
#define VSAction                       System1VSAction


#endif /* ifndef _visualSTATE_SYSTEM1SEMBDEF_H */
