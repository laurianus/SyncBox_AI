/*
 * Id:        System1SEMLibB.c
 *
 * Function:  Contains all API functions.
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


/*
 * Used internally in the API
 */
#define STATE_SEM_INITIALIZE  0x00u
#define STATE_SEM_SIGNAL      0x01u
#define STATE_SEM_PREPARE     0x02u
#define STATE_SEM_CONSULT     0x03u
#define STATE_SEM_OUTPUT      0x04u
#define STATE_SEM_OKAY        0x05u


/*
 * Conditional Compilation Definitions for the API only.
 */
#define SEM_RDHW_TYPE_1                    1
#define SEM_RDHW_TYPE_2                    0
#define SEM_RDHW_TYPE_3                    0
#define SEM_RDHW_WIDTH_16_BIT              1
#define SEM_RDHW_WIDTH_24_BIT              0
#define SEM_RDHW_WIDTH_32_BIT              0
#define SEM_RDHW_WIDTH_48_BIT              0
#define SEM_RDHW_WIDTH_64_BIT              0
#define SEM_RD_WIDTH_8_BIT                 1
#define SEM_RD_WIDTH_16_BIT                0
#define SEM_RD_WIDTH_32_BIT                0
#define SEM_RDFM_NUMBER                    0
#define SEM_EVENT_GROUP_INDEX              0
#define SEM_EVENT_GROUP_TABLE_INDEX        0
#define SEM_SIGNAL_QUEUE_ERROR_IF_FULL     1
#define SEM_SIGNAL_QUEUE_NO_ERROR_IF_FULL  0
#define SEM_RMN_ACTIONS                    9
#define SEM_RMN_GUARDS                     0
#define SEM_RMN_NEGATIVE_STATE_SYNCS       0
#define SEM_RMN_NEXT_STATES                1
#define SEM_RMN_POSITIVE_STATE_SYNCS       1
#define SEM_RMN_SIGNALS                    0
#define SEM_SIGNAL                         0
#define SEM_EXPL                           0
#define SEM_EXPL_ABS                       0
#define SEM_FORCE_STATE                    0
#define SEM_GET_OUTPUT_ALL                 0
#define SEM_INIT_ALL                       1
#define SEM_MACHINE                        0
#define SEM_NAME                           0
#define SEM_NAME_ABS                       0
#define SEM_NEXT_STATE_CHG                 0
#define SEM_SIGNAL_QUEUE_INFO              0
#define SEM_STATE                          0
#define SEM_STATE_ALL                      0
#define SEM_INIT_EXTERNAL_VARIABLES        0
#define SEM_INIT_INTERNAL_VARIABLES        0
#define VS_ACTION_EXPLS                    0
#define VS_ACTION_FUNCTION_NAMES           0
#define VS_EVENT_EXPLS                     0
#define VS_EVENT_NAMES                     0
#define VS_STATE_EXPLS                     0
#define VS_STATE_NAMES                     0


/* Core model logic struct name */
#define VS System1


/* SEM data struct name */
#define SEM SEMSystem1


#ifdef VS_RUNTIME_INFO
System1VSRunTimeInfo volatile const VS_RUNTIME_INFO_EXTKW System1vsRunTimeInfo = 
{
  VS_SIGNATURE_VERSION,
  VS_SIGNATURE_CONTENT
};
#endif


#if (SEM_SIGNAL)
void System1SEM_InitSignalQueue (void)
{
  SEM.SPut = 0u;
  SEM.SGet = 0u;
  SEM.SUsed = 0u;
}


static unsigned char SEM_SignalQueuePut (SEM_EVENT_TYPE SignalNo)
{
  unsigned char result;

  if (SEM.SUsed == 2u)
  {
    result = SES_SIGNAL_QUEUE_FULL;
  }
  else
  {
    SEM.SUsed++;
    SEM.SQueue[SEM.SPut] = SignalNo;
    if (++SEM.SPut == 2u)
    {
      SEM.SPut = 0u;
    }
    result = SES_OKAY;
  }
  return (result);
}


static SEM_EVENT_TYPE SEM_SignalQueueGet (void)
{
  SEM_EVENT_TYPE SignalNo = EVENT_UNDEFINED;

  if (SEM.SUsed)
  {
    SEM.SUsed--;
    SignalNo = SEM.SQueue[SEM.SGet];
    if (++SEM.SGet == 2u)
    {
      SEM.SGet = 0u;
    }
  }
  return (SignalNo);
}


#if (SEM_SIGNAL_QUEUE_INFO == 1)
void System1SEM_SignalQueueInfo (SEM_SIGNAL_QUEUE_TYPE *NofSignals)
{
  *NofSignals = SEM.SUsed;
}
#endif
#endif


#if (SEM_INIT_ALL)
void System1SEM_InitAll (void)
{
  System1SEM_Init();
#if (SEM_INIT_EXTERNAL_VARIABLES)
  System1SEM_InitExternalVariables();
#endif
#if (SEM_INIT_INTERNAL_VARIABLES)
  System1SEM_InitInternalVariables();
#endif
#if (SEM_SIGNAL)
  System1SEM_InitSignalQueue();
#endif
}
#endif


void System1SEM_Init (void)
{
#ifdef VS_RUNTIME_INFO
  *System1vsRunTimeInfo.pSignatureVersion;
#endif
#if (VS_NOF_STATE_MACHINES != 0u)
  {
    SEM_STATE_MACHINE_TYPE i;
    for (i = 0u; i < VS_NOF_STATE_MACHINES; i++)
    {
      SEM.WSV[i] = STATE_UNDEFINED;
      SEM.CSV[i] = STATE_UNDEFINED;
    }
  }
#if (SEM_NEXT_STATE_CHG == 1)
  SEM.Chg = 0;
#endif
#endif
  SEM.State = STATE_SEM_INITIALIZE;
}


unsigned char System1SEM_Deduct (SEM_EVENT_TYPE EventNo)
{
  if (VS_NOF_EVENTS <= EventNo)
  {
    return (SES_RANGE_ERR);
  }
  SEM.EventNo = EventNo;
  SEM.State = STATE_SEM_PREPARE;
#if (VS_NOF_EVENT_GROUPS != 0u)
  SEM.DIt = 2;
#endif
  return (SES_OKAY);
}


unsigned char System1SEM_GetOutput (SEM_ACTION_EXPRESSION_TYPE *ActionNo)
{
  for(;;)
  {
    switch (SEM.State)
    {
#if (SEM_SIGNAL)
    case STATE_SEM_SIGNAL:
Signal:
      SEM.EventNo = SEM_SignalQueueGet();
      if (SEM.EventNo == EVENT_UNDEFINED)
      {
        SEM.State = STATE_SEM_OKAY;
        return (SES_OKAY);
      }
#if (VS_NOF_STATE_MACHINES != 0u)
      {
        SEM_STATE_MACHINE_TYPE i;
        for (i = 0u; i < VS_NOF_STATE_MACHINES; i++)
        {
          if (SEM.WSV[i] != STATE_UNDEFINED)
          {
#if (SEM_NEXT_STATE_CHG == 1)
            if (SEM.CSV[i] != SEM.WSV[i])
            {
              SEM.Chg = 1;
            }
#endif
            SEM.CSV[i] = SEM.WSV[i];
            SEM.WSV[i] = STATE_UNDEFINED;
          }
        }
      }
#endif
      goto FirstRule;
#endif

    case STATE_SEM_PREPARE:
#if (VS_NOF_EVENT_GROUPS != 0u)
#if (SEM_EVENT_GROUP_INDEX)
      if (SEM.DIt == 1)
      {
        SEM.EventNo = VS.EGI[SEM.EventNo];
        if (SEM.EventNo == EVENT_GROUP_UNDEFINED)
        {
#if (SEM_SIGNAL)
          SEM.DIt = 0;
          goto Signal;
#else
          SEM.State = STATE_SEM_OKAY;
          return (SES_OKAY);
#endif
        }
        SEM.EventNo += VS_NOF_EVENTS;
      }
      else if (SEM.DIt == 0)
      {
#if (SEM_SIGNAL)
        goto Signal;
#else
        SEM.State = STATE_SEM_OKAY;
        return (SES_OKAY);
#endif
      }
      SEM.DIt--;
#endif
#if (SEM_EVENT_GROUP_TABLE_INDEX)
      if (SEM.DIt == 0)
      {
        if (++SEM.iFirstEgi >= SEM.iLastEgi)
        {
#if (SEM_SIGNAL)
          goto Signal;
#else
          SEM.State = STATE_SEM_OKAY;
          return (SES_OKAY);
#endif
        }
        SEM.EventNo = VS.EGT[SEM.iFirstEgi];
        SEM.EventNo += VS_NOF_EVENTS;
      }
      else if (SEM.DIt == 1)
      {
        SEM.iFirstEgi = VS.EGTI[SEM.EventNo];
        SEM.iLastEgi = VS.EGTI[SEM.EventNo + 1];
        if (SEM.iFirstEgi == SEM.iLastEgi)
        {
#if (SEM_SIGNAL)
          SEM.DIt = 0;
          goto Signal;
#else
          SEM.State = STATE_SEM_OKAY;
          return (SES_OKAY);
#endif
        }
        SEM.EventNo = VS.EGT[SEM.iFirstEgi];
        SEM.EventNo += VS_NOF_EVENTS;
        SEM.DIt--;
      }
      else if (SEM.DIt == 2)
      {
        SEM.DIt--;
      }
#endif
#endif
#if (SEM_SIGNAL)
FirstRule:
#endif
      SEM.iFirstR = VS.RTI[SEM.EventNo];
      SEM.iLastR = VS.RTI[SEM.EventNo + 1];
      SEM.State = STATE_SEM_CONSULT;
      /* fall through */

    case STATE_SEM_CONSULT:
      while (SEM.iFirstR < SEM.iLastR)
      {
        SEM_INTERNAL_TYPE i;
        VS_UINT8 nNo;
        VS_UINT8 nPos;
        VS_UINT8 nNxt;
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        VS_UINT8 nNeg;
#endif
#if (SEM_RMN_SIGNALS)
        VS_UINT8 nSignal;
#endif
#if (SEM_RMN_GUARDS)
        VS_UINT8 nGuard;
#endif
        SEM_RULE_INDEX_TYPE iRI;

        iRI = VS.RI[SEM.iFirstR++];
#if (SEM_RD_WIDTH_8_BIT && SEM_RDHW_TYPE_1 && SEM_RDHW_WIDTH_16_BIT)
        i = VS.RD[iRI++];
        nNxt = (unsigned char)(i & 0x0Fu);
        SEM.nAction = (unsigned char)(i >> 4u);
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0Fu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)(i >> 4u);
#endif
#endif
#if (SEM_RD_WIDTH_8_BIT && SEM_RDHW_TYPE_2 && SEM_RDHW_WIDTH_24_BIT)
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0Fu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)(i >> 4u);
#endif
        i = VS.RD[iRI++];
#if (SEM_RMN_GUARDS)
        nGuard = (unsigned char)(i & 0x0Fu);
#endif
        nNxt = (unsigned char)(i >> 4u);
        i = VS.RD[iRI++];
        SEM.nAction = (unsigned char)(i & 0x0Fu);
#if (SEM_RMN_SIGNALS)
        nSignal = (unsigned char)(i >> 4u);
#endif
#endif
#if (SEM_RD_WIDTH_8_BIT && SEM_RDHW_TYPE_1 && SEM_RDHW_WIDTH_32_BIT)
        SEM.nAction = (unsigned char)VS.RD[iRI++];
        nNxt = (unsigned char)VS.RD[iRI++];
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)VS.RD[iRI++];
#else
        iRI++;
#endif
        nPos = (unsigned char)VS.RD[iRI++];
#endif
#if (SEM_RD_WIDTH_8_BIT && SEM_RDHW_TYPE_2 && SEM_RDHW_WIDTH_48_BIT)
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)VS.RD[iRI++];
#else
        iRI++;
#endif
        nPos = (unsigned char)VS.RD[iRI++];
        nNxt = (unsigned char)VS.RD[iRI++];
#if (SEM_RMN_GUARDS)
        nGuard = (unsigned char)VS.RD[iRI];
#endif
        iRI++;
#if (SEM_RMN_SIGNALS)
        nSignal = (unsigned char)VS.RD[iRI];
#endif
        iRI++;
        SEM.nAction = (unsigned char)VS.RD[iRI++];
#endif
#if (SEM_RD_WIDTH_16_BIT && SEM_RDHW_TYPE_1 && SEM_RDHW_WIDTH_16_BIT)
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0Fu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)((i >> 4u) & 0x0Fu);
#endif
        nNxt = (unsigned char)((i >> 8u) & 0x0Fu);
        SEM.nAction = (unsigned char)((i >> 12u) & 0x0Fu);
#endif
#if (SEM_RD_WIDTH_16_BIT && SEM_RDHW_TYPE_3 && SEM_RDHW_WIDTH_32_BIT)
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0Fu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)((i >> 4u) & 0x0Fu);
#endif
#if (SEM_RMN_GUARDS)
        nGuard = (unsigned char)((i >> 8u) & 0x0Fu);
#endif
        nNxt = (unsigned char)(i >> 12u);
        i = VS.RD[iRI++];
        SEM.nAction = (unsigned char)(i & 0x0Fu);
#if (SEM_RMN_SIGNALS)
        nSignal = (unsigned char)((i >> 4u) & 0x0Fu);
#endif
#endif
#if (SEM_RD_WIDTH_16_BIT && SEM_RDHW_TYPE_1 && SEM_RDHW_WIDTH_32_BIT)
        i = VS.RD[iRI++];
        nNxt = (unsigned char)(i & 0x0FFu);
        SEM.nAction = (unsigned char)(i >> 8u);
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0FFu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)(i >> 8u);
#endif
#endif
#if (SEM_RD_WIDTH_16_BIT && SEM_RDHW_TYPE_2 && SEM_RDHW_WIDTH_48_BIT)
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0FFu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)(i >> 8u);
#endif
        i = VS.RD[iRI++];
#if (SEM_RMN_GUARDS)
        nGuard = (unsigned char)(i & 0x0FFu);
#endif
        nNxt = (unsigned char)(i >> 8u);
        i = VS.RD[iRI++];
        SEM.nAction = (unsigned char)(i & 0x0FFu);
#if (SEM_RMN_SIGNALS)
        nSignal = (unsigned char)(i >> 8u);
#endif
#endif
#if (SEM_RD_WIDTH_32_BIT && SEM_RDHW_TYPE_1 && SEM_RDHW_WIDTH_32_BIT)
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0X0FFu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)((i >> 8u) & 0X0FFu);
#endif
        nNxt = (unsigned char)((i >> 16u) & 0x0FFu);
        SEM.nAction = (unsigned char)((i >> 24u) & 0x0FFu);
#endif
#if (SEM_RD_WIDTH_32_BIT && SEM_RDHW_TYPE_3 && SEM_RDHW_WIDTH_64_BIT)
        i = VS.RD[iRI++];
        nPos = (unsigned char)(i & 0x0FFu);
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        nNeg = (unsigned char)((i >> 8u) & 0x0FFu);
#endif
#if (SEM_RMN_GUARDS)
        nGuard = (unsigned char)((i >> 16u) & 0x0FFu);
#endif
        nNxt = (unsigned char)(i >> 24u);
        i = VS.RD[iRI++];
        SEM.nAction = (unsigned char)(i & 0x0FFu);
#if (SEM_RMN_SIGNALS)
        nSignal = (unsigned char)((i >> 8u) & 0x0FFu);
#endif
#endif
#if (VS_NOF_STATE_MACHINES != 0u)
        for (nNo = 0 ; nNo < nPos; nNo++)
        {
          SEM_STATE_TYPE sa;
          sa = (SEM_STATE_TYPE) VS.RD[iRI++];
          if (sa != SEM.CSV[VS.SMI[sa]])
          {
            goto NextRule;
          }
        }
#if (SEM_RMN_NEGATIVE_STATE_SYNCS)
        for (nNo = 0; nNo < nNeg; nNo++)
        {
          SEM_STATE_TYPE sa;
          i = VS.RD[iRI++];
          sa = SEM.CSV[VS.SMI[i]];
          if ((sa == STATE_UNDEFINED) || (sa == (SEM_STATE_TYPE) i))
          {
            goto NextRule;
          }
        }
#endif
#endif

#if (VS_NOF_STATE_MACHINES != 0u)
        for (nNo = 0; nNo < nNxt; nNo++)
        {
          SEM_STATE_TYPE sa;
          sa = (SEM_STATE_TYPE) VS.RD[iRI++];
          i = VS.SMI[sa];
          if (SEM.WSV[i] == STATE_UNDEFINED)
          {
            SEM.WSV[i] = sa;
          }
          else if (SEM.WSV[i] != sa)
          {
            SEM._iRI = iRI;
            return (SES_CONTRADICTION);
          }
        }
#endif
#if (SEM_RMN_SIGNALS)
        if (nSignal)
        {
          for (nNo = 0; nNo < nSignal; nNo++)
          {
            i = VS.RD[iRI++];
#if (SEM_SIGNAL_QUEUE_ERROR_IF_FULL)
            if (SEM_SignalQueuePut((SEM_EVENT_TYPE)i) == SES_SIGNAL_QUEUE_FULL)
            {
              SEM._iRI = iRI;
              return (SES_SIGNAL_QUEUE_FULL);
            }
#endif
#if (SEM_SIGNAL_QUEUE_NO_ERROR_IF_FULL)
            SEM_SignalQueuePut((SEM_EVENT_TYPE)i);
#endif
          }
        }
#endif
        if (SEM.nAction)
        {
          *ActionNo = (SEM_ACTION_EXPRESSION_TYPE)VS.RD[iRI];
          if (SEM.nAction > 1)
          {
            iRI++;
            SEM._iRI = iRI;
            SEM.nAction--;
            SEM.State = STATE_SEM_OUTPUT;
          }
          return (SES_FOUND);
        }
NextRule:
        ;
      }
#if (VS_NOF_EVENT_GROUPS != 0u)
      {
        SEM.State = STATE_SEM_PREPARE;
        break;
      }
#else
      {
#if (SEM_SIGNAL)
        goto Signal;
#else
        SEM.State = STATE_SEM_OKAY;
        return (SES_OKAY);
#endif
      }
#endif

    case STATE_SEM_OUTPUT:
      if (SEM.nAction)
      {
        *ActionNo = (SEM_ACTION_EXPRESSION_TYPE) VS.RD[SEM._iRI++];
        SEM.nAction--;
        return (SES_FOUND);
      }
      SEM.State = STATE_SEM_CONSULT;
      break;

    case STATE_SEM_OKAY:
      return (SES_OKAY);

    default:
      return (SES_EMPTY);
    }
  }
}


#if (SEM_GET_OUTPUT_ALL == 1)
#if (SEM_SIGNAL)
#error SEM_GetOutputAll cannot be used when the Project contains signals.
#endif
unsigned char System1SEM_GetOutputAll (SEM_ACTION_EXPRESSION_TYPE *ActionVector,
  SEM_ACTION_EXPRESSION_TYPE MaxSize)
{
  SEM_ACTION_EXPRESSION_TYPE i;
  unsigned char CC;

  for (i = 0; ((CC = System1SEM_GetOutput(&ActionVector[i])) == SES_FOUND) && i < MaxSize - 1; i++);

  if (CC == SES_OKAY)
  {
    ActionVector[i] = ACTION_EXPRESSION_TERMINATION_ID;
    return (SES_OKAY);
  }
  if (CC == SES_FOUND)
  {
    return (SES_BUFFER_OVERFLOW);
  }
  return (CC);
}
#endif


#if (SEM_NEXT_STATE_CHG == 1)
unsigned char System1SEM_NextStateChg (void)
#else
unsigned char System1SEM_NextState (void)
#endif
{
  unsigned char CC;
  SEM_ACTION_EXPRESSION_TYPE nAction;

  if (SEM.State != STATE_SEM_OKAY)
  {
    while ((CC = System1SEM_GetOutput (&nAction)) == SES_FOUND)
    {
      ;
    }
    if (CC != SES_OKAY)
    {
      return(CC);
    }
  }
#if (VS_NOF_STATE_MACHINES != 0u)
  {
    SEM_STATE_MACHINE_TYPE i;
    for (i = 0u; i < VS_NOF_STATE_MACHINES; i++)
    {
      if (SEM.WSV[i] != STATE_UNDEFINED)
      {
#if (SEM_NEXT_STATE_CHG == 1)
        if (SEM.CSV[i] != SEM.WSV[i])
        {
          SEM.Chg = 1;
        }
#endif
        SEM.CSV[i] = SEM.WSV[i];
        SEM.WSV[i] = STATE_UNDEFINED;
      }
    }
  }
#endif
  SEM.State = STATE_SEM_INITIALIZE;

#if (SEM_NEXT_STATE_CHG == 1)
  if (SEM.Chg)
  {
    SEM.Chg = 0;
    return (SES_FOUND);
  }
  return (SES_OKAY);
#else
  return (SES_OKAY);
#endif
}


#if (SEM_NAME == 1)
unsigned char System1SEM_Name (unsigned char IdentType, 
  SEM_EXPLANATION_TYPE IdentNo, char *Text, unsigned short MaxSize)
{
  char  c, *s;
  unsigned short i;

  if (!MaxSize)
  {
    return (SES_TEXT_TOO_LONG);
  }
  switch (IdentType)
  {
#if (VS_EVENT_NAMES != 0)
    case EVENT_TYPE :
      if (VS_NOF_EVENTS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      s = (char *)VS.ENames + VS.ENI[IdentNo];
      break;
#endif
#if ((VS_STATE_NAMES != 0) && (VS_NOF_STATES != 0u))
    case STATE_TYPE :
      if (VS_NOF_STATES <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      s = (char *)VS.SNames + VS.SNI[IdentNo];
      break;
#endif
#if ((VS_ACTION_FUNCTION_NAMES != 0) && (VS_NOF_ACTION_FUNCTIONS != 0u))
    case ACTION_TYPE :
      if (VS_NOF_ACTION_FUNCTIONS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      s = (char *)VS.ANames + VS.ANI[IdentNo];
      break;
#endif
    default :
      return (SES_TYPE_ERR);
  }
  for (i = 0; i < MaxSize; i++)
  {
    c = *s++;
    *Text++ = c;
    if (c == '\0')
    {
      return (SES_OKAY);
    }
  }
  Text--;
  *Text = '\0';
  return (SES_TEXT_TOO_LONG);
}
#endif


#if (SEM_NAME_ABS == 1)
unsigned char System1SEM_NameAbs (unsigned char IdentType, 
  SEM_EXPLANATION_TYPE IdentNo, char const **Text)
{
  switch (IdentType)
  {
#if (VS_EVENT_NAMES != 0)
    case EVENT_TYPE :
      if (VS_NOF_EVENTS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      *Text = (char const *)&VS.ENames + VS.ENI[IdentNo];
      break;
#endif
#if ((VS_STATE_NAMES != 0) && (VS_NOF_STATES != 0u))
    case STATE_TYPE :
      if (VS_NOF_STATES <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      *Text = (char const *)&VS.SNames + VS.SNI[IdentNo];
      break;
#endif
#if ((VS_ACTION_FUNCTION_NAMES != 0) && (VS_NOF_ACTION_FUNCTIONS != 0u))
    case ACTION_TYPE :
      if (VS_NOF_ACTION_FUNCTIONS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      *Text = (char const *)&VS.ANames + VS.ANI[IdentNo];
      break;
#endif
    default :
      return (SES_TYPE_ERR);
  }
  return (SES_OKAY);
}
#endif


#if (SEM_EXPL == 1)
unsigned char System1SEM_Expl (unsigned char IdentType, SEM_EXPLANATION_TYPE IdentNo,
  char *Text, unsigned short MaxSize)
{
  char  c, *s;
  unsigned short i;

  if (!MaxSize)
  {
    return (SES_TEXT_TOO_LONG);
  }
  switch (IdentType)
  {
#if (VS_EVENT_EXPLS != 0)
    case EVENT_TYPE :
      if (VS_NOF_EVENTS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      s = (char *)VS.EExpls + VS.EEI[IdentNo];
      break;
#endif
#if ((VS_STATE_EXPLS != 0) && (VS_NOF_STATES != 0u))
    case STATE_TYPE :
      if (VS_NOF_STATES <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      s = (char *)VS.SExpls + VS.SEI[IdentNo];
      break;
#endif
#if ((VS_ACTION_EXPLS != 0) && (VS_NOF_ACTION_FUNCTIONS != 0u))
    case ACTION_TYPE :
      if (VS_NOF_ACTION_FUNCTIONS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      s = (char *)VS.AExpls + VS.AEI[IdentNo];
      break;
#endif
    default :
      return (SES_TYPE_ERR);
  }

  for (i = 0; i < MaxSize; i++)
  {
    c = *s++;
    *Text++ = c;
    if (c == '\0')
    {
      return (SES_OKAY);
    }
  }
  Text--;
  *Text = '\0';
  return (SES_TEXT_TOO_LONG);
}
#endif


#if (SEM_EXPL_ABS == 1)
unsigned char System1SEM_ExplAbs (unsigned char IdentType,
  SEM_EXPLANATION_TYPE IdentNo, char const **Text)
{
  switch (IdentType)
  {
#if (VS_EVENT_EXPLS != 0)
    case EVENT_TYPE :
      if (VS_NOF_EVENTS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      *Text = (char const *)&VS.EExpls + VS.EEI[IdentNo];
      break;
#endif
#if ((VS_STATE_EXPLS != 0) && (VS_NOF_STATES != 0u))
    case STATE_TYPE :
      if (VS_NOF_STATES <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      *Text = (char const *)&VS.SExpls + VS.SEI[IdentNo];
      break;
#endif
#if ((VS_ACTION_EXPLS != 0) && (VS_NOF_ACTION_FUNCTIONS != 0u))
    case ACTION_TYPE :
      if (VS_NOF_ACTION_FUNCTIONS <= IdentNo)
      {
        return (SES_RANGE_ERR);
      }
      *Text = (char const *)&VS.AExpls + VS.AEI[IdentNo];
      break;
#endif
    default :
      return (SES_TYPE_ERR);
  }
  return (SES_OKAY);
}
#endif


#if (SEM_STATE == 1)
unsigned char System1SEM_State (SEM_STATE_MACHINE_TYPE StateMachineNo,
  SEM_STATE_TYPE *StateNo)
{
  if (VS_NOF_STATE_MACHINES <= StateMachineNo)
  {
    return (SES_RANGE_ERR);
  }
#if (VS_NOF_STATE_MACHINES != 0u)
  *StateNo = SEM.CSV[StateMachineNo];
#else
  *StateNo = 0;
#endif
  return (SES_FOUND);
}
#endif


#if (SEM_STATE_ALL == 1)
unsigned char System1SEM_StateAll (SEM_STATE_TYPE *StateVector,
  SEM_STATE_MACHINE_TYPE MaxSize)
{
  SEM_STATE_MACHINE_TYPE i;

  if (VS_NOF_STATE_MACHINES > MaxSize)
  {
    return (SES_BUFFER_OVERFLOW);
  }

#if (VS_NOF_STATE_MACHINES != 0u)
  for (i = 0u; i < VS_NOF_STATE_MACHINES; i++)
  {
    StateVector[i] = SEM.CSV[i];
  }
#else
  for (i = 0u; i < VS_NOF_STATE_MACHINES; i++)
  {
    StateVector[i] = 0;
  }
#endif
  return (SES_FOUND);
}
#endif


#if (SEM_MACHINE == 1)
unsigned char System1SEM_Machine (SEM_STATE_TYPE StateNo,
  SEM_STATE_MACHINE_TYPE *StateMachineNo)
{
  if (VS_NOF_STATES <= StateNo)
  {
    return (SES_RANGE_ERR);
  }
#if (VS_NOF_STATE_MACHINES != 0u)
  *StateMachineNo = VS.SMI[StateNo];
#else
  *StateMachineNo = 0;
#endif
  return (SES_FOUND);
}
#endif


#if (SEM_FORCE_STATE == 1)
unsigned char System1SEM_ForceState (SEM_STATE_TYPE StateNo)
{
  if (VS_NOF_STATES <= StateNo)
  {
    return (SES_RANGE_ERR);
  }
#if (VS_NOF_STATE_MACHINES != 0u)
  SEM.CSV[VS.SMI[StateNo]] = StateNo;
#endif
  return (SES_OKAY);
}
#endif
