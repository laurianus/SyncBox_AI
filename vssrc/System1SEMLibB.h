#ifndef __System1SEMLIBB_H
#define __System1SEMLIBB_H

/*
 * Id:        System1SEMLibB.h
 *
 * Function:  Contains definitions needed for API functions.
 *
 * Generated: Sat Jan 04 00:01:30 2020
 *
 * Coder 6, 3, 2, 1841
 * 
 * This is an automatically generated file. It will be overwritten by the Coder.
 * 
 * DO NOT EDIT THE FILE!
 */


#include <stddef.h>


/* Include SEM Library Definition Header File. */
#include "System1SEMBDef.h"


#if (VS_CODER_GUID != 0X01b841080L)
#error The generated file does not match the SEMTypes.h header file.
#endif


#ifndef VS_TRUE
#define VS_TRUE (1)
#endif


#ifndef VS_FALSE
#define VS_FALSE (0)
#endif


#ifdef VS_RUNTIME_INFO
typedef struct
{
  signed char pSignatureVersion[VS_SIGNATURE_VERSION_LENGTH];
  signed char pSignatureContent[VS_SIGNATURE_CONTENT_LENGTH];
} System1VSRunTimeInfo;
#endif


#ifndef VS_COMPLETION_CODES_DEFINED
#define VS_COMPLETION_CODES_DEFINED
enum
{
  /*
   * Status code:     SES_OKAY.
   *
   * Explanation:     Function performed successfully.
   *
   * Solution:        None.
   */
  SES_OKAY, /* 0 */


  /*
   * Status code:     SES_FOUND.
   *
   * Explanation:     The called function has returned an identifier index number.
   *
   * Solution:        Process the returned identifier index number. If the
   *                  function SEM_GetInput or SEM_GetOutput was called, the
   *                  function can be called again to find more events or
   *                  action expressions.
   */
  SES_FOUND, /* 1 */


  /*
   * Status code:     SES_ACTIVE.
   *
   * Explanation:     The completion code has one of the two expositions:
   *                  1)  A state/event deduction is started, while an event
   *                      inquiry is active. All inquired events have not been
   *                      returned by the function SEM_GetInput.
   *                  2)  An event inquiry is started, while a state/event
   *                      deduction is active. All deduced action expressions 
   *                      have not been returned by the function SEM_GetOutput 
   *                      and the SEM_NextState has not been called in order to 
   *                      complete the state/event deduction.
   *
   * Solution:        The completion code is a warning and perhaps the
   *                  application program should be rewritten. An event inquiry
   *                  and a state/event deduction should not be active at the
   *                  same time.
   */
  SES_ACTIVE, /* 2 */


  /*
   * Error code:      SES_CONTRADICTION.
   *
   * Explanation:     A contradiction has been detected between two states in a
   *                  state machine.
   *
   * Solution:        Check the VS System.
   */
  SES_CONTRADICTION, /* 3 */


  /*
   * Error code:      SES_RANGE_ERR.
   *
   * Explanation:     You are making a reference to an identifier that does not
   *                  exist. Note that the first index number is 0. If the 
   *                  VS System has 4 identifiers of the same type and you use a
   *                  function with the variable parameter = 4, the function 
   *                  will return an SES_RANGE_ERR error. In this case the 
   *                  highest variable index number is 3.
   *
   * Solution:        The application program will check the index parameters 
   *                  with one of the following symbols defined in the SEMBDef.h 
   *                  file:
   *
   *                    VS_NOF_EVENTS
   *                    VS_NOF_STATES
   *                    VS_NOF_ACTION_FUNCTIONS
   *                    VS_NOF_STATE_MACHINES
   */
  SES_RANGE_ERR, /* 4 */


  /*
   * Error code:      SES_TEXT_TOO_LONG.
   *
   * Explanation:     The requested text is longer than the specified maximum length.
   *
   * Solution:        Increase the maximum length.
   */
  SES_TEXT_TOO_LONG, /* 5 */


   /*
   * Error code:      SES_TYPE_ERR.
   *
   * Explanation:     A text function has been called with the wrong identifier
   *                  type or the specified text is not included in the VS System.
   *
   * Solution:        Use the identifier type symbols (EVENT_TYPE, STATE_TYPE
   *                  or ACTION_TYPE) defined in this file and remember
   *                  to include wanted text in the VS System.
   */
  SES_TYPE_ERR, /* 6 */


  /*
   * Error code:      SES_EMPTY.
   *
   * Explanation:     No events have been given to the SEM_Deduct function before
   *                  calling this function.
   *
   * Solution:        Call the SEM_Deduct function with an event number.
   */
  SES_EMPTY, /* 7 */


  /*
   * Error code:      SES_BUFFER_OVERFLOW.
   *
   * Explanation:     A destination buffer cannot hold the number of items found.
   *
   * Solution:        Call the function with an extended buffer as destination.
   */
  SES_BUFFER_OVERFLOW, /* 8 */


  /*
   * Error code:      SES_SIGNAL_QUEUE_FULL.
   *
   * Explanation:     Signal queue is full.
   *
   * Solution:        Increase the maximum signal queue size in the VS System or
   *                  via the VS Coder signal queue size option.
   */
  SES_SIGNAL_QUEUE_FULL /* 9 */
};
#endif /* VS_COMPLETION_CODES_DEFINED */


/* Identifier types, used when getting texts and explanations.*/
#ifndef VS_IDENTIFIER_TYPES_DEFINED
#define VS_IDENTIFIER_TYPES_DEFINED
enum {EVENT_TYPE, STATE_TYPE, ACTION_TYPE};
#endif /* VS_IDENTIFIER_TYPES_DEFINED */


/*
 * Name        : SEM_InitAll
 *
 * Description : The function is a wrapper to all initialization functions.
 *               The function calls the following functions in the listed
 *               order (provided the specific function exists):
 *                 SEM_Init
 *                 SEM_InitExternalVariables
 *                 SEM_InitInternalVariables
 *                 SEM_InitSignalQueue
 *                 SEM_InitInstances
 *
 * Argument    : None.
 *
 * Return      : None.
 *
 * Portability : ANSI-C Compiler.
 */
void System1SEM_InitAll (void);


/*
 * Name        : SEM_Init
 *
 * Description : Before calling any other functions this function must be
 *               called to initialize the VS System. In addition it is
 *               possible to call the initialization functions for the
 *               signal queue, internal variables and external variables.
 *
 * Argument    : None.
 *
 * Return      : None.
 *
 * Portability : ANSI-C Compiler.
 */
void System1SEM_Init (void);


/*
 * Name        : SEM_Deduct
 *
 * Description : The function prepares the reception of action expressions,
 *               that can be deduced on the basis of the given event, the
 *               internal current state vector and the rules in the VS
 *               System.
 *               All action expressions are deduced by continuously calling
 *               the function SEM_GetOutput or one call to the
 *               SEM_GetOutputAll function.
 *               Before calling the SEM_Deduct function again, the
 *               SEM_NextState or SEM_NextStateChg must be called to enter
 *               the new states, if any.
 *
 * Argument    : EventNo:
 *                 Event number to be processed.
 *
 *               Variable number of arguments:
 *                 Used if at least one event has a parameter.
 *                 The function call must include one argument for each type
 *                 name declared in the parameter list for each event.
 *                 
 *                 This sample declaration is for an event with three
 *                 parameters:
 *                 
 *                   EventName (VS_UINT8 par1, VS_CHAR par2, VS_INT par3)
 *                 
 *                 How to call the SEM_Deduct function for the event
 *                 EventName:
 *                 
 *                   SEM_Deduct (EventName, par1, par2, par3);
 *
 * Return      : Completion code:
 *
 *                 SES_ACTIVE:
 *                   The function SEM_Inquiry has been called. All inquired
 *                   events have not been returned by the function
 *                   SEM_GetInput or SEM_GetInputAll. The state/event
 *                   deduction is okay, but the user should not call the
 *                   function SEM_GetInput before the functions
 *                   SEM_NextState or SEM_NextStateChg and SEM_Inquiry have
 *                   been called.
 *
 *                 SES_RANGE_ERR:
 *                   Event is out of range.
 *
 *                 SES_OKAY:
 *                   Success.
 *
 * Portability : ANSI-C Compiler.
 */
unsigned char System1SEM_Deduct (SEM_EVENT_TYPE EventNo);


/*
 * Name        : SEM_GetOutput
 *
 * Description : The function finds an action expression, if any such one
 *               has been deduced on the basis of the event given to the
 *               function SEM_Deduct, the internal current state vector and
 *               the rules in the VS System.
 *               All action expressions are found by continuous calls to the
 *               SEM_GetOutput function.
 *               When the SEM_GetOutput returna the completion code
 *               SES_OKAY, all action expressions have been found.
 *               Use SEM_Action or SEM_TableAction to call the action
 *               expression functions.
 *
 * Argument    : ActionNo:
 *                 Pointer to store the deduced action expression.
 *
 * Return      : Completion code:
 *
 *                 SES_CONTRADICTION:
 *                   Contradiction detected, the VS System is not
 *                   consistent. Check the VS System. You will also get this
 *                   error code here if you forget to call SEM_Init
 *                   function.
 *
 *                 SES_EMPTY:
 *                   No event has been given to the SEM_Deduct function,
 *                   i.e. SEM_Deduct has not been called before calling the
 *                   SEM_GetOutput function.
 *
 *                 SES_FOUND:
 *                   An action expression has been found. The user can
 *                   continue to call the SEM_GetOutput in order to have
 *                   more action expressions found.
 *
 *                 SES_SIGNAL_QUEUE_FULL:
 *                   The signal queue is full. Increase the signal queue
 *                   size in the VS System.
 *
 *                 SES_OKAY:
 *                   Success. All action expressions have been found. The
 *                   user can now call the SEM_NextState or SEM_NextStateChg
 *                   function to change state.
 *                   If SEM_Deduct is called with the same event and
 *                   SEM_NextState and SEM_NextStateChg is not called, all
 *                   action expresssions can be found again by calling
 *                   SEM_GetOutput or SEM_GetOutputAll.
 *
 * Portability : ANSI-C Compiler.
 */
unsigned char System1SEM_GetOutput (SEM_ACTION_EXPRESSION_TYPE *ActionNo);


/*
 * Name        : SEM_NextState
 *
 * Description : The function will update the internal current state vector
 *               if any states can be found on the basis of the given event
 *               to the function SEM_Deduct, the internal current state
 *               vector and the rules in the VS System.
 *
 * Argument    : None.
 *
 * Return      : Completion code:
 *
 *                 SES_CONTRADICTION:
 *                   Contradiction detected, the VS System is not
 *                   consistent. Check the VS System. Your will also get
 *                   this error code here if youforget to call SEM_Init.
 *
 *                 SES_EMPTY:
 *                   No event has been given to the SEM_Deduct function,
 *                   i.e. SEM_Deduct has not been called before calling the
 *                   SEM_NextState function.
 *
 *                 SES_OKAY:
 *                   Success. The internal state vector was updated.
 *
 * Portability : ANSI-C Compiler.
 */
unsigned char System1SEM_NextState (void);


/*
 * Name        : SEM_Action
 *
 * Description : The macro will call an action expression function by using
 *               the VSAction function pointer table.
 *
 * Argument    : ActionNo:
 *                 Action expression index number.
 *
 * Return      : None.
 *
 * Portability : ANSI-C Compiler.
 */
#define System1SEM_Action(ActionNo) ((*System1VSAction[(ActionNo)])())


/*
 * Name        : SEM_TableAction
 *
 * Description : The macro will call an action expression function by using
 *               the specified function pointer table.
 *
 * Argument    : Table:
 *                 Function pointer table.
 *
 *               ActionNo:
 *                 Action expression index number.
 *
 * Return      : None.
 *
 * Portability : ANSI-C Compiler.
 */
#define System1SEM_TableAction(Table, ActionNo) ((*((Table))[(ActionNo)])())


#endif
