/*****************************************************************************
* IAR visualSTATE Static Event Handler Source File (sample code)
*
* The file implements an event queue with a static size, specified by the
* value of the macro EVENT_QUEUE_SIZE.
*
* Whenever the queue runs full (by a call to SEQ_AddEvent), the event is ig-
* nored and an error is returned.
*
* A function is provided for initializing the queue, as well as functions for
* adding and retrieving events from the queue. See the associated header file
* for descriptions and use of those functions.
*
* The queue is a static variable in this file. If multiple queues are to be
* used, the static variable must be removed and the queue handling functions
* must be modified to take a parameter identifying a particular queue, for
* example a pointer to a queue.
*****************************************************************************/



/* *** include directives *** */

#include "simpleEventHandler.h"
#include "stm32f4xx_hal.h"  /* For __get_PRIMASK / __disable_irq / __enable_irq */



/* *** macro definitions *** */

/** Event queue size. Defines the size of the event queue. */
#ifndef EVENT_QUEUE_SIZE
#define EVENT_QUEUE_SIZE 100
#endif



/* *** type definitions *** */

/** Event queue size type. Defines the type for indexes into the event queue.
    The type must be an unsigned ordinal type, capable of holding values in
    the range [0;EVENT_QUEUE_SIZE[. */
#if (EVENT_QUEUE_SIZE <= 0x0100)
typedef VS_UINT8 SEQ_SIZE_TYPE;
#elif (EVENT_QUEUE_SIZE <= 0x010000)
typedef VS_UINT16 SEQ_SIZE_TYPE;
#elif (EVENT_QUEUE_SIZE <= 0x01000000)
typedef VS_UINT32 SEQ_SIZE_TYPE;
#else
#error Cannot determine type of SEQ_SIZE_TYPE
#endif



/** Event queue type. Defines the internal structure of the event queue. */
struct StaticEventQueue
{
  /** Event queue. The array stores events in the queue. */
  volatile SEM_EVENT_TYPE queue[EVENT_QUEUE_SIZE];

  /** Front index into the queue. The front index specifies the index into the
      array, where the next event will be retrieved from. */
  volatile SEQ_SIZE_TYPE front;

  /** Back index into the queue. The back index specifies the index into the
      array, where the next event will be stored. */
  volatile SEQ_SIZE_TYPE back;
};
typedef struct StaticEventQueue StaticEventQueue;



/* *** variable definitions *** */

/** The event queue. If more than one event queue is needed (for example when
    a visualSTATE project contains more than one system), the event queue must
    be a parameter for all event queue handling functions instead of being a
    static variable. */
static StaticEventQueue eventQueue;



/* *** function definitions *** */

void SEQ_Initialize(void)
{
  SEQ_Clear();
}



UCC_TYPE SEQ_AddEvent(SEM_EVENT_TYPE event)
{
  UCC_TYPE result;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();  /* Critical section: protect shared queue indices */

  /* Test whether the queue is full (use modulo for correct wrap-around). */
  if (((eventQueue.back + 1) % EVENT_QUEUE_SIZE) == eventQueue.front)
  {
    result = UCC_QUEUE_FULL;
  }
  else
  {
    /* Store the event in the queue. */
    eventQueue.queue[eventQueue.back] = event;

    /* Increment the back queue index. */
    eventQueue.back = (SEQ_SIZE_TYPE) ((eventQueue.back + 1) % EVENT_QUEUE_SIZE);

    result = UCC_OK;
  }

  if (!primask) __enable_irq();  /* Restore previous interrupt state */
  return result;
}




UCC_TYPE SEQ_RetrieveEvent(SEM_EVENT_TYPE* pEvent)
{
  /* Test whether the queue is empty. */
  if (eventQueue.front == eventQueue.back)
    return UCC_QUEUE_EMPTY;

  /* Retrieve event from queue. */
  *pEvent = eventQueue.queue[eventQueue.front];

  /* Increment the front queue index. */
  eventQueue.front = (SEQ_SIZE_TYPE) ((eventQueue.front + 1) % EVENT_QUEUE_SIZE);

  return UCC_OK;
}



void SEQ_Clear(void)
{
  eventQueue.front = eventQueue.back = 0;
}



VS_BOOL SEQ_EventPendingP(void)
{
  return (VS_BOOL) (eventQueue.front != eventQueue.back);
}
