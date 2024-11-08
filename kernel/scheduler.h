#ifndef __SCHEDULER_H
#define __SCHEDULER_H

/**
\addtogroup kernel
\{
\addtogroup Scheduler
\{
*/

#include "opendefs.h"

//=========================== define ==========================================

typedef enum {
    TASKPRIO_NONE                  = 0x00,
    // tasks trigger by radio
    TASKPRIO_SIXTOP_NOTIF_RX       = 0x01,
    TASKPRIO_SIXTOP_NOTIF_TXDONE   = 0x02,
    // tasks triggered by timers
    TASKPRIO_OPENTIMERS            = 0x03,
    TASKPRIO_SIXTOP                = 0x04,
    TASKPRIO_FRAG                  = 0x05,
    TASKPRIO_IPHC                  = 0x06,
    TASKPRIO_RPL                   = 0x07,
    TASKPRIO_UDP                   = 0x08,
    TASKPRIO_COAP                  = 0x09,
    TASKPRIO_ADAPTIVE_SYNC         = 0x0a,
    TASKPRIO_MSF                   = 0x0b,
    // tasks trigger by other interrupts
    TASKPRIO_BUTTON                = 0x0c,
    TASKPRIO_SIXTOP_TIMEOUT        = 0x0d,
    TASKPRIO_SNIFFER               = 0x0e,
    TASKPRIO_OPENSERIAL            = 0X0f,
    TASKPRIO_MAX                   = 0x10,
} task_prio_t;

#define TASK_LIST_DEPTH           10

//=========================== typedef =========================================

typedef void (*task_cbt)(void);

typedef struct task_llist_t {
   task_cbt                       cb;
   task_prio_t                    prio;
   void*                          next;
} taskList_item_t;

//=========================== module variables ================================

typedef struct {
   taskList_item_t                taskBuf[TASK_LIST_DEPTH];
   taskList_item_t*               task_list;
   uint8_t                        numTasksCur;
   uint8_t                        numTasksMax;
} scheduler_vars_t;

typedef struct {
   uint8_t                        numTasksCur;
   uint8_t                        numTasksMax;
} scheduler_dbg_t;

//=========================== prototypes ======================================

void scheduler_init(void);
void scheduler_start(void);
void scheduler_push_task(task_cbt task_cb, task_prio_t prio);

/**
\}
\}
*/

#endif
