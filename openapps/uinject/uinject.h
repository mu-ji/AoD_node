#ifndef __UINJECT_H
#define __UINJECT_H

/**
\addtogroup AppUdp
\{
\addtogroup uinject
\{
*/

#include "config.h"
#include "opentimers.h"
#include "udp.h"

//=========================== define ==========================================

#define UINJECT_PERIOD_MS 60000

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    opentimers_id_t timerId;   ///< periodic timer which triggers transmission
    uint16_t counter;  ///< incrementing counter which is written into the packet
    uint16_t period;  ///< uinject packet sending period>
    udp_resource_desc_t desc;  ///< resource descriptor for this module, used to register at UDP stack
    bool busySendingUinject;  ///< TRUE when busy sending an uinject
} uinject_vars_t;

//=========================== prototypes ======================================

void uinject_init(void);

void uinject_sendDone(OpenQueueEntry_t *msg, owerror_t error);

void uinject_receive(OpenQueueEntry_t *msg);
/**
\}
\}
*/

#endif

