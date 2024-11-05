#include "config.h"

#if defined(OPENWSN_UECHO_C)

#include "opendefs.h"
#include "uecho.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"

//=========================== variables =======================================

uecho_vars_t uecho_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void uecho_init(void) {
    // clear local variables
    memset(&uecho_vars, 0, sizeof(uecho_vars_t));

    // register at UDP stack
    uecho_vars.desc.port = WKP_UDP_ECHO;
    uecho_vars.desc.callbackReceive = &uecho_receive;
    uecho_vars.desc.callbackSendDone = &uecho_sendDone;
    openudp_register(&uecho_vars.desc);
}

void uecho_receive(OpenQueueEntry_t *request) {
    OpenQueueEntry_t *reply;

    reply = openqueue_getFreePacketBuffer(COMPONENT_UECHO);

    if (reply == NULL) {
        LOG_ERROR(COMPONENT_UECHO, ERR_NO_FREE_PACKET_BUFFER, (errorparameter_t) 0, (errorparameter_t) 0);
        openqueue_freePacketBuffer(request); //clear the request packet as well
        return;
    }

    reply->owner = COMPONENT_UECHO;

    // reply with the same OpenQueueEntry_t
    reply->creator = COMPONENT_UECHO;
    reply->l4_protocol = IANA_UDP;
    reply->l4_destination_port = request->l4_sourcePortORicmpv6Type;
    reply->l4_sourcePortORicmpv6Type = request->l4_destination_port;
    reply->l3_destinationAdd.type = ADDR_128B;

    // copy source to destination to echo.
    memcpy(&reply->l3_destinationAdd.addr_128b[0], &request->l3_sourceAdd.addr_128b[0], 16);

    if (packetfunctions_reserveHeader(&reply, request->length) == E_FAIL) {
        openqueue_freePacketBuffer(reply);
        return;
    }
    memcpy(&reply->payload[0], &request->payload[0], request->length);

    openqueue_freePacketBuffer(request);

    if ((openudp_send(reply)) == E_FAIL) {
        openqueue_freePacketBuffer(reply);
    }
}

void uecho_sendDone(OpenQueueEntry_t *msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}

bool uecho_debugPrint(void) {
    return FALSE;
}

//=========================== private =========================================

#endif /* OPENWSN_UECHO_C */
