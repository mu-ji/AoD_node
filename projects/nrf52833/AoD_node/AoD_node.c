/**
\brief This program is for AoD node

At the beginning of the program, always turn on the radio and keep listening,
untill received a packet then do angle calculation and time synchronization.

After synchronization, only turn on the radio and listen when the AoD anchor
will send a packet. 

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>
*/

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio_df.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         0              ///< 0~39
#define TIMER_PERIOD    (0xffff>>4)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LENGTH_SERIAL_FRAME  127              // length of the serial frame

#define ENABLE_DF       1

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

#define NUM_SLOTS       5
#define SLOTDURATION    (32768/200)  // 5ms@ (32768/200)
#define SENDING_OFFSET  (32768/1000) // 1ms@ (32768/200)


//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_SYNC_DONE   = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
    uint8_t              num_slot;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                uint8_t         flags;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint32_t        sample_buffer[NUM_SAMPLES];
                //uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                //uint16_t        uart_lastTxByteIndex;

                uint8_t         txpk_txNow;
                uint8_t         pkt_sqn;
                
                uint8_t         slot_timerId;
                uint8_t         inner_timerId;
                uint8_t         slot_offset;

                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

void     cb_slot_timer(void);
void     cb_inner_slot_timer(void);

void     assemble_ibeacon_packet(uint8_t);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;

    uint8_t current_time;

    uint8_t packet_counter;
    packet_counter = 0;


    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

#if ENABLE_DF == 1
    radio_configure_direction_finding_antenna_switch();
    radio_configure_direction_finding_manual();
#endif

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);

    app_vars.slot_timerId = 0;
    app_vars.inner_timerId = 1;

    // start sctimer
    sctimer_set_callback(app_vars.slot_timerId, cb_slot_timer);
    sctimer_set_callback(app_vars.inner_timerId, cb_inner_slot_timer);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);


    while (1) {
        switch(app_vars.flags) {
        case APP_FLAG_SYNC_DONE:
        break;
        default:
            board_sleep();
        }


        app_vars.txpk_txNow = 0;
        while (app_vars.txpk_txNow==0) {
            board_sleep();
        }

        // if I get here, I just received a packet

        //===== send notification over serial port

        // led
        //leds_error_on();
        leds_error_toggle();
        // prepare packet
        app_vars.packet_len = sizeof(app_vars.packet);
        
        assemble_ibeacon_packet(packet_counter);

        // start transmitting packet
        radio_loadPacket(app_vars.packet,LENGTH_PACKET);

        radio_txEnable();
        radio_txNow();
        packet_counter++;
    }
}
//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t pkt_sqn) {

    uint8_t i;
    i=0;

    memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

    app_vars.packet[i++]  = 0x42;               // BLE ADV_NONCONN_IND (this is a must)
    app_vars.packet[i++]  = 0x21;               // Payload length
    app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
    app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
    app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
    app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
    app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
    app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

    app_vars.packet[i++]  = 0x1a;
    app_vars.packet[i++]  = 0xff;
    app_vars.packet[i++]  = 0x4c;
    app_vars.packet[i++]  = 0x00;

    app_vars.packet[i++]  = 0x02;
    app_vars.packet[i++]  = 0x15;
    memcpy(&app_vars.packet[i], &ble_uuid[0], 16);
    i                    += 16;
    app_vars.packet[i++]  = 0x00;               // major
    app_vars.packet[i++]  = 0xff;
    app_vars.packet[i++]  = 0x00;               // minor
    app_vars.packet[i++] = pkt_sqn;
    app_vars.packet[i++]  = TXPOWER;            // power level
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_START_FRAME;

    // update debug stats
    switch (app_vars.flags) {
    // if sync done, do nothing
    case APP_FLAG_SYNC_DONE:
    break;
    // if sync not done, sync board
    default:
        sctimer_setCompare(app_vars.slot_timerId, sctimer_readCounter() + SLOTDURATION - SENDING_OFFSET);
        app_vars.slot_offset = 0;
    }
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_END_FRAME;

    // update debug stats
    app_dbg.num_endFrame++;
    switch (app_vars.flags) {
    
    case APP_FLAG_SYNC_DONE:
    break;
    default:
        app_vars.flags |= APP_FLAG_SYNC_DONE;
    }
}

void cb_slot_timer(void) {
    // set flag
    //app_vars.flags |= APP_FLAG_TIMER;

    // update debug stats
    app_dbg.num_timer++;
    app_vars.txpk_txNow = 1;

    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
}

void cb_inner_slot_timer(void) {

    radio_txNow();
}


