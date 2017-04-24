/* ============================================
This example is compatible with ICS IoT Egg device (mbed lpc1768).
2017-04-20 by Jihoon Yang <j.yang@surrey.ac.uk>

Temperature and humidity values are sending to IoTEgg App via BLE. 
And also it can change the RGB LED color by receiving LED value from IoTEgg App via BLE.

ICS IoT Egg BLE Breakout Board has been designed by William Headley <w.headley@surrey.ac.uk>
- BLE RST pin: P0_5
- BLE TX pin: P0_0
- BLE RX pin: P0_1

Copyright (c) 2017 by Institute for Communication Systems (ICS), University of Surrey
Klaus Moessner <k.moessner@surrey.ac.uk>
William Headley <w.headley@surrey.ac.uk>
=============================================== */

#include "mbed.h"
#include "BGLib.h" // for BLE
#include "RGBLed.h" // for RGB LED
#include "HTU21D.h" // for Temperature & Humidity

// BLE
#define BLE_STATE_STANDBY           0
#define BLE_STATE_SCANNING          1
#define BLE_STATE_ADVERTISING       2
#define BLE_STATE_CONNECTING        3
#define BLE_STATE_CONNECTED_MASTER  4
#define BLE_STATE_CONNECTED_SLAVE   5
#define GATT_HANDLE_C_RX_DATA   17  // 0x11, supports "write" operation
#define GATT_HANDLE_C_TX_DATA   20  // 0x14, supports "read" and "indicate" operations
#define BGAPI_GET_RESPONSE(v, dType) dType *v = (dType *)ioteggble.getLastRXPayload()

// BLE state/link status tracker
uint8_t ble_state = BLE_STATE_STANDBY;
uint8_t ble_encrypted = 0;  // 0 = not encrypted, otherwise = encrypted
uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle
uint8_t pre_ble_state;

DigitalOut BLE_RES_PIN(P0_5);

DigitalOut LED(LED1);

Serial pc(USBTX, USBRX,NULL,38400); // tx, rx for pc
Serial eggble(P0_0,P0_1,NULL,38400); // tx, rx for BLE
BGLib ioteggble((Serial *)&eggble, (Serial *)&pc, 1);
HTU21D temphum(P0_27, P0_28); //Temperature and humidity || SDA, SCL
RGBLed eggRGBled(P2_5,P2_4,P2_3); //RGB PWM pins

typedef struct {
    float tempvalue;
    float humvalue;
} sensor_data;

sensor_data eggsensor;
Timer sensortimer;
uint8_t *sensorbuf;
uint32_t temphumCurTimer,temphumOldTimer;
uint32_t temphum_interval=1000; //

void rgbled_update(char R, char G, char B){
    float R_Value = (float)(R&0xFF);
    float G_Value = (float)(G&0xFF);
    float B_Value = (float)(B&0xFF);
    
    eggRGBled.write(R_Value/255,G_Value/255,B_Value/255); 
    #ifdef _DEBUG_  
    pc.printf( "led: %d, %d, %d \n", R,G,B);
    #endif
}

// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

// called when the module begins sending a command
void onBusy() {
    // turn LED on when we're busy
    LED=1;
}

// called when the module receives a complete response or "system_boot" event
void onIdle() {
    // turn LED off when we're no longer busy
    LED=0;
}

// called when the parser does not read the expected response in the specified time limit
void onTimeout() {
    // reset module (might be a bit drastic for a timeout condition though)
    BLE_RES_PIN=0;
    wait(0.05); // wait 5ms
    BLE_RES_PIN=1;
}

// ================================================================
// APPLICATION EVENT HANDLER FUNCTIONS
// ================================================================

void my_ble_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
    #ifdef _DEBUG_
        pc.printf("###\tsystem_boot: { ");
        pc.printf("major: "); pc.printf("%X",msg -> major);
        pc.printf(", minor: "); pc.printf("%X",msg -> minor);
        pc.printf(", patch: "); pc.printf("%X",msg -> patch);
        pc.printf(", build: "); pc.printf("%X",msg -> build);
        pc.printf(", ll_version: "); pc.printf("%X",msg -> ll_version);
        pc.printf(", protocol_version: "); pc.printf("%X",msg -> protocol_version);
        pc.printf(", hw: "); pc.printf("%X",msg -> hw);
        pc.printf(" }\r\n");
    #endif

    // system boot means module is in standby state
    //ble_state = BLE_STATE_STANDBY;
    // ^^^ skip above since we're going right back into advertising below

    // set advertisement interval to 200-300ms, use all advertisement channels
    // (note min/max parameters are in units of 625 uSec)
    
    ioteggble.ble_cmd_gap_set_adv_parameters(320, 480, 7);
    while (ioteggble.checkActivity(1000));
    
    // USE THE FOLLOWING TO LET THE BLE STACK HANDLE YOUR ADVERTISEMENT PACKETS
    // ========================================================================
    // start advertising general discoverable / undirected connectable
    //ioteggble.ble_cmd_gap_set_mode(BGLIB_GAP_GENERAL_DISCOVERABLE, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    //while (ioteggble.checkActivity(1000));

    // USE THE FOLLOWING TO HANDLE YOUR OWN CUSTOM ADVERTISEMENT PACKETS
    // =================================================================

    // build custom advertisement data
    // default BLE stack value: 0201061107e4ba94c3c9b7cdb09b487a438ae55a19
    uint8 adv_data[] = {
        0x02, // field length
        BGLIB_GAP_AD_TYPE_FLAGS, // field type (0x01)
        0x06, // data (0x02 | 0x04 = 0x06, general discoverable + BLE only, no BR+EDR)
        0x11, // field length
        BGLIB_GAP_AD_TYPE_SERVICES_128BIT_ALL, // field type (0x07)
        0xe4, 0xba, 0x94, 0xc3, 0xc9, 0xb7, 0xcd, 0xb0, 0x9b, 0x48, 0x7a, 0x43, 0x8a, 0xe5, 0x5a, 0x19
    };

    // set custom advertisement data
    ioteggble.ble_cmd_gap_set_adv_data(0, 0x15, adv_data);
    while (ioteggble.checkActivity(1000));

    // build custom scan response data (i.e. the Device Name value)
    // default BLE stack value: 140942474c69622055314131502033382e344e4657
    uint8 sr_data[] = {
        0x14, // field length
        BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE, // field type
        'I', 'C', 'S', ' ', 'I', 'o', 'T', 'E', 'g', 'g', ' ', '0', '0', ':', '0', '0', ':', '0', '0'
    };

    // get BLE MAC address
    ioteggble.ble_cmd_system_address_get();
    while (ioteggble.checkActivity(1000));
    BGAPI_GET_RESPONSE(r0, ble_msg_system_address_get_rsp_t);

    // assign last three bytes of MAC address to ad packet friendly name (instead of 00:00:00 above)
    sr_data[13] = (r0 -> address.addr[2] / 0x10) + 48 + ((r0 -> address.addr[2] / 0x10) / 10 * 7); // MAC byte 4 10's digit
    sr_data[14] = (r0 -> address.addr[2] & 0xF)  + 48 + ((r0 -> address.addr[2] & 0xF ) / 10 * 7); // MAC byte 4 1's digit
    sr_data[16] = (r0 -> address.addr[1] / 0x10) + 48 + ((r0 -> address.addr[1] / 0x10) / 10 * 7); // MAC byte 5 10's digit
    sr_data[17] = (r0 -> address.addr[1] & 0xF)  + 48 + ((r0 -> address.addr[1] & 0xF ) / 10 * 7); // MAC byte 5 1's digit
    sr_data[19] = (r0 -> address.addr[0] / 0x10) + 48 + ((r0 -> address.addr[0] / 0x10) / 10 * 7); // MAC byte 6 10's digit
    sr_data[20] = (r0 -> address.addr[0] & 0xF)  + 48 + ((r0 -> address.addr[0] & 0xF ) / 10 * 7); // MAC byte 6 1's digit

    // set custom scan response data (i.e. the Device Name value)
    ioteggble.ble_cmd_gap_set_adv_data(1, 0x15, sr_data);
    while (ioteggble.checkActivity(1000));

    // put module into discoverable/connectable mode (with user-defined advertisement data)
    ioteggble.ble_cmd_gap_set_mode(BGLIB_GAP_USER_DATA, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    while (ioteggble.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}

void my_ble_evt_address_status(const ble_msg_system_address_get_rsp_t *msg){
    #ifdef _DEBUG_
        pc.printf("###\tmy_ble_evt_address_status: { ");
        pc.printf(" address: ");
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) pc.putc('0');
            pc.printf("%X",msg -> address.addr[i]);
        }
        pc.printf(" }\r\n");
    #endif
}

void my_ble_evt_connection_status(const ble_msg_connection_status_evt_t *msg) {
    #ifdef _DEBUG_
        pc.printf("###\tconnection_status: { ");
        
        pc.printf("connection: "); pc.printf("%X",msg -> connection);
        pc.printf(", flags: "); pc.printf("%X",msg -> flags);
        pc.printf(", address: ");
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) pc.putc('0');
            pc.printf("%X",msg -> address.addr[i]);
        }
        pc.printf(", address_type: "); pc.printf("%X",msg -> address_type);
        pc.printf(", conn_interval: "); pc.printf("%X",msg -> conn_interval);
        pc.printf(", timeout: "); pc.printf("%X",msg -> timeout);
        pc.printf(", latency: "); pc.printf("%X",msg -> latency);
        pc.printf(", bonding: "); pc.printf("%X",msg -> bonding);
        
        pc.printf(" }\r\n");
    #endif

    // "flags" bit description:
    //  - bit 0: connection_connected
    //           Indicates the connection exists to a remote device.
    //  - bit 1: connection_encrypted
    //           Indicates the connection is encrypted.
    //  - bit 2: connection_completed
    //           Indicates that a new connection has been created.
    //  - bit 3; connection_parameters_change
    //           Indicates that connection parameters have changed, and is set
    //           when parameters change due to a link layer operation.

    // check for new connection established
    if ((msg -> flags & 0x05) == 0x05) {
        // track state change based on last known state, since we can connect two ways
        if (ble_state == BLE_STATE_ADVERTISING) {
            ble_state = BLE_STATE_CONNECTED_SLAVE;
        } else {
            ble_state = BLE_STATE_CONNECTED_MASTER;
        }
    }

    // update "encrypted" status
    ble_encrypted = msg -> flags & 0x02;
    
    // update "bonded" status
    ble_bonding = msg -> bonding;
}

void my_ble_evt_connection_disconnect(const struct ble_msg_connection_disconnected_evt_t *msg) {
    #ifdef _DEBUG_
        pc.printf("###\tconnection_disconnect: { ");
        pc.printf("connection: "); pc.printf("%X",msg -> connection);
        pc.printf(", reason: "); pc.printf("%X",msg -> reason);
        pc.printf(" } \r\n");
    #endif

    ioteggble.ble_cmd_gap_set_mode(BGLIB_GAP_USER_DATA, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    while (ioteggble.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
}

void my_ble_evt_attributes_value(const struct ble_msg_attributes_value_evt_t *msg) {
    #ifdef _DEBUG_
        pc.printf("###\tattributes_value: { ");
        pc.printf("connection: "); pc.printf("%X",msg -> connection);
        pc.printf(", reason: "); pc.printf("%X",msg -> reason);
        pc.printf(", handle: "); pc.printf("%X",msg -> handle);
        pc.printf(", offset: "); pc.printf("%X",msg -> offset);
        pc.printf(", value_len: "); pc.printf("%X",msg -> value.len);
        
        pc.printf(", value_data: ");
        // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
        for (uint8_t i = 0; i < msg -> value.len; i++) {
            if (msg -> value.data[i] < 16) pc.putc('0');
            pc.printf("%X",msg -> value.data[i]);
        }
        
        pc.printf(" }\r\n");
    #endif

    // check for data written to "c_rx_data" handle
    if (msg -> handle == GATT_HANDLE_C_RX_DATA && msg -> value.len > 0) {
        if((msg -> value.data[0] == '@')&&(msg -> value.data[(msg -> value.len)-1] == '$')){
            switch (msg -> value.data[1]){
                case 'L': // LED
                    rgbled_update(msg -> value.data[3],msg -> value.data[4],msg -> value.data[5]);
                    break;
                default:
                    break;
            }
        }
    }
}

// BLE initialization
void bleinit(){ 
    // set up internal status handlers (these are technically optional)
    BLE_RES_PIN=1;

    ioteggble.onBusy = onBusy;
    ioteggble.onIdle = onIdle;
    ioteggble.onTimeout = onTimeout;

    // set up BGLib event handlers
    ioteggble.ble_evt_system_boot = my_ble_evt_system_boot;
    ioteggble.ble_rsp_system_address_get=my_ble_evt_address_status;
    ioteggble.ble_evt_connection_status = my_ble_evt_connection_status;
    ioteggble.ble_evt_connection_disconnected = my_ble_evt_connection_disconnect;
    ioteggble.ble_evt_attributes_value = my_ble_evt_attributes_value;   

    BLE_RES_PIN=0;
    wait(0.05);
    BLE_RES_PIN=1; 
}

int main() {
    bleinit();
    wait(1.0);
    sensortimer.reset();
    sensortimer.start();
    
    while(1){
        ioteggble.checkActivity();        
        if (ble_state == BLE_STATE_CONNECTED_SLAVE) {                        
            temphumCurTimer = sensortimer.read_ms();        
            if((temphumCurTimer - temphumOldTimer) >= temphum_interval){
                temphumOldTimer = temphumCurTimer;
                eggsensor.tempvalue = temphum.sample_ctemp();
                eggsensor.humvalue = temphum.sample_humid();

                sensorbuf = (uint8_t *)malloc(12);                
                sensorbuf[0]='@';
                sensorbuf[1]='T'; // Temperature & Humidity
                sensorbuf[2]=8; // data length
                memcpy(sensorbuf+3,&eggsensor.tempvalue,sizeof(int));
                memcpy(sensorbuf+7,&eggsensor.humvalue,sizeof(int));
                sensorbuf[sensorbuf[2]+3]='$'; 
                #ifdef _DEBUG_
                pc.printf("Temperature: %.2f, Humidity: %.2f \n", eggsensor.tempvalue,eggsensor.humvalue);
                #endif
                
                ioteggble.ble_cmd_attributes_write(GATT_HANDLE_C_TX_DATA, 0, sensorbuf[2]+4 , sensorbuf);
                free(sensorbuf);
            }
        }    
    }
}
