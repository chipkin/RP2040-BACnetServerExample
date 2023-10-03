/**
 * RP2040 BACnet Server Example 
 * --------------------------------------
 * In this project we are using the CAS BACnet stack (https://www.bacnetstack.com/) to generate a 
 * simple BACnet server running on a Raspberry Pi Pico W. The application contains one Multi-state-value (MSV) object
 * for the LED status and an analog input for the internal cpu temperature sensor. 
 * A BACnet client (such as the CAS BACnet Explorer) can be used to write to the 
 * MSV to change the LED mode and modify the temperature unit (celsius or fahrenheit). 
 * 
 * In this example the MSV has four possible values 
 * - 1 = Off
 * - 2 = On
 * - 3 = Blink
 * - 4 = Blink
 * 
 * Created by: Justin Chang
 * Created on: October 3rd, 2023
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <CASBACnetStackAdapter.h>
#include <CIBuildSettings.h>
#include "CASBACnetStackExampleConstants.h"

// Application Version
// -----------------------------
const char* APPLICATION_VERSION = "0.0.1";  // See CHANGELOG.md for a full list of changes.

// Application Settings
// =======================================
// Wifi credentials
const char WIFI_SSID[] = "";
const char WIFI_PASSWORD[] = "";

// Device Settings
const uint32_t APPLICATION_BACNET_DEVICE_INSTANCE = 389001;
const char* APPLICATION_BACNET_DEVICE_OBJECT_NAME = "RP2040 BACnet Server Example";
const char* APPLICATION_BACNET_DEVICE_DESCRIPTION = "Running CAS BACnet Stack. Project can be found at http://github.com/chipkin/RP2040-BACnetServerExample";

// Analog input
const uint32_t APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE = 2;
const char* APPLICATION_BACNET_OBJECT_ANALOG_INPUT_OBJECT_NAME = "Internal Temperature Sensor";
const char* APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION = "CPU temp";
const float CONVERSION_FACTOR = 3.3f / (1 << 12);  // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
uint32_t temperature_unit = CASBACnetStackExampleConstants::TEMPERATURE_ENUM_DEGREES_CELSIUS;

// Multi-state variable
const uint32_t APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE = 4;
const char* APPLICATION_BACNET_OBJECT_MSV_LED_OBJECT_NAME = "LED State";
const char* APPLICATION_BACNET_OBJECT_MSV_LED_STATE_TEXT[] = { "Off", "On", "Blink", "Fast Blink"};

// Network port
const uint32_t APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE = 4;
const char* APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE_OBJECT_NAME = "Network port";

// LED
// -----------------------------
// Time
const long APPLICATION_LED_BLINK_RATE_MS = 1000; 
const long APPLICATION_LED_FAST_BLINK_RATE_MS = 250; 
// Modes
const uint16_t LED_MODE_OFF = 1;
const uint16_t LED_MODE_ON = 2;
const uint16_t LED_MODE_BLINK = 3;
const uint16_t LED_MODE_FAST_BLINK = 4;
uint32_t gLEDMode = LED_MODE_BLINK;
bool     gLEDModeFlag = 0; // flag set when value is changed
const uint16_t LED_MODE_STATE_COUNT = LED_MODE_FAST_BLINK;
// LED blinking
struct repeating_timer blink_timer;
bool  LEDState = 0; // blinking (on/off)

// Server settings
#define APPLICATION_BACNET_UDP_PORT 47808
#define IPV4_ADDR_LENGTH 4

// UDP packet
// -----------------------------
// UDP packet setup
#define MAX_PAYLOAD_SIZE 500
#define PACKET_QUEUE_SIZE 20
struct udp_packet {
    uint8_t src_addr[4] = {0, 0, 0, 0};
    uint16_t port = 0;
    uint16_t length = 0;
    char* payload[MAX_PAYLOAD_SIZE] = {0};
} udp_packet;

// Global variables
struct udp_pcb  * upcb; // udp socket
struct udp_packet packet_queue[PACKET_QUEUE_SIZE];
int8_t packet_queue_front = -1;
int8_t packet_queue_rear = -1;

// Callback functions
// -----------------------------
// Messages
uint16_t CallbackReceiveMessage(uint8_t* message, const uint16_t maxMessageLength, uint8_t* receivedConnectionString, const uint8_t maxConnectionStringLength, uint8_t* receivedConnectionStringLength, uint8_t* networkType);
uint16_t CallbackSendMessage(const uint8_t* message, const uint16_t messageLength, const uint8_t* connectionString, const uint8_t connectionStringLength, const uint8_t networkType, bool broadcast);
// Get property
time_t CallbackGetSystemTime();
bool CallbackGetPropertyCharString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, char* value, uint32_t* valueElementCount, const uint32_t maxElementCount, uint8_t* encodingType, const bool useArrayIndex, const uint32_t propertyArrayIndex);
bool CallbackGetPropertyUInt(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t* value, bool useArrayIndex, uint32_t propertyArrayIndex);
bool CallbackGetPropertyReal(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, float* value, bool useArrayIndex, uint32_t propertyArrayIndex);
bool CallbackGetPropertyEnum(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t* value, bool useArrayIndex, uint32_t propertyArrayIndex);
// Set property
bool CallbackSetPropertyUInt(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t* errorCode);
bool CallbackSetPropertyEnum(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t* errorCode);
// Remote device management
bool CallbackReinitializeDevice(const uint32_t deviceInstance, const uint32_t reinitializedState, const char* password, const uint32_t passwordLength, uint32_t* errorCode);

// Helpers
// -----------------------------
// General
bool get_broadcast_address(uint8_t* broadcastAddress, size_t maxbroadcastAddressSize);
bool example_ntoa(uint8_t* addressBuffer, size_t maxBufferSize, unsigned long ipAddress);
bool blink_led(struct repeating_timer *t);
void update_led_status();

// UDP
void send_udp(char * IP , int port, const void * data, int data_size);
void receive_udp_callback(void * arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t*addr,u16_t port);
bool get_udp_packet(struct udp_packet* packet);
bool enqueue_udp_packet(struct udp_packet* packet);


int main() {
    // 1. Hardware setup
	// ==================================================================================
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Failed to initialize cyw43\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
	cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);

    // Print application information
    printf("FYI: RP2040 BACnet Server Example: v%s\n", APPLICATION_VERSION);
    // TODO: update when github repo
    printf("https://github.com/chipkin/RP2040-BACnetServerExample\n");
    
    // Connect to wi-fi - loop until connected
    while(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0){
        printf("Attempting to connect to %s...\n", WIFI_SSID);
    }
    printf("Connected!\n");

    // Setup protocol control block
    upcb = udp_new();
    
    // Bind pcb to BACnet port
    err_t err2 = udp_bind(upcb, IP_ADDR_ANY, APPLICATION_BACNET_UDP_PORT);
    printf("Binded to UDP socket %d\n", APPLICATION_BACNET_UDP_PORT);

    // Setup UDP receive message callback function
    udp_recv(upcb, receive_udp_callback, NULL);

    // Add timer for LED
    add_repeating_timer_ms(APPLICATION_LED_BLINK_RATE_MS, blink_led, NULL, &blink_timer);

    // Setup temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // 4 = internal cpu temp

	// 2. Load the CAS BACnet stack functions
	// ==================================================================================
	if (!LoadBACnetFunctions()) {
		printf("ERROR: Failed to load stack functions");
		return 0;
	}
    printf("FYI: CAS BACnet Stack version: %d.%d.%d.%d\n", fpGetAPIMajorVersion(), fpGetAPIMinorVersion(), fpGetAPIPatchVersion(), fpGetAPIBuildVersion());


    // 3. Register callback functions
	// ==================================================================================
    // Message 
    fpRegisterCallbackReceiveMessage(CallbackReceiveMessage);
    fpRegisterCallbackSendMessage(CallbackSendMessage);

    // Get property 
    fpRegisterCallbackGetPropertyCharacterString(CallbackGetPropertyCharString);
    fpRegisterCallbackGetPropertyUnsignedInteger(CallbackGetPropertyUInt);
    fpRegisterCallbackGetPropertyReal(CallbackGetPropertyReal);
    fpRegisterCallbackGetPropertyEnumerated(CallbackGetPropertyEnum);

    // Set property 
    fpRegisterCallbackSetPropertyUnsignedInteger(CallbackSetPropertyUInt);
    fpRegisterCallbackSetPropertyEnumerated(CallbackSetPropertyEnum);

    // Remote device management
    fpRegisterCallbackReinitializeDevice(CallbackReinitializeDevice);

    // 4. Setup the BACnet device
    // ==================================================================================
    // Setup device object
    if (!fpAddDevice(APPLICATION_BACNET_DEVICE_INSTANCE)) {
        printf("Error: Could not add device. Device instanse=%u\n", APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: BACnet device created: Device instance=%u\n", APPLICATION_BACNET_DEVICE_INSTANCE);

    // Enable Optional Device Properties
    // Description
	if (!fpSetPropertyEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_DEVICE, APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION, true)) {
		printf("Failed to enable the description property for Device");
		return 0;
	}

    // Application software version
    if (!fpSetPropertyEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_DEVICE, APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_APPLICATION_SOFTWARE_VERSION, true)) {
		printf("Failed to enable the application software property for Device");
		return 0;
	}

    // Enable services
    // By default the write service is not enabled. We have to enable it to allow users to write to points.
    if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_WRITE_PROPERTY, true)) {
        printf("Error: Failed to enabled the WriteProperty service=[%u] for Device %u\n", CASBACnetStackExampleConstants::SERVICE_WRITE_PROPERTY, APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: Enabled WriteProperty for Device %u\n", APPLICATION_BACNET_DEVICE_INSTANCE);

    // Read Property Multiple service is a nice to have, not required for a BACnet server to work but it does make polling the device easier.
    if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_READ_PROPERTY_MULTIPLE, true)) {
        printf("Error: Failed to enabled the Read Property Multiple service=[%u] for Device %u\n", CASBACnetStackExampleConstants::SERVICE_READ_PROPERTY_MULTIPLE, APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: Enabled Read Property Multiple for Device %u\n", CASBACnetStackExampleConstants::SERVICE_READ_PROPERTY_MULTIPLE);

    printf("Enabling SubscribeCOV... ");
	if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_SUBSCRIBE_COV, true)) {
		printf("Failed to enable the SubscribeCOV service\n");
		return false;
	}
	printf("OK");

	printf("Enabling SubscribeCOVProperty... ");
	if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_SUBSCRIBE_COV_PROPERTY, true)) {
		printf("Failed to enable the SubscribeCOVProperty service\n");
		return false;
	}
	printf("OK");


    // Add objects
    // Analog Input
    printf("Adding AnalogInput. analogInput.instance=[%d]...", APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE);
	if (!fpAddObject(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE)) {
		printf("Failed to add AnalogInput\n");
		return 0;
	}

    fpSetPropertyWritable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_UNITS, true);
    fpSetPropertySubscribable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE, true);
    fpSetPropertyByObjectTypeEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION, true);

    // Multi-State Value
    if (!fpAddObject(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE, APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE)) {
        printf("Error: Failed to add multi-state-output (%u) to Device (%u)\n", APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE, APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    fpSetPropertyWritable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE, APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE, true);
    fpSetPropertyEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE, APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_STATE_TEXT, true);
    
    // Network Port
	if (!fpAddNetworkPortObject(APPLICATION_BACNET_DEVICE_INSTANCE, APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE, CASBACnetStackExampleConstants::NETWORK_TYPE_IPV4, CASBACnetStackExampleConstants::PROTOCOL_LEVEL_BACNET_APPLICATION, CASBACnetStackExampleConstants::NETWORK_PORT_LOWEST_PROTOCOL_LAYER)) {
		printf("Failed to add NetworkPort object\n");
		return 0;
	}
    printf("Added NetworkPort. networkPort.instance=[%s]", APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE);
    

    // 5. Send I-Am of this device
	// ==================================================================================
    uint8_t connectionString[6];
    if (!get_broadcast_address(connectionString, 6)) {
        printf("Error: Could not get the broadcast IP address\n");
        return 0;
    }
    connectionString[4] = APPLICATION_BACNET_UDP_PORT / 256;
    connectionString[5] = APPLICATION_BACNET_UDP_PORT % 256;

    // Send a IAm Message to announce to the network that a new BACnet device has started.
    if (!fpSendIAm(APPLICATION_BACNET_DEVICE_INSTANCE, connectionString, 6, CASBACnetStackExampleConstants::NETWORK_TYPE_BACNET_IP, true, 65535, NULL, 0)) {
        printf("Error: Unable to send IAm for Device %u", APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: Sent broadcast IAm message\n");

    // 6. Main Loop
	// ==================================================================================
    printf("FYI: Entering main loop\n");
    for (;;) {
        fpLoop();
        cyw43_arch_poll();
        update_led_status();
        sleep_ms(0);
    }
    return 0;
}

// Helper functions
// ==============================================================================
bool get_broadcast_address(uint8_t* broadcastAddress, size_t maxbroadcastAddressSize)
{
    if (broadcastAddress == NULL || maxbroadcastAddressSize < 4) {
        return false; // Not enough space
    }
    uint8_t localIP[IPV4_ADDR_LENGTH];
    uint8_t subnetMask[IPV4_ADDR_LENGTH];
    if (
        !example_ntoa(localIP, IPV4_ADDR_LENGTH, cyw43_state.netif[0].ip_addr.addr) ||
        !example_ntoa(subnetMask, IPV4_ADDR_LENGTH, cyw43_state.netif[0].netmask.addr)) return false;

    broadcastAddress[0] = localIP[0] | ~subnetMask[0];
    broadcastAddress[1] = localIP[1] | ~subnetMask[1];
    broadcastAddress[2] = localIP[2] | ~subnetMask[2];
    broadcastAddress[3] = localIP[3] | ~subnetMask[3];
    return true;
}

// https://forums.raspberrypi.com/viewtopic.php?t=340208
// Send UDP message
void send_udp(char * IP , int port, const void * data, int data_size)
{
      ip_addr_t   destAddr;
      ip4addr_aton(IP,&destAddr);
      struct pbuf * p = pbuf_alloc(PBUF_TRANSPORT,data_size,PBUF_RAM);
      memcpy(p->payload,data,data_size);
      cyw43_arch_lwip_begin();
      udp_sendto(upcb,p,&destAddr,port);
      cyw43_arch_lwip_end();
      pbuf_free(p);
}

// Receive UDP message callback
void receive_udp_callback(void * arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t*addr,u16_t port)
{
    struct udp_packet packet;
    packet.src_addr[0] = addr->addr&0xff;
    packet.src_addr[1] = (addr->addr>>8)&0xff;
    packet.src_addr[2] = (addr->addr>>16)&0xff;
    packet.src_addr[3] = addr->addr>>24;
    packet.port = port;
    packet.length = p->len;
    memcpy(packet.payload, p->payload, p->len);

    enqueue_udp_packet(&packet);
    pbuf_free(p); 
}

// Convert unsigned long address into int array
bool example_ntoa(uint8_t* addressBuffer, size_t maxBufferSize, unsigned long ipAddress) {
    if (addressBuffer == NULL || maxBufferSize < 4) {
        return false; // not enough space
    }
    addressBuffer[0] = ipAddress & 0xFF;
    addressBuffer[1] = (ipAddress >> 8) & 0xFF;
    addressBuffer[2] = (ipAddress >> 16) & 0xFF;
    addressBuffer[3] = (ipAddress >> 24) & 0xFF;

    return true;
}

// Enqueue received UDP packet into packet queue
// Returns false if packet_queue is full
bool enqueue_udp_packet(struct udp_packet* packet)
{
    // Packet queue is full
    if (packet_queue_rear == PACKET_QUEUE_SIZE - 1)
    {
        return false;
    }

    if (packet_queue_front == -1) packet_queue_front = 0;

    packet_queue_rear = packet_queue_rear + 1;
    packet_queue[packet_queue_rear] = *packet;
    return true;

} 

// Dequeue front of packet queue (if available)
// Returns false if no packet available to dequeue
bool get_udp_packet(struct udp_packet* packet)
{
    // no packet is available in queue
    if (packet_queue_front == -1 || packet_queue_front > packet_queue_rear)
    {
        return false;
    }

    // packet available
    *packet = packet_queue[packet_queue_front];
    packet_queue_front = packet_queue_front + 1;

    if (packet_queue_front > packet_queue_rear) {
        packet_queue_front = packet_queue_rear = -1;
    }
    return true;
} 

// handle LED blinking
bool blink_led(struct repeating_timer *t) {
    LEDState = !LEDState;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LEDState);
    return true;
}

// Change the timer interrupt of the LED
void update_led_status() {
    if (gLEDModeFlag) {
        cancel_repeating_timer(&blink_timer);
        gLEDModeFlag = 0;
        switch(gLEDMode) {
            case LED_MODE_OFF:   
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            break;
            case LED_MODE_ON:    
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            break;
            case LED_MODE_BLINK:        
            add_repeating_timer_ms(APPLICATION_LED_BLINK_RATE_MS, blink_led, NULL, &blink_timer);
            break;
            case LED_MODE_FAST_BLINK:   
            add_repeating_timer_ms(APPLICATION_LED_FAST_BLINK_RATE_MS, blink_led, NULL, &blink_timer);
            default:
            break;
        }
    }
}

// Callback Functions
// ==============================================================================
uint16_t CallbackReceiveMessage(uint8_t* message, const uint16_t maxMessageLength, uint8_t* receivedConnectionString, const uint8_t maxConnectionStringLength, uint8_t* receivedConnectionStringLength, uint8_t* networkType)
{
    // Check parameters
    if (message == NULL || maxMessageLength == 0) {
        printf("Error: Invalid input buffer");
        return 0;
    }
    if (receivedConnectionString == NULL || maxConnectionStringLength == 0) {
        printf("Error: Invalid connection string buffer");
        return 0;
    }
    if (maxConnectionStringLength < 6) {
        printf("Error: Not enough space for a UDP connection string");
        return 0;
    }

    // processing incoming packet, must be called before reading the buffer
    struct udp_packet packet;
    if (!get_udp_packet(&packet)) return 0;

    // We got a message.
    receivedConnectionString[0] = packet.src_addr[0];
    receivedConnectionString[1] = packet.src_addr[1];
    receivedConnectionString[2] = packet.src_addr[2];
    receivedConnectionString[3] = packet.src_addr[3];
    receivedConnectionString[4] = (packet.port >> 8) & 0xFF;
    receivedConnectionString[5] = packet.port;
    memcpy(message, (uint8_t*)packet.payload ,packet.length);

    *receivedConnectionStringLength = 6;
    *networkType = CASBACnetStackExampleConstants::NETWORK_TYPE_BACNET_IP;

    printf("FYI: Recived message with %u bytes from %u.%u.%u.%u:%u\n", packet.length, receivedConnectionString[0], receivedConnectionString[1], receivedConnectionString[2], receivedConnectionString[3], packet.port);
    return packet.length;
}

uint16_t CallbackSendMessage(const uint8_t* message, const uint16_t messageLength, const uint8_t* connectionString, const uint8_t connectionStringLength, const uint8_t networkType, bool broadcast)
{
    if (message == NULL || messageLength == 0) {
        printf("FYI: Nothing to send");
        return 0;
    }
    if (connectionString == NULL || connectionStringLength == 0) {
        printf("FYI: No connection string");
        return 0;
    }

    // Verify Network Type
    if (networkType != CASBACnetStackExampleConstants::NETWORK_TYPE_BACNET_IP) {
        printf("FYI: Message for different network");
        return 0;
    }

    // Prepare the IP Address
    char destinationIPAddressAsString[32];
    if (broadcast) {
        uint8_t broadcastIPAddress[4] = {255, 255, 255, 255};
        if (!get_broadcast_address(broadcastIPAddress, 4)) {
            printf("Error: Could not get the broadcast iIP address");
            return 0;
        }
        snprintf(destinationIPAddressAsString, 32, "%u.%u.%u.%u", broadcastIPAddress[0], broadcastIPAddress[1], broadcastIPAddress[2], broadcastIPAddress[3]);
    } else {
        snprintf(destinationIPAddressAsString, 32, "%u.%u.%u.%u", connectionString[0], connectionString[1], connectionString[2], connectionString[3]);
    }

    // Get the port
    uint16_t udpPort = 0;
    udpPort += connectionString[4] * 256;
    udpPort += connectionString[5];

    // Send
    send_udp(destinationIPAddressAsString, udpPort, message, messageLength);

    printf("FYI: Sent message with %u bytes to %s:%u\n", messageLength, destinationIPAddressAsString, udpPort);
    return messageLength;
}

// GetProperty Callbacks
// ==============================================================================
bool CallbackGetPropertyCharString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, char* value, uint32_t* valueElementCount, const uint32_t maxElementCount, uint8_t* encodingType, const bool useArrayIndex, const uint32_t propertyArrayIndex)
{
    printf("FYI: CallbackGetPropertyCharString deviceInstance=%d, objectType=%d, objectInstance=%d, propertyIdentifier=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier);
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE) {
        return false;
    }

    // Device properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_DEVICE && objectInstance == APPLICATION_BACNET_DEVICE_INSTANCE) {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION) {
            if (strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION) <= maxElementCount) {
                memcpy(value, APPLICATION_BACNET_DEVICE_DESCRIPTION, strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION));
                *valueElementCount = (uint32_t)strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION);
                return true;
            }
            return false;
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_APPLICATION_SOFTWARE_VERSION) {
            if (strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION) <= maxElementCount) {
                memcpy(value, APPLICATION_VERSION, strlen(APPLICATION_VERSION));
                *valueElementCount = (uint32_t)strlen(APPLICATION_VERSION);
                return true;
            }
            return false;
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false) {
        size_t objectNameStringLength = strlen(APPLICATION_BACNET_DEVICE_OBJECT_NAME);
        if (objectNameStringLength <= maxElementCount) {
            memcpy(value, APPLICATION_BACNET_DEVICE_OBJECT_NAME, objectNameStringLength);
            *valueElementCount = objectNameStringLength;
            return true;
        }
    }
    }

    // Analog-input object properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE) {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false) {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount) {
                memcpy(value, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION) {
            if (strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION) <= maxElementCount) {
                memcpy(value, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION, strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION));
                *valueElementCount = (uint32_t)strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION);
                return true;
            }
        }
		return false;
	}

    // Multi-state value object properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE && objectInstance == APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE) {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false) {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_MSV_LED_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount) {
                memcpy(value, APPLICATION_BACNET_OBJECT_MSV_LED_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        } else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_STATE_TEXT && useArrayIndex == true) {
            if (propertyArrayIndex > LED_MODE_STATE_COUNT || propertyArrayIndex <= 0) {
                return false; // Invalid range
            }

            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_MSV_LED_STATE_TEXT[propertyArrayIndex - 1]);
            if (objectNameStringLength <= maxElementCount) {
                memcpy(value, APPLICATION_BACNET_OBJECT_MSV_LED_STATE_TEXT[propertyArrayIndex - 1], objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
    } 

    // Network port object properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE) {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false) {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount) {
                memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
		return false;
	}

    return false;
}

bool CallbackGetPropertyUInt(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t* value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
    printf("FYI: CallbackGetPropertyUInt deviceInstance=%d, objectType=%d, objectInstance=%d, propertyIdentifier=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier);

    if (deviceInstance == APPLICATION_BACNET_DEVICE_INSTANCE && objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE && objectInstance == APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE) {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE && useArrayIndex == false) {
            *value = gLEDMode;
            return true;
        } else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_NUMBER_OF_STATES && useArrayIndex == false) {
            *value = LED_MODE_STATE_COUNT;
            return true;
        } else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_STATE_TEXT && useArrayIndex == true && propertyArrayIndex == 0) {
            *value = LED_MODE_STATE_COUNT;
            return true;
        }
    }

    return false;
}

// Callback used by the BACnet Stack to get Real property values from the user
bool CallbackGetPropertyReal(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, float* value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
	// Example of Analog Input / Value Object Present Value property
	if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE) {
		if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE) {
			uint16_t adc = adc_read();
            float ADC_Voltage = float(adc) * CONVERSION_FACTOR;
            *value = 27 - (ADC_Voltage - 0.706) / 0.001721; 
            if (temperature_unit == CASBACnetStackExampleConstants::TEMPERATURE_ENUM_DEGREES_FAHRENHEIT) *value = *value * 9/5 + 32;
			return true;
		}
	}
	return false;
}

// Callback used by the BACnet Stack to get Enumerated property values from the user
bool CallbackGetPropertyEnum(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t* value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
	printf("CallbackGetPropertyEnum deviceInstance=%d, , objectType=%d, objectInstance=%d, propertyIdentifier=%d useArrayIndex=%d, propertyArrayIndex=%d\n",deviceInstance, objectType, objectInstance, propertyIdentifier, useArrayIndex, propertyArrayIndex); 

	// Example of Analog Input / Value Object Present Value property
	if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_UNITS) {
		if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE) {
			*value = temperature_unit; 
			return true;
		}
	}
    
	// We could not answer this request. 
	return false;
}

// SetProperty Callbacks
// ==============================================================================
// Callback used by the BACnet Stack to set Unsigned Int property values to the user
bool CallbackSetPropertyUInt(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t* errorCode)
{
    printf("FYI: CallbackSetPropertyUInt deviceInstance=%d, objectType=%d, objectInstance=%d, propertyIdentifier=%d, value=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier, value);

    if (deviceInstance == APPLICATION_BACNET_DEVICE_INSTANCE && objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE && objectInstance == APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE && propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE && useArrayIndex == false) {
        switch (value) {
            case LED_MODE_OFF:
            case LED_MODE_ON:
            case LED_MODE_BLINK:
            case LED_MODE_FAST_BLINK:
                gLEDMode = value;
                gLEDModeFlag = 1;
                printf("SET LED MODE TO %d", gLEDMode);
                return true;
            default:
                // Out of range
                *errorCode = CASBACnetStackExampleConstants::ERROR_VALUE_OUT_OF_RANGE;
                return false;
        }
    }

    return false;
}

// Callback used by the BACnet Stack to set Enumerated property values to the user
bool CallbackSetPropertyEnum(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t* errorCode)
{
	if (deviceInstance == APPLICATION_BACNET_DEVICE_INSTANCE) {
		if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT) {
			// Setting temperature units in temperature sensor analog input
			if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_UNITS && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE) {
				switch (value) {
                    case CASBACnetStackExampleConstants::TEMPERATURE_ENUM_DEGREES_CELSIUS:
                    case CASBACnetStackExampleConstants::TEMPERATURE_ENUM_DEGREES_FAHRENHEIT:
                        temperature_unit = value;
                        return true;
                    default:
                        // Out of range
                        *errorCode = CASBACnetStackExampleConstants::ERROR_VALUE_OUT_OF_RANGE;
                        return false;
                }
			}
		}
	}
	return false;
}

// Other Callbacks
// ==============================================================================

bool CallbackReinitializeDevice(const uint32_t deviceInstance, const uint32_t reinitializedState, const char* password, const uint32_t passwordLength, uint32_t* errorCode) {
	// This callback is called when this BACnet Server device receives a ReinitializeDevice message
	// In this callback, you will handle the reinitializedState.
	// If reinitializedState = ACTIVATE_CHANGES (7) then you will apply any network port changes and store the values in non-volatile memory
	// If reinitializedState = WARM_START(1) then you will apply any network port changes, store the values in non-volatile memory, and restart the device.

	// Before handling the reinitializedState, first check the password.
	// If your device does not require a password, then ignore any password passed in.
	// Otherwise, validate the password.
	//		If password invalid, missing, or incorrect: set errorCode to PasswordInvalid (26)
	// In this example, a password of 12345 is required.
	
	if (password == NULL || passwordLength == 0) {
		*errorCode = CASBACnetStackExampleConstants::ERROR_PASSWORD_FAILURE;
		return false;
	}

	if (strcmp(password, "12345") != 0) {
		*errorCode = CASBACnetStackExampleConstants::ERROR_PASSWORD_FAILURE;
		return false;
	}

	// In this example, only the NetworkPort Object FdBbmdAddress and FdSubscriptionLifetime properties are writable and need to be
	// stored in non-volatile memory.  For the purpose of this example, we will not storing these values in non-volaitle memory.

	// 1. Store values that must be stored in non-volatile memory (i.e. must survive a reboot).

	// 2. Apply any Network Port values that have been written to. 
	// If any validation on the Network Port values failes, set errorCode to INVALID_CONFIGURATION_DATA (46)

	// 3. Set Network Port ChangesPending property to false

	// 4. Handle ReinitializedState. If ACTIVATE_CHANGES, no other action, return true.
	//								 If WARM_START, prepare device for reboot, return true. and reboot.  
	// NOTE: Must return true first before rebooting so the stack sends the SimpleAck.
	if (reinitializedState == CASBACnetStackExampleConstants::REINITIALIZED_STATE_ACTIVATE_CHANGES) {
		//g_database.networkPort.ChangesPending = false;
		return true;
	}
	else if (reinitializedState == CASBACnetStackExampleConstants::REINITIALIZED_STATE_WARM_START) {
		// Flag for reboot and handle reboot after stack responds with SimpleAck.
		//g_database.networkPort.ChangesPending = false;
		return true;
	}
	else {
		// All other states are not supported in this example.
		*errorCode = CASBACnetStackExampleConstants::ERROR_OPTIONAL_FUNCTIONALITY_NOT_SUPPORTED;
		return false;
	}
}
/*
time_t CallbackGetSystemTime()
{
    return millis() / 1000;
}
*/