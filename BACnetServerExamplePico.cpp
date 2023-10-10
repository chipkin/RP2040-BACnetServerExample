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
 * Created on: October 10th, 2023
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/dns.h"

#include <CASBACnetStackAdapter.h>
#include <CIBuildSettings.h>
#include "CASBACnetStackExampleConstants.h"

// Application Version
// -----------------------------
const char *APPLICATION_VERSION = "0.0.1"; // See CHANGELOG.md for a full list of changes.

// Server settings
#define APPLICATION_BACNET_UDP_PORT 47808
#define IPV4_ADDR_LENGTH 4
#define INTERNAL_TEMPERATURE_SENSOR_PIN 4
#define DNS_SERVER_MAXSIZE 5

// Application Settings
// =======================================
// Wifi credentials
const char WIFI_SSID[] = "TODO_WIFI_SSID";
const char WIFI_PASSWORD[] = "TODO_WIFI_PASSWORD";

// Device Settings
const uint32_t APPLICATION_BACNET_DEVICE_INSTANCE = 389007;
const char *APPLICATION_BACNET_DEVICE_OBJECT_NAME = "Chipkin RP2040 BACnet Server Example";
const char *APPLICATION_BACNET_DEVICE_DESCRIPTION = "Running CAS BACnet Stack. Project can be found at http://github.com/chipkin/RP2040-BACnetServerExample";

// Analog input
const uint32_t APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE = 2;
const char *APPLICATION_BACNET_OBJECT_ANALOG_INPUT_OBJECT_NAME = "Internal Temperature Sensor";
const char *APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION = "CPU temp";
const float CONVERSION_FACTOR = 3.3f / (1 << 12); // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
uint32_t temperature_unit = CASBACnetStackExampleConstants::TEMPERATURE_ENUM_DEGREES_CELSIUS;

// Multi-state variable
const uint32_t APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE = 4;
const char *APPLICATION_BACNET_OBJECT_MSV_LED_OBJECT_NAME = "LED State";
const char *APPLICATION_BACNET_OBJECT_MSV_LED_STATE_TEXT[] = {"Off", "On", "Blink", "Fast Blink"};

// Network port
const uint32_t APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE = 0;
const char *APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE_OBJECT_NAME = "WiFi";
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[IPV4_ADDR_LENGTH];
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[IPV4_ADDR_LENGTH];
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[IPV4_ADDR_LENGTH];
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_GATEWAY[IPV4_ADDR_LENGTH];
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER[DNS_SERVER_MAXSIZE][IPV4_ADDR_LENGTH];
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER_SIZE = 0;
bool APPLICATION_BACNET_OBJECT_NETWORKPORT_CHANGESPENDING = 0;

// BBMD
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDHOSTTYPE = 0;
uint8_t APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDHOSTIP[IPV4_ADDR_LENGTH];
uint16_t APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDPORT;
uint16_t APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDSUBSCRIPTIONLIFETIME;

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
bool gLEDModeFlag = 0; // flag set when value is changed
const uint16_t LED_MODE_STATE_COUNT = LED_MODE_FAST_BLINK;
// LED blinking
struct repeating_timer blink_timer;
bool LEDState = 0; // blinking (on/off)

// UDP packet
// -----------------------------
// UDP packet setup
#define MAX_PAYLOAD_SIZE 1497 // max NPDU length (bytes)
#define PACKET_QUEUE_MAXSIZE 20
struct udp_packet
{
    uint8_t src_addr[4] = {0, 0, 0, 0};
    uint16_t port = 0;
    uint16_t length = 0;
    char *payload[MAX_PAYLOAD_SIZE] = {0};
} udp_packet;

// Global variables
struct udp_pcb *upcb; // udp socket
struct udp_packet packet_queue[PACKET_QUEUE_MAXSIZE];
int8_t packet_queue_front = 0;
int8_t packet_queue_rear = PACKET_QUEUE_MAXSIZE - 1;
int8_t packet_queue_size = 0;

// Callback functions
// -----------------------------
// System
time_t CallbackGetSystemTime();
// Messages
uint16_t CallbackReceiveMessage(uint8_t *message, const uint16_t maxMessageLength, uint8_t *receivedConnectionString, const uint8_t maxConnectionStringLength, uint8_t *receivedConnectionStringLength, uint8_t *networkType);
uint16_t CallbackSendMessage(const uint8_t *message, const uint16_t messageLength, const uint8_t *connectionString, const uint8_t connectionStringLength, const uint8_t networkType, bool broadcast);
// Get property
time_t CallbackGetSystemTime();
bool CallbackGetPropertyCharString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, char *value, uint32_t *valueElementCount, const uint32_t maxElementCount, uint8_t *encodingType, const bool useArrayIndex, const uint32_t propertyArrayIndex);
bool CallbackGetPropertyUInt(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t *value, bool useArrayIndex, uint32_t propertyArrayIndex);
bool CallbackGetPropertyReal(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, float *value, bool useArrayIndex, uint32_t propertyArrayIndex);
bool CallbackGetPropertyEnum(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t *value, bool useArrayIndex, uint32_t propertyArrayIndex);
bool CallbackGetPropertyOctetString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, uint8_t *value, uint32_t *valueElementCount, const uint32_t maxElementCount, const bool useArrayIndex, const uint32_t propertyArrayIndex);
bool CallbackGetPropertyBool(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, bool *value, const bool useArrayIndex, const uint32_t propertyArrayIndex);
// Set property
bool CallbackSetPropertyUInt(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t *errorCode);
bool CallbackSetPropertyEnum(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t *errorCode);
bool CallbackSetPropertyOctetString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint8_t *value, const uint32_t length, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t *errorCode);
// Remote device management
bool CallbackReinitializeDevice(const uint32_t deviceInstance, const uint32_t reinitializedState, const char *password, const uint32_t passwordLength, uint32_t *errorCode);

// Helpers
// -----------------------------
// General
bool get_broadcast_address(uint8_t *broadcastAddress, size_t maxbroadcastAddressSize);
bool example_ntoa(uint8_t *addressBuffer, size_t maxBufferSize, unsigned long ipAddress);
bool blink_led(struct repeating_timer *t);
void update_led_status();

// UDP
// Send a UDP packet
void send_udp(char *IP, int port, const void *data, int data_size);

// UDP packet receive callback
void receive_udp_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

// Dequeues a UDP packet from packet queue
// Returns true if packet is available
bool get_udp_packet(struct udp_packet *packet);

// Enqueues a UDP packet into packet queue
bool enqueue_udp_packet(struct udp_packet *packet);

// Sets up network port object and properties
bool setup_network_port();

int main()
{
    // 1. Hardware setup
    // ==================================================================================
    stdio_init_all();

    // Print application information
    printf("FYI: Chipkin RP2040 BACnet Server Example: v%s\n", APPLICATION_VERSION);
    // TODO: update when github repo
    printf("https://github.com/chipkin/RP2040-BACnetServerExample\n");

    if (cyw43_arch_init())
    {
        printf("Failed to initialize cyw43\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);

    // Connect to wi-fi - loop until connected
    printf("FYI: Attempting to connect to wifi %s\n", WIFI_SSID);
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
    {
        printf("Attempting to connect to %s...\n", WIFI_SSID);
    }
    printf("FYI: Connected!\n");

    // Setup protocol control block
    upcb = udp_new();

    // Bind pcb to BACnet port
    err_t udp_error = udp_bind(upcb, IP_ADDR_ANY, APPLICATION_BACNET_UDP_PORT);
    if (udp_error != ERR_OK)
    {
        printf("Failed to bind to UDP socket%d\n", APPLICATION_BACNET_UDP_PORT);
        return 0;
    }
    printf("FYI: Binded to UDP socket %d\n", APPLICATION_BACNET_UDP_PORT);

    // Setup UDP receive message callback function
    udp_recv(upcb, receive_udp_callback, NULL);

    // Add timer for LED
    add_repeating_timer_ms(APPLICATION_LED_BLINK_RATE_MS, blink_led, NULL, &blink_timer);

    // Setup temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(INTERNAL_TEMPERATURE_SENSOR_PIN); // 4 = internal cpu temp

    // 2. Load the CAS BACnet stack functions
    // ==================================================================================
    if (!LoadBACnetFunctions())
    {
        printf("ERROR: Failed to load stack functions");
        return 0;
    }
    printf("FYI: CAS BACnet Stack version: %d.%d.%d.%d\n", fpGetAPIMajorVersion(), fpGetAPIMinorVersion(), fpGetAPIPatchVersion(), fpGetAPIBuildVersion());

    // 3. Register callback functions
    // ==================================================================================
    // System
    fpRegisterCallbackGetSystemTime(CallbackGetSystemTime);

    // Message
    fpRegisterCallbackReceiveMessage(CallbackReceiveMessage);
    fpRegisterCallbackSendMessage(CallbackSendMessage);

    // Get property
    fpRegisterCallbackGetPropertyCharacterString(CallbackGetPropertyCharString);
    fpRegisterCallbackGetPropertyUnsignedInteger(CallbackGetPropertyUInt);
    fpRegisterCallbackGetPropertyReal(CallbackGetPropertyReal);
    fpRegisterCallbackGetPropertyEnumerated(CallbackGetPropertyEnum);
    fpRegisterCallbackGetPropertyOctetString(CallbackGetPropertyOctetString);
    fpRegisterCallbackGetPropertyBool(CallbackGetPropertyBool);

    // Set property
    fpRegisterCallbackSetPropertyUnsignedInteger(CallbackSetPropertyUInt);
    fpRegisterCallbackSetPropertyEnumerated(CallbackSetPropertyEnum);
    fpRegisterCallbackSetPropertyOctetString(CallbackSetPropertyOctetString);

    // Remote device management
    fpRegisterCallbackReinitializeDevice(CallbackReinitializeDevice);

    // 4. Setup the BACnet device
    // ==================================================================================
    // Setup device object
    if (!fpAddDevice(APPLICATION_BACNET_DEVICE_INSTANCE))
    {
        printf("Error: Could not add device. Device instance=%u\n", APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: BACnet device created: Device instance=%u\n", APPLICATION_BACNET_DEVICE_INSTANCE);

    // Enable Optional Device Properties
    // Description
    if (!fpSetPropertyEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_DEVICE, APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION, true))
    {
        printf("Failed to enable the description property for Device");
        return 0;
    }

    // Application software version
    if (!fpSetPropertyEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_DEVICE, APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_APPLICATION_SOFTWARE_VERSION, true))
    {
        printf("Failed to enable the application software property for Device");
        return 0;
    }

    // Enable services
    // By default the write service is not enabled. We have to enable it to allow users to write to points.
    if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_WRITE_PROPERTY, true))
    {
        printf("Error: Failed to enabled the WriteProperty service=[%u] for Device %u\n", CASBACnetStackExampleConstants::SERVICE_WRITE_PROPERTY, APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: Enabled WriteProperty for Device %u\n", APPLICATION_BACNET_DEVICE_INSTANCE);

    // Read Property Multiple service is a nice to have, not required for a BACnet server to work but it does make polling the device easier.
    if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_READ_PROPERTY_MULTIPLE, true))
    {
        printf("Error: Failed to enabled the Read Property Multiple service=[%u] for Device %u\n", CASBACnetStackExampleConstants::SERVICE_READ_PROPERTY_MULTIPLE, APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: Enabled Read Property Multiple for Device %u\n", CASBACnetStackExampleConstants::SERVICE_READ_PROPERTY_MULTIPLE);

    printf("FYI: Enabling SubscribeCOV... ");
    if (!fpSetServiceEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::SERVICE_SUBSCRIBE_COV, true))
    {
        printf("Failed to enable the SubscribeCOV service\n");
        return false;
    }
    printf("OK\n");

    // Add objects
    // Analog Input
    printf("FYI: Adding AnalogInput. analogInput.instance=[%d]... ", APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE);
    if (!fpAddObject(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE))
    {
        printf("Failed to add AnalogInput\n");
        return 0;
    }
    printf("OK\n");

    fpSetPropertyWritable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_UNITS, true);
    fpSetPropertySubscribable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE, true);
    fpSetPropertyByObjectTypeEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION, true);

    // Multi-State Value
    printf("FYI: Adding Multi-State Value. multiStateValue.instance=[%d]... ", APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE);
    if (!fpAddObject(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE, APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE))
    {
        printf("Error: Failed to add multi-state-output (%u) to Device (%u)\n", APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE, APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    fpSetPropertyWritable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE, APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE, true);
    fpSetPropertyEnabled(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE, APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_STATE_TEXT, true);
    printf("OK\n");

    // Network Port
    printf("FYI: Adding NetworkPort. networkPort.instance=[%d]... ", APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE);
    if (!fpAddNetworkPortObject(APPLICATION_BACNET_DEVICE_INSTANCE, APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE, CASBACnetStackExampleConstants::NETWORK_TYPE_IPV4, CASBACnetStackExampleConstants::PROTOCOL_LEVEL_BACNET_APPLICATION, CASBACnetStackExampleConstants::NETWORK_PORT_LOWEST_PROTOCOL_LAYER))
    {
        printf("Failed to add NetworkPort object\n");
        return 0;
    }
    fpSetPropertyWritable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT, APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_BBMD_ADDRESS, true);
    fpSetPropertyWritable(APPLICATION_BACNET_DEVICE_INSTANCE, CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT, APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE, CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_SUBSCRIPTION_LIFETIME, true);
    printf("OK\n");

    // Print IP address, subnet mask, and broadcast IP address
    if (!setup_network_port())
        printf("Failed to setup network port object");

    printf("FYI: NetworkPort.IPAddress: %d.%d.%d.%d\n", APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[0], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[1], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[2], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[3]);
    printf("FYI: NetworkPort.IPSubnetMask: %d.%d.%d.%d\n", APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[0], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[1], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[2], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[3]);
    printf("FYI: NetworkPort.BroadcastIPAddress: %d.%d.%d.%d\n", APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[0], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[1], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[2], APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[3]);

    // 5. Send I-Am of this device
    // ==================================================================================
    uint8_t connectionString[6];
    memcpy(connectionString, APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS, 4);
    connectionString[4] = APPLICATION_BACNET_UDP_PORT / 256;
    connectionString[5] = APPLICATION_BACNET_UDP_PORT % 256;

    // Send a IAm Message to announce to the network that a new BACnet device has started.
    if (!fpSendIAm(APPLICATION_BACNET_DEVICE_INSTANCE, connectionString, 6, CASBACnetStackExampleConstants::NETWORK_TYPE_BACNET_IP, true, 65535, NULL, 0))
    {
        printf("Error: Unable to send IAm for Device %u", APPLICATION_BACNET_DEVICE_INSTANCE);
        return 0;
    }
    printf("FYI: Sent broadcast IAm message\n");

    // 6. Main Loop
    // ==================================================================================
    printf("FYI: Entering main loop\n");
    for (;;)
    {
        fpLoop();
        cyw43_arch_poll();
        update_led_status();
        sleep_ms(0);
    }
    return 0;
}

// Helper functions
// ==============================================================================
bool get_broadcast_address(uint8_t *broadcastAddress, size_t maxbroadcastAddressSize)
{
    if (broadcastAddress == NULL || maxbroadcastAddressSize < 4)
    {
        return false; // Not enough space
    }
    uint8_t localIP[IPV4_ADDR_LENGTH];
    uint8_t subnetMask[IPV4_ADDR_LENGTH];
    if (
        !example_ntoa(localIP, IPV4_ADDR_LENGTH, cyw43_state.netif[0].ip_addr.addr) ||
        !example_ntoa(subnetMask, IPV4_ADDR_LENGTH, cyw43_state.netif[0].netmask.addr))
        return false;

    broadcastAddress[0] = localIP[0] | ~subnetMask[0];
    broadcastAddress[1] = localIP[1] | ~subnetMask[1];
    broadcastAddress[2] = localIP[2] | ~subnetMask[2];
    broadcastAddress[3] = localIP[3] | ~subnetMask[3];
    return true;
}

// https://forums.raspberrypi.com/viewtopic.php?t=340208
// Send UDP message
void send_udp(char *IP, int port, const void *data, int data_size)
{
    ip_addr_t destAddr;
    ip4addr_aton(IP, &destAddr);
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_size, PBUF_RAM);
    memcpy(p->payload, data, data_size);
    cyw43_arch_lwip_begin();
    udp_sendto(upcb, p, &destAddr, port);
    cyw43_arch_lwip_end();
    pbuf_free(p);
}

// Receive UDP message callback
void receive_udp_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    struct udp_packet packet;
    packet.src_addr[0] = addr->addr & 0xff;
    packet.src_addr[1] = (addr->addr >> 8) & 0xff;
    packet.src_addr[2] = (addr->addr >> 16) & 0xff;
    packet.src_addr[3] = addr->addr >> 24;
    packet.port = port;
    packet.length = p->len;
    memcpy(packet.payload, p->payload, p->len);

    enqueue_udp_packet(&packet);
    pbuf_free(p);
}

// Convert unsigned long address into int array
bool example_ntoa(uint8_t *addressBuffer, size_t maxBufferSize, unsigned long ipAddress)
{
    if (addressBuffer == NULL || maxBufferSize < 4)
    {
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
bool enqueue_udp_packet(struct udp_packet *packet)
{
    // Packet queue is full
    if (packet_queue_rear == PACKET_QUEUE_MAXSIZE - 1)
    {
        return false;
    }

    if (packet_queue_front == -1)
        packet_queue_front = 0;

    packet_queue_rear = packet_queue_rear + 1;
    packet_queue[packet_queue_rear] = *packet;
    return true;
}

// Dequeue front of packet queue (if available)
// Returns false if no packet available to dequeue
bool get_udp_packet(struct udp_packet *packet)
{
    // no packet is available in queue
    if (packet_queue_front == -1 || packet_queue_front > packet_queue_rear)
    {
        return false;
    }

    // packet available
    *packet = packet_queue[packet_queue_front];
    packet_queue_front = packet_queue_front + 1;

    if (packet_queue_front > packet_queue_rear)
    {
        packet_queue_front = packet_queue_rear = -1;
    }
    return true;
}

bool setup_network_port()
{
    if (
        !example_ntoa(APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS, IPV4_ADDR_LENGTH, cyw43_state.netif[0].ip_addr.addr) ||
        !example_ntoa(APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK, IPV4_ADDR_LENGTH, cyw43_state.netif[0].netmask.addr) ||
        !example_ntoa(APPLICATION_BACNET_OBJECT_NETWORKPORT_GATEWAY, IPV4_ADDR_LENGTH, cyw43_state.netif[0].gw.addr))
        return false;

    memcpy(APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER[0], dns_getserver(0), IPV4_ADDR_LENGTH);
    APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER_SIZE = 1;
    APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[0] = APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[0] | ~APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[0];
    APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[1] = APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[1] | ~APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[1];
    APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[2] = APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[2] | ~APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[2];
    APPLICATION_BACNET_OBJECT_NETWORKPORT_IPBROADCASTADDRESS[3] = APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS[3] | ~APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK[3];
    return true;
}

// handle LED blinking
bool blink_led(struct repeating_timer *t)
{
    LEDState = !LEDState;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LEDState);
    return true;
}

// Change the timer interrupt of the LED
void update_led_status()
{
    if (gLEDModeFlag)
    {
        cancel_repeating_timer(&blink_timer);
        gLEDModeFlag = 0;
        switch (gLEDMode)
        {
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
uint16_t CallbackReceiveMessage(uint8_t *message, const uint16_t maxMessageLength, uint8_t *receivedConnectionString, const uint8_t maxConnectionStringLength, uint8_t *receivedConnectionStringLength, uint8_t *networkType)
{
    // Check parameters
    if (message == NULL || maxMessageLength == 0)
    {
        printf("Error: Invalid input buffer");
        return 0;
    }
    if (receivedConnectionString == NULL || maxConnectionStringLength == 0)
    {
        printf("Error: Invalid connection string buffer");
        return 0;
    }
    if (maxConnectionStringLength < 6)
    {
        printf("Error: Not enough space for a UDP connection string");
        return 0;
    }

    // processing incoming packet, must be called before reading the buffer
    struct udp_packet packet;
    if (!get_udp_packet(&packet))
        return 0;

    // We got a message.
    receivedConnectionString[0] = packet.src_addr[0];
    receivedConnectionString[1] = packet.src_addr[1];
    receivedConnectionString[2] = packet.src_addr[2];
    receivedConnectionString[3] = packet.src_addr[3];
    receivedConnectionString[4] = (packet.port >> 8) & 0xFF;
    receivedConnectionString[5] = packet.port;
    memcpy(message, (uint8_t *)packet.payload, packet.length);

    *receivedConnectionStringLength = 6;
    *networkType = CASBACnetStackExampleConstants::NETWORK_TYPE_BACNET_IP;

    printf("FYI: Received message with %u bytes from %u.%u.%u.%u:%u\n", packet.length, receivedConnectionString[0], receivedConnectionString[1], receivedConnectionString[2], receivedConnectionString[3], packet.port);
    return packet.length;
}

uint16_t CallbackSendMessage(const uint8_t *message, const uint16_t messageLength, const uint8_t *connectionString, const uint8_t connectionStringLength, const uint8_t networkType, bool broadcast)
{
    if (message == NULL || messageLength == 0)
    {
        printf("FYI: Nothing to send");
        return 0;
    }
    if (connectionString == NULL || connectionStringLength == 0)
    {
        printf("FYI: No connection string");
        return 0;
    }

    // Verify Network Type
    if (networkType != CASBACnetStackExampleConstants::NETWORK_TYPE_BACNET_IP)
    {
        printf("FYI: Message for different network. Type = %d\n", networkType);
        printf("FYI: This server is configured for BACnet IP network type messages only\n");
        return 0;
    }

    // Prepare the IP Address
    char destinationIPAddressAsString[32];
    if (broadcast)
    {
        uint8_t broadcastIPAddress[4] = {255, 255, 255, 255};
        if (!get_broadcast_address(broadcastIPAddress, 4))
        {
            printf("Error: Could not get the broadcast iIP address");
            return 0;
        }
        snprintf(destinationIPAddressAsString, 32, "%u.%u.%u.%u", broadcastIPAddress[0], broadcastIPAddress[1], broadcastIPAddress[2], broadcastIPAddress[3]);
    }
    else
    {
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
bool CallbackGetPropertyCharString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, char *value, uint32_t *valueElementCount, const uint32_t maxElementCount, uint8_t *encodingType, const bool useArrayIndex, const uint32_t propertyArrayIndex)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }
    printf("FYI: CallbackGetPropertyCharString deviceInstance=%d, objectType=%d, objectInstance=%d, propertyIdentifier=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier);

    // Device properties
    if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_DEVICE && objectInstance == APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION)
        {
            if (strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION) <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_DEVICE_DESCRIPTION, strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION));
                *valueElementCount = (uint32_t)strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION);
                return true;
            }
            return false;
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_APPLICATION_SOFTWARE_VERSION)
        {
            if (strlen(APPLICATION_BACNET_DEVICE_DESCRIPTION) <= maxElementCount)
            {
                memcpy(value, APPLICATION_VERSION, strlen(APPLICATION_VERSION));
                *valueElementCount = (uint32_t)strlen(APPLICATION_VERSION);
                return true;
            }
            return false;
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false)
        {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_DEVICE_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_DEVICE_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
    }

    // Analog-input object properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE)
    {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false)
        {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_DESCRIPTION)
        {
            if (strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION) <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION, strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION));
                *valueElementCount = (uint32_t)strlen(APPLICATION_BACNET_OBJECT_ANALOG_INPUT_DESCRIPTION);
                return true;
            }
        }
        return false;
    }

    // Multi-state value object properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE && objectInstance == APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE)
    {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false)
        {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_MSV_LED_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_OBJECT_MSV_LED_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_STATE_TEXT && useArrayIndex == true)
        {
            if (propertyArrayIndex > LED_MODE_STATE_COUNT || propertyArrayIndex <= 0)
            {
                return false; // Invalid range
            }

            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_MSV_LED_STATE_TEXT[propertyArrayIndex - 1]);
            if (objectNameStringLength <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_OBJECT_MSV_LED_STATE_TEXT[propertyArrayIndex - 1], objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
    }

    // Network port object properties
    else if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
    {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_OBJECT_NAME && useArrayIndex == false)
        {
            size_t objectNameStringLength = strlen(APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE_OBJECT_NAME);
            if (objectNameStringLength <= maxElementCount)
            {
                memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE_OBJECT_NAME, objectNameStringLength);
                *valueElementCount = objectNameStringLength;
                return true;
            }
        }
        return false;
    }
    return false;
}

bool CallbackGetPropertyUInt(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t *value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    printf("FYI: CallbackGetPropertyUInt deviceInstance=%d, objectType=%d, objectInstance=%d, propertyIdentifier=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier);

    if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE && objectInstance == APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE)
    {
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE && useArrayIndex == false)
        {
            *value = gLEDMode;
            return true;
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_NUMBER_OF_STATES && useArrayIndex == false)
        {
            *value = LED_MODE_STATE_COUNT;
            return true;
        }
        else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_STATE_TEXT && useArrayIndex == true && propertyArrayIndex == 0)
        {
            *value = LED_MODE_STATE_COUNT;
            return true;
        }
    }

    // Network Port Object FdBbmdAddress Port
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_BBMD_ADDRESS)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            if (useArrayIndex && propertyArrayIndex == CASBACnetStackExampleConstants::FD_BBMD_ADDRESS_PORT)
            {
                // Check for index 2, which is looking for the fdBbmdAddress port portion
                *value = APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDPORT;
                return true;
            }
        }
    }
    // Network Port Object FdSubscriptionLifetime
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_SUBSCRIPTION_LIFETIME)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            *value = APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDSUBSCRIPTIONLIFETIME;
            return true;
        }
    }

    // Example of Network Port Object BACnet IP UDP Port property
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_BACNET_IP_UDP_PORT)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            *value = APPLICATION_BACNET_UDP_PORT;
            return true;
        }
    }

    // Example of Network Port Object IP DNS Server Array Size property
    // Any properties that are an array must have an entry here for the array size.
    // The array size is provided only if the useArrayIndex parameter is set to true and the propertyArrayIndex is zero.
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_IP_DNS_SERVER)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            if (useArrayIndex && propertyArrayIndex == 0)
            {
                *value = (uint32_t)APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER_SIZE;
                return true;
            }
        }
    }

    return false;
}

// Callback used by the BACnet Stack to get Real property values from the user
bool CallbackGetPropertyReal(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, float *value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    // Example of Analog Input / Value Object Present Value property
    if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE)
        {
            // Conversion factor from https://github.com/DeimosHall/RP2040_CPU_Temperature
            uint16_t adc = adc_read();
            float ADC_Voltage = float(adc) * CONVERSION_FACTOR;
            *value = 27 - (ADC_Voltage - 0.706) / 0.001721;
            if (temperature_unit == CASBACnetStackExampleConstants::TEMPERATURE_ENUM_DEGREES_FAHRENHEIT)
                *value = *value * 9 / 5 + 32;
            return true;
        }
    }
    return false;
}

// Callback used by the BACnet Stack to get Enumerated property values from the user
bool CallbackGetPropertyEnum(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, uint32_t *value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    printf("CallbackGetPropertyEnum deviceInstance=%d, , objectType=%d, objectInstance=%d, propertyIdentifier=%d useArrayIndex=%d, propertyArrayIndex=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier, useArrayIndex, propertyArrayIndex);

    // Example of Analog Input / Value Object Present Value property
    if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_UNITS)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE)
        {
            *value = temperature_unit;
            return true;
        }
    }

    // Network Port Object - FdBbmdAddress Host Type
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_BBMD_ADDRESS)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            *value = APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDHOSTTYPE;
            return true;
        }
    }
    // We could not answer this request.
    return false;
}

// Callback used by the BACnet Stack to get Boolean property values from the user
bool CallbackGetPropertyBool(uint32_t deviceInstance, uint16_t objectType, uint32_t objectInstance, uint32_t propertyIdentifier, bool *value, bool useArrayIndex, uint32_t propertyArrayIndex)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    // Network Port Object - Changes Pending property
    if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_CHANGES_PENDING)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            *value = APPLICATION_BACNET_OBJECT_NETWORKPORT_CHANGESPENDING;
            return true;
        }
    }
    return false;
}

// Callback used by the BACnet Stack to get OctetString property values from the user
bool CallbackGetPropertyOctetString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, uint8_t *value, uint32_t *valueElementCount, const uint32_t maxElementCount, const bool useArrayIndex, const uint32_t propertyArrayIndex)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    // Example of Network Port Object IP Address property
    if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_IP_ADDRESS)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_IPADDRESS, IPV4_ADDR_LENGTH);
            *valueElementCount = IPV4_ADDR_LENGTH;
            return true;
        }
    }
    // Example of Network Port Object IP Default Gateway property
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_IP_DEFAULT_GATEWAY)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_GATEWAY, IPV4_ADDR_LENGTH);
            *valueElementCount = IPV4_ADDR_LENGTH;
            return true;
        }
    }
    // Example of Network Port Object IP Subnet Mask property
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_IP_SUBNET_MASK)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_IPSUBNETMASK, IPV4_ADDR_LENGTH);
            *valueElementCount = IPV4_ADDR_LENGTH;
            return true;
        }
    }

    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_IP_DNS_SERVER)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            // The IP DNS Server property is an array of DNS Server addresses
            if (useArrayIndex)
            {
                if (propertyArrayIndex != 0 && propertyArrayIndex <= APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER_SIZE)
                {
                    memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_DNSSERVER[propertyArrayIndex - 1], IPV4_ADDR_LENGTH);
                    *valueElementCount = IPV4_ADDR_LENGTH;
                    return true;
                }
            }
        }
    }
    // Network Port Object FdBbmdAddress Host (as IP Address)
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_BBMD_ADDRESS)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            if (useArrayIndex && propertyArrayIndex == CASBACnetStackExampleConstants::HOST_TYPE_IPADDRESS)
            {
                memcpy(value, APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDHOSTIP, IPV4_ADDR_LENGTH);
                *valueElementCount = IPV4_ADDR_LENGTH;
                return true;
            }
        }
    }
    return false;
}

// SetProperty Callbacks
// ==============================================================================
// Callback used by the BACnet Stack to set Unsigned Int property values to the user
bool CallbackSetPropertyUInt(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t *errorCode)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    printf("FYI: CallbackSetPropertyUInt deviceInstance=%d, objectType=%d, objectInstance=%d, propertyIdentifier=%d, value=%d\n", deviceInstance, objectType, objectInstance, propertyIdentifier, value);

    if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_MULTI_STATE_VALUE && objectInstance == APPLICATION_BACNET_OBJECT_MSV_LED_INSTANCE && propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_PRESENT_VALUE && useArrayIndex == false)
    {
        switch (value)
        {
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

    // Network Port Object FdSubscriptionLifetime
    else if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_SUBSCRIPTION_LIFETIME)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDSUBSCRIPTIONLIFETIME = value;
            APPLICATION_BACNET_OBJECT_NETWORKPORT_CHANGESPENDING = true;
            return true;
        }
    }
    return false;
}

// Callback used by the BACnet Stack to set Enumerated property values to the user
bool CallbackSetPropertyEnum(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint32_t value, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t *errorCode)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_ANALOG_INPUT)
    {
        // Setting temperature units in temperature sensor analog input
        if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_UNITS && objectInstance == APPLICATION_BACNET_OBJECT_ANALOG_INPUT_INSTANCE)
        {
            switch (value)
            {
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
    return false;
}

// Callback used by the BACnet Stack to set Enumerated property values to the user
bool CallbackSetPropertyOctetString(const uint32_t deviceInstance, const uint16_t objectType, const uint32_t objectInstance, const uint32_t propertyIdentifier, const uint8_t *value, const uint32_t length, const bool useArrayIndex, const uint32_t propertyArrayIndex, const uint8_t priority, uint32_t *errorCode)
{
    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    // Example of setting FdBbmdAddress Host IP
    if (propertyIdentifier == CASBACnetStackExampleConstants::PROPERTY_IDENTIFIER_FD_BBMD_ADDRESS)
    {
        if (objectType == CASBACnetStackExampleConstants::OBJECT_TYPE_NETWORK_PORT && objectInstance == APPLICATION_BACNET_OBJECT_NETWORKPORT_INSTANCE)
        {
            if (useArrayIndex && propertyArrayIndex == CASBACnetStackExampleConstants::FD_BBMD_ADDRESS_HOST)
            {
                if (length > 4)
                {
                    *errorCode = CASBACnetStackExampleConstants::ERROR_VALUE_OUT_OF_RANGE;
                    return false;
                }
                if (memcmp(APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDHOSTIP, value, length) == 0)
                {
                    // No change, return true
                    return true;
                }
                else
                {
                    // Store new value and set changes pending to true
                    memcpy(APPLICATION_BACNET_OBJECT_NETWORKPORT_FDBBMDHOSTIP, value, length);
                    APPLICATION_BACNET_OBJECT_NETWORKPORT_CHANGESPENDING = true;
                    return true;
                }
            }
        }
    }
    return false;
}

// Other Callbacks
// ==============================================================================
bool CallbackReinitializeDevice(const uint32_t deviceInstance, const uint32_t reinitializedState, const char *password, const uint32_t passwordLength, uint32_t *errorCode)
{
    // This callback is called when this BACnet Server device receives a ReinitializeDevice message
    // In this callback, you will handle the reinitializedState.
    // If reinitializedState = ACTIVATE_CHANGES (7) then you will apply any network port changes and store the values in non-volatile memory
    // If reinitializedState = WARM_START(1) then you will apply any network port changes, store the values in non-volatile memory, and restart the device.

    // Before handling the reinitializedState, first check the password.
    // If your device does not require a password, then ignore any password passed in.
    // Otherwise, validate the password.
    //		If password invalid, missing, or incorrect: set errorCode to PasswordInvalid (26)
    // In this example, a password of 12345 is required.

    if (deviceInstance != APPLICATION_BACNET_DEVICE_INSTANCE)
    {
        return false;
    }

    if (password == NULL || passwordLength == 0)
    {
        *errorCode = CASBACnetStackExampleConstants::ERROR_PASSWORD_FAILURE;
        return false;
    }

    if (strcmp(password, "12345") != 0)
    {
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
    if (reinitializedState == CASBACnetStackExampleConstants::REINITIALIZED_STATE_ACTIVATE_CHANGES)
    {
        APPLICATION_BACNET_OBJECT_NETWORKPORT_CHANGESPENDING = false;
        return true;
    }
    else if (reinitializedState == CASBACnetStackExampleConstants::REINITIALIZED_STATE_WARM_START)
    {
        // Flag for reboot and handle reboot after stack responds with SimpleAck.
        APPLICATION_BACNET_OBJECT_NETWORKPORT_CHANGESPENDING = false;
        return true;
    }
    else
    {
        // All other states are not supported in this example.
        *errorCode = CASBACnetStackExampleConstants::ERROR_OPTIONAL_FUNCTIONALITY_NOT_SUPPORTED;
        return false;
    }
}

time_t CallbackGetSystemTime()
{
    // get_absolute_time returns absolute_time_t (uint64_t)
    return (time_t)get_absolute_time();
}
