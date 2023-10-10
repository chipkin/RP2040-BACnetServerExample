# RP2040-BACnetServerExample
Server example using the Raspberry PI RP2040 wifi chip

In this project we are using the [CAS BACnet stack](https://www.bacnetstack.com/) to generate a simple BACnet server with an Analog Input, Multi-state-value (MSV), and Netowrk Port object. The analog input displays the CPU temperature and can be configured with Celsius or Fahrenheit units. The MSV shows the current mode of the built-in LED.

A BACnet client (such as the [CAS BACnet Explorer](https://store.chipkin.com/products/tools/cas-bacnet-explorer)) can be used to write to this MSV to change the mode.

The MSV allows for three possaible values to be written.

- 1 = **Off**
- 2 = **On**
- 3 = **Blink** - Blinks the LED on and off at a rate of 1000 ms.
- 4 = **Blink** - Blinks the LED on and off at a rate of 250  ms.

The Analog Input allows for three possaible values to be written to the units.

- 1 = **Celsius** - 62
- 2 = **Fahrenheit** - 64

## Quick start

The [CAS BACnet Stack submodule](https://github.com/chipkin/BACnetServerExampleCPP/issues/8) is required for compilation.

For the example server to run properly, please enable the needed object types and features of the CAS BACNet Stack. For more details, please reference `Enabling optional functionality` and `Compiling example projects` sections of the *Quick Start Guide* (please contact Chipkin for this document).

To connect the pico to wifi, you need to update the WIFI_SSID and WIFI_PASSWORD fields in the BACnetServerExamplePico.cpp file.

## Build

1. Download and install the pico standard library and compilation tools
2. Update repository submodules
3. Create "build" folder in top of directory
4. Run "cmake .."
5. Run "make"
6. Holding the BOOTSEL button, plug the Raspberry Pi Pico W into the computer
7. Drag the compiled "BACnetServerExamplePico.uf2" file from the build directory onto the Raspberry Pi Pico W build folder

## Supported BIBBs

The CAS BACnet stack supports many other BIBBs, this minumial example only supports the following:

- **DS-RP-B**: Data Sharing - ReadProperty-B
- **DS-RPM-B**: Data Sharing - ReadPropertyMultiple-B
- **DS-WP-B**: Data Sharing - WriteProperty-B
- **DM-DDB-B**: Device and Network Management - Dynamic Device Binding-B (Who-Is)
- **DM-DOB-B**: Device and Network Management - Dynamic Object Binding-B (Who-Has)

## BACnet objects supported

The CAS BACnet stack supports many other object types this minimal example only supports the following:

- Analog Input (0)
- Device (8)
- Multi-state Value (19)
- Network Port (56)

## Device Tree

Below is the device tree.

![Preview of device tree](/device_tree.png?raw=true "Preview of device tree")

## Example of setting LED mode

![Preview of setting the MSV LED mode](/LED_demo.gif?raw=true "Preview of setting the MSV LED mode")

## Example output

The example output from the serial terminal of the ESP32

```txt
FYI: Attempting to connect to wifi SSID
FYI: Connected!
FYI: Binded to UDP socket 47808
FYI: CAS BACnet Stack version: 4.1.19.0
FYI: BACnet device created: Device instance=389007
FYI: Enabled WriteProperty for Device 389007
FYI: Enabled Read Property Multiple for Device 14
FYI: Enabling SubscribeCOV... OK
FYI: Adding AnalogInput. analogInput.instance=[2]... OK
FYI: Adding NetworkPort. networkPort.instance=[0]... OK
FYI: NetworkPort.IPAddress: 192.168.1.78
FYI: NetworkPort.IPSubnetMask: 255.255.255.0
FYI: NetworkPort.BroadcastIPAddress: 192.168.1.255
FYI: CallbackGetPropertyUInt deviceInstance=389007, objectType=8, objectInstance=389007, propertyIdentifier=62
FYI: CallbackGetPropertyUInt deviceInstance=389007, objectType=8, objectInstance=389007, propertyIdentifier=120
CallbackGetPropertyEnum deviceInstance=389007, , objectType=8, objectInstance=389007, propertyIdentifier=107 useArrayIndex=0, propertyArrayIndex=0
FYI: Sent message with 25 bytes to 192.168.1.255:47808
FYI: Sent broadcast IAm message
FYI: Entering main loop
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

```

## Tested hardware

- [Raspberry Pi Pico W](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)

## FAQ

### Can't build the project. *CASBACnetStackAdapter.h: No such file or directory*

Please see this issue [CASBACnetStackAdapter.h: No such file or directory](https://github.com/chipkin/ESP32-BACnetServerExample/issues/1)

### Binary size too large for FLASH

This issue occurs because the full compiled stack size exceeds the FLASH size of the Raspberry Pi Pico W. This can be resolved by reducing the number of compiled 
features in the stack. Instructions for how to do this can be found in the CAS BACnet Stack Quick Start Guide in the docs folder of the stack.
