# RoboRacer

Traditional training methods for solo track athletes have hit a plateau, with limited means to mimic real-race pacing and conditions. Athletes and coaches are in search of new, innovative solutions to break through these barriers and enhance performance. The RoboRacer aims to solve this problem by creating an autonomous robot that acts as a training partner for track athletes.

This robotic system uses advanced algorithms to follow lines on a track and adjust to various training regimens. With the integration of obstacle detection, it ensures safety for solitary training sessions. The accompanying app offers user-friendly control and access to essential training data, empowering athletes to refine their strategies and improve performance with actionable insights.

For instructions on how to use the RoboRacer, visit https://robo-racer.github.io/RoboRacer/usage

# Table of Contents

- [Hardware](#hardware)
    - [Electronics Diagram](#electronics-diagram)
    - [Wire Diagrams](#wire-diagrams)
- [Obstacle Avoidance](#obstacle-avoidance)
- [Line Tracking](#line-tracking)
- [User Communication](#user-communication)

# Hardware

The RoboRacer was built from scratch using the following parts:

| Hardware   | Description             |
| ---------- | ----------------------- |
| [Arrma Vortex Truck RC car](https://www.arrma-rc.com/en/product/1-10-vorteks-4x4-3s-blx-stadium-truck-rtr-red/ARA4305V3T1.html) | The RoboRacer was built with the base of an RC car as the body of the robot. |
| [Arduino Portenta H7 Vision Bundle](https://store-usa.arduino.cc/collections/portenta-family/products/machine-vision-bundle) | This bundle includes our main microcontroller for the device and a camera system designed for vision processing. |
| [Portenta Breakout Board](https://store-usa.arduino.cc/collections/portenta-family/products/arduino-portenta-breakout) | To simplify development. |
| [HOOVO 3S Lipo Battery 10000 mAh](https://www.amazon.com/HOOVO-Battery-10000mAh-Softcase-Compatible/dp/B09YM8P25C?th=1) | A battery to power the RoboRacer. |
| [Power Distribution Board](https://www.amazon.com/Distribution-FCHUB-12S-Supports-Regulators-Current/dp/B07MHKTF7F/ref=sr_1_35?keywords=Power+Distribution+Board&sr=8-35) | A power distribution board is needed to power the robot and monitor its current. |
| [ESP32 Development Board](https://www.amazon.com/ESP32-S3-Bluetooth-Internet-Development-ESP32-S3-DevKit/dp/B0BPP28ZFB/ref=sr_1_4?crid=15SEBRVTA5Q0L&keywords=esp32+s3+usb&qid=1704830175&sprefix=esp32+s3+usbc+%2Caps%2C214&sr=8-4) | WebUI hosting device. |
| [Color Sensors](https://www.adafruit.com/product/3595) | To detect the lines on the track. |
| [Motion Sensor (IMU)](https://www.adafruit.com/product/4754) | For detecting any type of motion values, such as acceleration and direction. |
| [PETG Filament](https://www.amazon.com/OVERTURE-Filament-Consumables-Dimensional-Accuracy/dp/B07PGYHYV8/ref=sr_1_1_sspa?keywords=petg+filament&qid=1704830055&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) | Used for prototyping and 3D printing mounts onto the device. |
| [MicroSD Modules](https://www.adafruit.com/product/4682) | To store information on the device. |
| [LiPo Battery Charger](https://www.amazon.com/Haisito-HB6-lipo-Charger/dp/B08C592PNV/ref=sxin_14_pa_sp_search_thematic_sspa?content-id=amzn1.sym.92181fe7-c843-4c1b-b489-84c087a93895%3Aamzn1.sym.92181fe7-c843-4c1b-b489-84c087a93895&cv_ct_cx=3s+lipo+battery+charger&keywords=3s+lipo+battery+charger&pd_rd_i=B08C592PNV&pd_rd_r=675e1556-c8ba-45eb-a0ff-37846665caf2&pd_rd_w=ckPZV&pd_rd_wg=2XPQt&pf_rd_p=92181fe7-c843-4c1b-b489-84c087a93895&pf_rd_r=A6D16VTCQCB6W7WD2GCM&qid=1706645569&sbo=RZvfv%2F%2FHxDF%2BO5021pAnSA%3D%3D&sr=1-1-364cf978-ce2a-480a-9bb0-bdb96faa0f61-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9zZWFyY2hfdGhlbWF0aWM&psc=1) | Used to charge the LiPo batteries used for the RC car. |
| [TCA9548A I2C Multiplexer](https://www.amazon.com/gp/product/B017C09ETS/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1) | Differentiate color sensors, as multiple Devices share the same I2C address. |
| [Hall Effect Sensor](https://www.amazon.com/HiLetgo-NJK-5002C-Proximity-3-Wires-Normally/dp/B01MZYYCLH/ref=sr_1_12?crid=1DWAL653S785H&keywords=Hall+effect+sensor+-+US5881LUA&qid=1707163489&sprefix=hall+effect+sensor+-+us5881lua%2Caps%2C207&sr=8-12) | To determine the speed of the RC car using rpm of the main axle. |
| [PLA Filament White](https://www.amazon.com/gp/product/B07PGZNM34/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1) | Utilized for quicker prototyping, whilst the PETG will be used for final designs due to heat resistance. |
| [OpenMV Cam H7 Plus](https://www.sparkfun.com/products/16989) | For line tracking. |
| [Ultrasonic sensor: MB1260 XL-MaxSonar-EZL0](https://www.digikey.com/en/products/detail/maxbotix-inc./MB1260-000/7896804?utm_adgroup=General&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Shopping_Product_Zombie%20SKUs&utm_term=&utm_content=General&utm_id=go_cmp-17815035045_adg-_ad-__dev-c_ext-_prd-7896804_sig-CjwKCAiA6KWvBhAREiwAFPZM7uXuBr5x2_9tsJn6quUxP_FLTGHsh9kwDjljyNy30PU-59ozLj2mChoCTV8QAvD_BwE&gad_source=1&gclid=CjwKCAiA6KWvBhAREiwAFPZM7uXuBr5x2_9tsJn6quUxP_FLTGHsh9kwDjljyNy30PU-59ozLj2mChoCTV8QAvD_BwE) | For detecting obstacles in front of the robot. |

---
## Electronics Diagram


---
## Wire Diagrams

Wire diagrams used during the development process:

## RoboRacer Wire Diagram
![RoboRacer wire diagram](/images/RoboRacer_Wire_Diagram.png)

## Color Sensor Wire Diagram
![Color sensor and multiplexer wire diagram](/images/ColorSensorMultiplexerWireDiagram.jpg)

---
# Obstacle Avoidance

<!-- TODO -->

---
# Line Tracking

<!-- TODO -->
Line tracking is implemented using the OpenMV Cam H7 Plus. 

---
# User Communication

## Installation
### Using PlatformIO
[PlatformIO](https://docs.platformio.org/en/latest/what-is-platformio.html) - "PlatformIO is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products."

1. Install [PlatformIO IDE](http://platformio.org/platformio-ide)
2. Copy [esp32-s3-devkitm-1.json](./public/esp32-s3-devkitm-1.json) from the public directory in this project and paste it into the .platformio/platforms/espressif32/boards folder on your personal system.
3. In the web-ui directory, run the npm install command
3. Follow [Compilation](#compilation) steps below to compile and run this project 

## The Project

## ESP32-S3-DevkitM-1
### Components
<p align="center">
  <img src="https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/_images/ESP32-S3-DevKitM-1_v1-annotated-photo.png" alt="ESP32-S3-DevkitM-1" height=350 style="background-color: white;">
  <p align="center">Figure 1. ESP32 DevkitM-1 Components</p>
</p>

| Key Component | Description |
| ------------- | ----------- |
| ESP32-S3-MINI-1/1U | ESP32-S3-MINI-1 and ESP32-S3-MINI-1U are two general-purpose Wi-Fi and Bluetooth Low Energy combo modules that have a rich set of peripherals. ESP32-S3-MINI-1 comes with a PCB antenna. ESP32-S3-MINI-1U comes with an external antenna connector. At the core of the modules is ESP32-S3FN8, a chip equipped with an 8 MB flash. Since flash is packaged in the chip, rather than integrated into the module, ESP32-S3-MINI-1/1U has a smaller package size. |
| 5V to 3.3V LDO | Power regulator that converts a 5 V supply into a 3.3 V output. |
| Pin Headers | All available GPIO pins (except for the SPI bus for flash) are broken out to the pin headers on the board for easy interfacing and programming. For details, please see Header Block. |
| USB-to-UART Port | A Micro-USB port used for power supply to the board, for flashing applications to the chip, as well as for communication with the chip via the on-board USB-to-UART bridge. |
| Boot Button | Download button. Holding down Boot and then pressing Reset initiates Firmware Download mode for downloading firmware through the serial port. |
| Reset Button | Press this button to restart ESP32-S3. |
| ESP32-S3 USB Port | ESP32-S3 full-speed USB OTG interface, compliant with the USB 1.1 specification. The interface is used for power supply to the board, for flashing applications to the chip, for communication with the chip using USB 1.1 protocols, as well as for JTAG debugging. |
| USB-to-UART Bridge | Single USB-to-UART bridge chip provides transfer rates up to 3 Mbps. |
| RGB LED | Addressable RGB LED, driven by GPIO48. |
| 3.3 V Power On LED | Turns on when the USB power is connected to the board. |
| | |

### Memory Space Allocation via Partitioning
Generally, the ESP32 should have the [default partitioning scheme for 8MB](https://github.com/espressif/arduino-esp32/blob/master/tools/partitions/default_8MB.csv?plain=1), but due to the size needed to be allocated for the filesystem image, a new partitioning set, [partitions_custom.csv](./partitions_custom.csv), has been created. As can be seen below, we reallocated memory from app1 to also be included in the spiffs section.
```ini
#default_8MB.csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x330000,
app1,     app,  ota_1,   0x340000,0x330000,
spiffs,   data, spiffs,  0x670000,0x180000,
coredump, data, coredump,0x7F0000,0x10000,

#partitions_custom.csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x330000,
spiffs,   data, spiffs,  0x340000,0x4B0000,
coredump, data, coredump,0x7F0000,0x10000,
```


## The Code
### Web Server Code
The ESP32 is treated as an asynchronous web server, responding to a connected users HTTP requests as it recieves them. It holds the primary job of communication between the user and the RoboRacer, utilizing its UART (Universal asynchronous receiver-transmitter) capabilities to recieve and transmit data to the Portenta H7. This section will serve as a breakdown of how the code in [main.cpp](./src/main.cpp) accomplishes this task.

```cpp
if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
{
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
}
```
First we set up our SPIFFS (Serial Peripheral Interface Flash File System), which will allow us to format our files as listed in the ./data directory. SPIFFS can now act as a filesystem, which we use in the following code.

```cpp
server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
server.serveStatic("/static/", SPIFFS, "/");
```
This sets our server (which was set up on port 80) to serve our web app when connected to. The ESP32 WiFi is set up as an Access Point, which means we are simply allowing devices to connect with us, but not providing actual WiFi.
```cpp
WiFi.softAP(ssid, password, 1, 0, 1);

IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);
```
This code sets up the Access Point with ssid (wifi name) "ESP32-Access-Point", password "123456789", and several other options, notably limiting the maximum number of connections to 1, so no others can connect when the user is connected. We are able to output the IP address, which we give the user on the side of the RoboRacer.

```cpp
server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (request->method() == HTTP_POST) {
         if (!handlepostData(request, data)) {
            Serial.print("Something went wrong!!!");
            request->send(400, "text/plain", "false");
         }
         request->send(200, "text/plain", "true");
      } 
      if (request->method() == HTTP_GET) {
        //...
      }
      //...
    });
```
The above code details how we respond to HTTP requests. When we receive a request, we check what method it is, then we handle that request with built in functionality that decodes the body of the request and checks what it contains, like so:
```cpp
    JsonDocument jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, (const char *)datas);
    //...
    if (jsonDoc.containsKey("name")){
        String _name = jsonDoc["name"].as<String>();
        Serial.println(_name);
    }
    //...
```

### Web Application Code


### Libraries Used
- [WiFi.h](https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h)
    - Allows us to set up the ESP32 as an access point and connect to another device via WiFi
- [ESPAsyncWebServer.h](https://github.com/me-no-dev/ESPAsyncWebServer)
    - Treats the ESP32 as an asynchronous web server, allowing us to process HTTP Requests from the user
- [SPIFFS.h](https://github.com/espressif/arduino-esp32/blob/master/libraries/SPIFFS/src/SPIFFS.h)
    - Lets us work with the spiffs section of our board, storing a filesystem in the flash memory
- [FS.h](https://github.com/espressif/arduino-esp32/blob/master/libraries/FS/src/FS.h)
    - Makes a more cohesive filesystem on the ESP32, letting us navigate through it as one would a computer
- [ArduinoJson.h](https://arduinojson.org/)
    - Lets us both create and parse through JSON objects, allowing sending and retrieving of data from user
- [React Router DOM](https://reactrouter.com/en/main)
    - Builds user interface to route to pages
- [React Joystick Component](https://www.npmjs.com/package/react-joystick-component)
    - Allows usage of joystick on a dev page, to direct RoboRacer


## Compilation
1. In the web-ui directory, run the nmp run build command. This will compile the web application and move the appropriate files out of the /static directory.
2. Delete the contents of the ./data directory
3. Copy the contents of ./web-ui/build and paste them to ./data
4. In the [index.html](./data/index.html) file, search for the "static" keyword using ctrl+f. Update both instances of the word to reflect the new file structure, as demonstrated below ([why are we doing this?](#why-are-we-deleting-static)):
```html
<script defer="defer" src="/static/js/main.72aff11b.js"> --> <script defer="defer" src="/js/main.72aff11b.js">

<link href="/static/css/main.cf5403a1.css" rel="stylesheet"> --> <link href="/css/main.cf5403a1.css" rel="stylesheet">
```
5. Connect the ESP32 to your computer via it's USB Port
6. Run the platformio "Build", "Upload", "Build Filesystem Image", and "Upload Filesystem Image" commands (which can be found in the PlatformIO portion of VSCode's extention bar, or by using PlatformIO's commands in the terminal) 
7. You are now ready to connect your device to the ESP32's WIFI Access Point!

## Usage Example

## Extra Notes
### Why Are We Deleting Static?
We delete the /static directory and update index.html to no longer include it because spiffs can only store files up to 32 bytes long. data\static\js\main.72aff11b.js.map is 35 bytes long, and thus we remove the static directory from the name to reduce it to 28 bytes.

## Sources
[Esspressif ESP32-S3-DevkitM-1 User Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitm-1.html)
[]()
[]()
