# RoboRacer

Traditional training methods for solo track athletes have hit a plateau, with limited means to mimic real-race pacing and conditions. Athletes and coaches are in search of new, innovative solutions to break through these barriers and enhance performance. The RoboRacer aims to solve this problem by creating an autonomous robot that acts as a training partner for track athletes.

This robotic system uses advanced algorithms to follow lines on a track and adjust to various training regimens. With the integration of obstacle detection, it ensures safety for solitary training sessions. The accompanying app offers user-friendly control and access to essential training data, empowering athletes to refine their strategies and improve performance with actionable insights.

# Table of Contents

- [Hardware](#hardware)
    - [Electronics Diagram](#electronics-diagram)
    - [Wire Diagrams](#wire-diagrams)
- [Obstacle Avoidance](#obstacle-avoidance)
- [Web Communication](#web-communication)

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

## Electronics Diagram



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
# Web Communication

<!-- TODO Zak: add your documentation here -->