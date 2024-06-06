# Using the RoboRacer

The RoboRacer is controlled via the user interface hosted on a website.

On the interface, [control buttons](#controlling-the-roboracer) and [performance metrics](#performance-metrics) are available.

# Accessing the User Interface

The web UI is easily accessible from a device (preferably a mobile device) that connects to WiFi. 

1. Turn on the RoboRacer and [compile](https://robo-racer.github.io/RoboRacer/#compilation) the website.

2. On your device, navigate to the WiFi settings. Find the WiFi host on the device and connect to it.

3. Open a browser on the device and navigate to the URL `http://192.168.4.1`

**Done!** You should now see the web interface and be able to control the robot.

> Note: WiFi connection is limited to one device.

# Controlling the RoboRacer

The RoboRacer has two customizable components: **time** and **distance**. These values are up to the user, dependent on the desired track event the user is training for. The speed of the robot is then calculated so that it reaches the inputted distance within the given time.

**Time**: Enter the time you wish to finish the race by in the format **mm:ss**, representing **minutes** and **seconds**.

**Distance**: Enter the distance (in **meters**) you want the RoboRacer to meet the time by.

After inputting these values, lock them in with the **set** button. When you are ready to race, click the **start** button.

# Performance Metrics

The stats of your run will appear after the race is finished, displaying the **time**, **distance**, and **speed**.
