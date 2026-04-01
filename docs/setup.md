# Installation Setup Guide
This mini-guide goes setting up and installing all the necessary software/firmware to get
the Turtlebot4 platform to work should somebody stumble on this.

You're mainly just following other people's guides, mainly [Clearpath's](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html),  but this is useful if you're starting
from nothing:
1) Turn the Turtlebot4 off and make sure it's not on the charger, you need to disconnect the top from the base 
to access the PI, as [shown](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html). The SD card slot is on the side opposide of the big ports.
If there's already an SD card sticking out, remove it, otherwise move on with a new one.
2) Acquire a 32/64 GB Micro SD Card, install [Raspberry PI Imager](https://www.raspberrypi.com/software/), insert SD card into your computer.
3) Select the Raspberry PI 4 option, you want to install Ubuntu on whatever the latest
version of ROS 2 is compatible with, for me Jazzy/Ubuntu 24.04 **Desktop**. We choose the
Desktop variant b/c we assume that we still don't have an IoT waiver. It is painstakingly
impossible to setup an embedded device on Eduroam. Alternatively, if you have access to the IoT network
or a home network w/no restrictions select the **Server**.
4) Slot the SD card from your PC into the Turtlebot's PI. You only need 3 connections to the PI:
a USB for a keyboard, an HDMI0 connection (on the left side of the board) for a HDMI display for 
which you'll probably need an adapter. Then the power board stemming from the underside base battery.
5) Place on the charger and the PI should boot up, otherwise hit the power button. After a 
few seconds the Ubuntu OS should begin to appear on your secondary monitor.
6) Turn on your phone's hotspot, any form of connection to the internet will work in place
of this, typically a home wifi network wouldn't have the restrictions that Eduroam does, so
something like that will also work.
7) In the bootup menu, you will want to connect your Turtlebot's OS to your phone so we
can briefly access the internet for setup purposes, beyond that we'll be able to use Eduroam.
8) Navigate to the [turtlebot4_setup Github](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html), which contains
a script you'll want to run after you've connected to the internet. As the guide mentions, after
the install script passes, run `sudo reboot` then `turtlebot4-setup`. This will create the
Turtlebot4 wifi that we need to get this running.


## Once Turtlebot4 Has been setup
1) ssh into the turtlebot4, if you're on a local network, you can use the command
`ssh your_hostname@192.xxx.xxx.xxx` where 192.xxx is the IP of the turtlebot which you can 
grab off your computers `ipconfig` or an app like `Fing`. Sign in with the given
user/pass which you created when flashing the SD. You then want to run `turtlebot4-setup` and
configure the network to the network you'll be using. 


## Configuration Settings for each of the Turtlebots

### Turtlebot1
Username of Turtlebot1: ubuntu
Password of Turtlebot1: ubuntu
IP Address of Turtlebot1: 192.168.8.108 (on BadBunny)
PI's MAC Address: e4:5f:01:cf:18:f5
ROS_DOMAIN_ID=5

### Turtlebot2
Username of Turtlebot2: ubuntu
Password of Turtlebot2: ubuntu
IP Address of Turtlebot2: 192.168.xxx.xxx (on BadBunny)
ROS_DOMAIN_ID=5


### "PS4" Controller
MAC Address: A0:5A:5C:E5:66:A1

