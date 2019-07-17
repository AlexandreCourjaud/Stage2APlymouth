[![Current version on Arduino](https://img.shields.io/badge/Arduino-v1.8.5-blue.svg)](https://www.arduino.cc/en/Main/Software)

# Autonomous Sailboat 

C++ and Python code for Plymouth's Autonomous Sailboat. This contains the catkin workspace of the nodes for the sailboat. And the code for Arduino.
This project is done in collaboration with :
https://github.com/Matthix7/plymouth_internship_2019
https://github.com/corentin-j/WRSC_plymouth_JEGAT


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

- Arduino IDE : https://www.arduino.cc/en/Main/Software
- Arduino Mega 2560 : (tested and working)
- Connected to a Raspberry Pi 3 containing our ROS packages
(https://github.com/AlexandreCourjaud/Stage2APlymouth
https://github.com/Matthix7/plymouth_internship_2019
https://github.com/corentin-j/WRSC_plymouth_JEGAT)
- Install gps-common (sudo apt-get install ros-melodic-gps-common)



### Building and Uploading

First copy the folder `/trimaranArduinoInterface/libraries` and put it in `~/Documents/Arduino/` on your PC.

You should now be able to launch the *.ino* sketch, compile and upload to the arduino using the [Arduino IDE](https://www.arduino.cc/en/Guide/ArduinoMega2560).
This will launch the Arduino, just receiving data from the sensors and sending them to the PC. To know the connections, see in the file TrimaranArduinoInterface
It will put the rudder and the sail at angle 0.


## Authors

* **Alexandre COURJAUD** - *Initial work* - [AlexandreCourjaud](https://github.com/AlexandreCourjaud)

## Acknowledgments
This project uses well-known libraries from :
* [TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)

