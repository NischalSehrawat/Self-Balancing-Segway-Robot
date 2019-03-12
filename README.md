# Balancing-Robot-RaspberryPi [(Current Status)](https://www.youtube.com/watch?v=qLhnLk07C0s&list=PLveRMPt4kAsA41ivMscrzFSWWx54CI52J&index=7&t=0s)


This is my project for balancing an inverted pendulum using an Arduino Mega and Raspberry Pi. In my [earlier ](https://www.youtube.com/watch?v=-TRXWSr9_dE&list=PLveRMPt4kAsA41ivMscrzFSWWx54CI52J&t=0s&index=2) approach I was successful in making the robot balance but in order to move it like a **Segway**, it needs more thorough analysis. Therefore I decided to go into greater details and study it more thoroughly.  

## Objectives and theory

The main aim of the project is to learn about the following topics
- Gain an insight into the physics of the problem and understand the equation of motions. I have used [this paper](https://content.sciendo.com/view/journals/meceng/61/2/article-p331.xml) and followed [this course](https://www.coursera.org/learn/mobile-robot). 
- Learn about classic and modern control theories. [Source](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
- Learn about concepts like controllability, observability, LQR technique to derive controller gain. [Source](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
- Learn and code my own Kalman Filter for Sensor Fusion. I struggled a lot to find sources that explained it from the very basics and finally found some good reads. Sources [1](https://home.wlu.edu/~levys/kalman_tutorial/), [2](http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/) and [3](https://github.com/balzer82/Kalman/blob/master/Kalman-Filter-CV.ipynb?create=1).
- Check how do Kalman and Complimentary filters compare with each other.
- Learn how to implement the concepts learnt on physical hardware and understand the problems encountered.
- Learn how to make RaspberryPi communicate with arduino over Serial port. [Source](http://forum.arduino.cc/index.php?topic=396450)
- Learn how to use rotory encoders to get RPM of a motor and smoothen the data. Info about using encoders can be found [here](https://www.youtube.com/watch?v=oLBYHbLO8W0). For smoothing RPM data I have used exponential averaging.
- Info about dc motor control (using 2 wires instead of 3 for speed control) can be found [here](https://www.bluetin.io/python/gpio-pwm-raspberry-pi-h-bridge-dc-motor-control/).  However for robot control it is easier to use **Robot** builtin class on gpiozero library on RasPi. Info about using various builtin classes on RaspPi can be found [here](https://projects.raspberrypi.org/en/projects/physical-computing/16). Info about using MPU6050 library on a RaspberryPi can be found [here](https://libraries.io/pypi/mpu6050-raspberrypi).
- An important (but a bit late) observation in this project was that while the Raspberry Pi is significantly faster than an Arduino, but since it runs an OS, it cannot be used for realtime control that requires precise timing. For example,when the potentiometer data was sent over Serial port to the Pi from Arduino, the Pi responded very late and to compensate that, the data was sent to the Pi at 20 mS interval instead of 1-2 ms. This will slow down the control loop considerably. Therefore, the main balancing task is be performed on the Arduino board, while the Pi will be used for higher level tasks such as decision making.

## Implementation on Arduino: Problems, findings and solutions.

![IMG_20190310_165336_Bokeh](https://user-images.githubusercontent.com/33701903/54087989-3c0ac580-4359-11e9-92f2-298b7dcfcb00.jpg)

A lot of problems were encountered and solved during the implementation on a microcontroller. Problems / findings are listed below in decreasing order of importance.

- **MPU Caliberation**: Make sure that the MPU6050 is correctly caliberated . It is very important to note that since the MPU won't be located at the exact center of the chassis, therefore it must be checked that when the robot is vertical, the measured angle does not vary more than +/- 0.5 deg. If it does, add a correction factor to it to bring it closer to "0". In my case the correction was **2.5 Deg**. The weighting parameter **alpha** in complimentary filter depends upon the loop time. So experiment with alpha depending upon loop time. I found that **alpha = 0.98** was a good choice.

- **Derivative term Balancing PID**: When calculating the derivative part of the balancing PID algorithm, the rate of change of angle should be used as is directly given by the **gyroscope gyrX**. If the derived rate of change of angle (from the complimentary filter) is used, then it will fluctuate a lot and the robot won't be stable. Using the gyroX value directly gives a **massive** improvement to the stability.

- **Baudrates**: Serial baudrate must be higher than **115200**. It won't work at all if baudrate is set at 9600; If using HC-05 bluetooth modules, change its baudrate to 115200 as the default is 9600 as shown [here](https://www.youtube.com/watch?v=zrp-GBwOAQQ). 

- **Loop time**: The main control loop must be very fast. However, if it is too fast then the motors might not have sufficient time to react. I experimented with the loop time starting from **3 ms** (I had used my Kp and Kd values determined earlier) and I observed that at very low loop times (<10 ms) the robot was very jittery. Then I started increasing the loop time and I found that between **20 and 30 ms** it became very stable. Therefore the current loop time is set to **20 ms**.  

- **Motor speed / torques / wheel sizes**: While trying to make the robot move, it was realised that motors having **high torque and high speed** are required. In the first attempt I tried with motors having high torqure but low RPM (**107 RPM** with a gear ratio of **1:90**). Torque is required to carry the weight of the robot. But, since for balancing only a small amount of motor rotation is required, this will work. However in order to be able the move, the robot must quickly try to come under the position of center of gravity of the top frame. Thus a new gearbox with a gear ratio of **1:30** was used to increase the motor **RPM to 350**. Also, bigger diameter wheels (**upgraded from 85mm to 130mm**) were used. Indeed, the robot is able to catchup with the centre of gravity

- **Cascade Control**: I used a cascade control loop to keep the robot stable. The outer loop comes from linear velocity of the wheels (omega * r_wheel). This makes sure that the robot does not start moving on its own in forward or backward direction due the fact that the parts are not mounted symmetrically and it is imbalanced.  

![HCUEV](https://user-images.githubusercontent.com/33701903/54088220-fa2f4e80-435b-11e9-9dd7-16c272d76783.png)

Algorithm to try: Since the robot needs correction to move in a straight line, the following algorithm will be tried out

![Control](https://user-images.githubusercontent.com/33701903/54217804-9ab47880-44ec-11e9-9ca5-8b65ceeb794e.PNG)


- **Connecting Arduino to RaspberryPi**: It is very frustrating to connect the Arduino to the computer with a USB cable each time some piece of code needs to be updated. Therefore, it is very hand to install a RaspberryPi and connect the Arduino to it so that by using **VNC viewer** or **TeamViewer**, we can remotely log into the RaspberryPi and program the Arduino. But is it very important that you donot install the Arduino IDE from the command line using **sudo apt-get install arduino** because this throws a lot of errors and even the simplest codes (like blink) donto get uploaded to the Arduino. Therefore we need to install Arduino IDE using [this link](https://www.raspberrypi.org/magpi/program-arduino-uno-raspberry-pi/). After this it works perfectly fine.
