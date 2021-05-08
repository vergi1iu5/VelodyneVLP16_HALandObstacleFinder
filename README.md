# VelodyneVLP16_HALandObstacleFinder

  While there exist many hardware abstraction programs to interact with the Velodyne VLP16 layer, they all depend heavily on other tech stacks such as ROS. This is not 
very convenient for competions such as FIRST Robotics where competitiors are required to use computers with limited RAM or for unexperienced programmers. My goal 
with the HAL shown here is to develope a simple and stand-alone driver for interacting with the lidar. Some of the functions in the PacketDecoder module are specifically
tailored to our setup (lidar tilted and facing the ground), but it should be fairly simple to adapt for other uses.

  This repo is specifically developed in Java to one, help me become more familiar with the lenguage, and two, speed up development and testing of software. The team took a large risk with this however. We gave up a large portion of our RAM to the JVM, but  it also allowed me to developed the code necessary under a tight deadline. I hope to in the future develop a similar program, but in C++ to free up more of the limited amount of RAM availible in the on-board computer (256 MB).

# Hardware

This folder contains the actual HAL. It comes with a PacketDriver class to be used as the way to bind to the Lidar's socket, and with a PacketDecoder class to processes the
packets comming from the Lidar. Below is a picture from Velodyne's manual which showcases the content of each package. It will be helpful when reading my code.

![Lidar's Data Package](https://github.com/vergi1iu5/VelodyneVLP16_HALandObstacleFinder/blob/main/doc/images/data_packet.JPG)

Finally there is the actual VelodyneLidar class which is to serve as the top-most layer of abstaraction. This class can be used to scan the full field of view (FOV), start and stop
the lidar, set field of view (work in progress), and analyze frame for any present obstacles. Read the reference manual for more information for setting up the hardware abstarction layes.

# TerrainAnalysis

This module integrates to the HAL module to take in data from the Lidar and look for any obstacles in the way. Once the ObstacleFinder class has been initialized, all you need to do is feed in a Frame coming from the PacketDecoder. If any Obstacles are found within the frame, it will provide them through the getter functions. My goal was to keep the algorithms as simple as possible to allow them to run in the enviroment provided by the original team. In essence, all it does is compare the current field to a flat plane. If a flat plane has not been scanned (helps with RAM conmsuption), then the lidar can still detect obstacles through point-cloud calculations.

This module also contains the Obstacles it is able to detect as extensions of the Obstacle class. You can further extend this class to add other types of Obstacles. My current goal is to add the ability to detect a ramp and count it as a surface for the rover to navigate. This task has proven to be a lot more difficult than expected. 

As a final note for this section: I only added part of the algorith the team planned on using. The team is planning on reusing it next year so I can not reveal their secrets.

# Threads

Finally, I added a simple module to allow the use of all the other modules in a multi-threaded system. The file contains an example of how to start and interact with the thread. I was not able to fully test this module so any PR request and comments are welcomed.
