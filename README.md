# VelodyneVLP16_HALandObstacleFinder

  While there exist many hardware abstraction programs to interact with the Velodyne VLP16 layer, they all depend heavily on other tech stacks such as ROS. This is not 
very convenient for competions such as FIRST Robotics wher competitiors are required to use computers with limited RAM or for unexperienced programmers. My goal 
with the HAL shown here is to develope a simple and stand-alone driver for interacting with the lidar. Some of the functions in the PacketDecoder module are specifically
tailored to our setup (lidar tilted and facing the ground), but it should be fairly simple to adapt for other uses.

  This repo is specifically developed in Java to one, help me become more familiar with the lenguage, and two, to allow less experince developers the freedom to not have to
deal with "garbage collection" that is necessary when developing with C++. The team took a large risk with this however. We gave up a large portion of our RAM to the JVM, but 
it also allowed me to developed the code necessary under a tight deadline. I hope to in the future develop a similar program, but in C++ to free up more of the limited amount 
of RAM availible in the on-board computer (256 MB).

#Hardware

This folder contains the actual HAL. It comes with a PacketDriver class to be used as the way to bind to the Lidar's socket, and with a PacketDecoder class to processes the
packets comming from the Lidar. Below is a picture from Velodyne's manual which showcases the content of each package. It will be helpful when reading my code.

![Lidar's Data Package](https://your-copied-image-address)
