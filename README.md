# Humanoid head camera stabilization using a soft robotic neck and a robust fractional order controller
Code repository of the identification and control of a soft neck. This code has been tested on an Ubuntu 22.04 LTS operating system.
## 1. Library and dependencies installation:
```bash
$ sudo apt install libeigen3-dev
$ sudo apt install libboost1.74-all-dev
$ sudo apt install libqt5serialport5-dev
```
Note: To perform the experiment with full robot integration, [Yarp](https://www.yarp.it/latest/install_yarp_linux.html) needs to be installed.

## 2. Compilation:
```bash
$ git clone https://github.com/rauldesantosrico/soft-neck-camera-stabilization.git
$ cd soft-neck-camera-stabilization/
$ mkdir build
$ cd build/
$ cmake ..
$ make -j$(nproc)
```
