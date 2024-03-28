#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include <iostream>
#include <chrono>
#include <thread>

#include "imu3dmgx510.h"
#include "fcontrol.h"

#define LG0 0.003 //meters
#define radio 0.0075

/* Programa test en modo posicion del cuello en bucle abierto */

// pitch, roll to lenght of tendons in meters
void pr2tendons(double pitch, double roll, std::vector<double> & lengths)
{
    // pitch & roll -> lenghts
    lengths[0] = -0.001* (pitch/1.5);
    lengths[1] =  0.001* (pitch/3 - roll/1.732);
    lengths[2] =  0.001* (pitch/3 + roll/1.732);
}

// setPosition <pitch> <roll> (in rads)
int main (int argc, char *argv[])
{
    if(argc!=3)
    {
        printf("Error: setPosition <pitch> <roll>");
        return 1;
    }

    std::chrono::system_clock::time_point start, current;

    //m1
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    //m1.Reset();
    m1.SetupPositionMode(10, 10);

    //m2
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);
    //m2.Reset();
    m2.SetupPositionMode(10, 10);

    //m3
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);
    //m3.Reset();
    m3.SetupPositionMode(10, 10);


    // Input /output
    std::vector<double> pose{atof(argv[1]), atof(argv[2])};
    std::vector<double> lengths(3);
    std::vector<double> posan(3);


    start = std::chrono::system_clock::now();

    //printf("GetPosition -> %f %f %f\n", m1.GetPosition(), m2.GetPosition(), m3.GetPosition());


    printf("Moving to pose: [%f] [%f]\n", pose[0], pose[1]);

    pr2tendons(pose[0], pose[1], lengths);

    posan[0] = (LG0 - lengths[0])/radio;
    posan[1] = (LG0 - lengths[1])/radio;
    posan[2] = (LG0 - lengths[2])/radio;

    //printf("tendons: [%f] [%f] [%f]\n", lengths[0], lengths[1], lengths[2]);
    //printf("target angular pos : [%f] [%f] [%f]\n", posan[0], posan[1], posan[2]);

    m1.SetPosition(posan[0]);
    m2.SetPosition(posan[1]);
    m3.SetPosition(posan[2]);



    return 0;
}

