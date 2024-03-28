#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include <iostream>

#include "imu3dmgx510.h"
#include "fcontrol.h"

#include <chrono>
#include <thread>

#define LG0 0.003 //meters
#define radio 0.0075

#include <random>

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
    std::vector<double> pose(2);
    std::vector<double> lengths(3);
    std::vector<double> posan(3);

    // file results
    ofstream csvFile;
    csvFile.open("../data.csv", ofstream::out);
    csvFile << "Alpha, Beta, IMU-Pitch, IMU-Roll"<< endl;

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    for(int i=0; i<200; i++)
    {
        imu.GetPitchRollYaw(pitch,roll,yaw);
    }

    printf("IMU Calibrada :) \n");

    // tiempo actual
    time_t inicio = time(NULL);
    time_t actual;


    int alpha = 0;
    int beta = 0;

    printf("Inicio\n");

    for(alpha=-40; alpha<=40; alpha+=4)
    {
        for(beta=-40; beta<=40; beta+=4)
        {

            pose[0] = alpha;
            pose[1] = beta;

            pr2tendons(pose[0], pose[1], lengths);

            posan[0] = (LG0 - lengths[0])/radio;
            posan[1] = (LG0 - lengths[1])/radio;
            posan[2] = (LG0 - lengths[2])/radio;

            m1.SetPosition(posan[0]);
            m2.SetPosition(posan[1]);
            m3.SetPosition(posan[2]);

            // para que le de tiempo a retornar a la posición beta= -40
            if(beta==-40)
            {
                printf("retornando a la posición...\n");
                while( time(NULL)-inicio < 2 ) // 2 sec
                {
                    imu.GetPitchRollYaw(pitch,roll,yaw);
                }

                inicio  =  time(NULL);
            }
            else
            {
                while( time(NULL)-inicio < 1 ) // 1 sec
                {
                    // Leo IMU
                    imu.GetPitchRollYaw(pitch,roll,yaw);
                }
            }

            imu.GetPitchRollYaw(pitch,roll,yaw);

            printf("Pose: [%f] [%f] -> IMU: [%f] [%f] \n", pose[0], pose[1], pitch, roll);
            csvFile << alpha << "," << beta << "," << pitch << "," << roll << endl;

            //reseteo startTime
            inicio  =  time(NULL);
         }
     }

    printf("Finished :)\n");
    return 0;
}

