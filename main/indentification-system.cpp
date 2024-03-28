#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include "fcontrol.h"
#include <iostream>
#include <chrono>
#include <thread>


#define LG0 0.003 //meters
#define TRADIO 0.0075 // radio del tambor
#define PRADIO 0.05 // radio de la plataforma

//-------------------------------------------------------------------------

// pitch, roll to angular velocity in radians/sec
void pr2tendons(double pitch, double roll, std::vector<double> & vel)
{
    double T  = PRADIO / TRADIO;
    vel[0] =  pitch * T;
    vel[1] =  roll * T * sin(2*M_PI/3) + pitch * T * cos(2*M_PI/3);
    vel[2] =  roll * T * sin(4*M_PI/3) + pitch * T * cos(4*M_PI/3);
}

int main ()
{
    // control loop sampling time
    double freq=50; // F=80 para T=0.02. F=50 para T=0.03
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // configuring IMU
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    cout<<"Calibrating IMU..."<<endl;

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }

    cout<<"Calibrated!"<<endl;

    // file results
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file,"TargetPitch,SensorPitch,TargetRoll,SensorRoll\n");

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);

    //m2 setup
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);
    m2.StartNode();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(10);

    //m3 setup
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);
    m3.StartNode();
    m3.SwitchOn();
    m3.Setup_Velocity_Mode(10);

    // controller for motor
    PIDBlock cntrl1(0.015,36,0,dts);
    PIDBlock cntrl2(0.015,36,0,dts);
    PIDBlock cntrl3(0.015,36,0,dts);

    std::vector<double> pose[2];
    std::vector<double> targetVel(3);

    pose[0] = { 0, -0.17};
    pose[1] = { 0,  0.17};

    for(int i=0; i<3; i++)
    {
        for(int p=0; p<2; p++){

            printf("---- moving to pose [%d]: [%f] [%f]\n", p, pose[p][0], pose[p][1]);

            for(double t=0; t<=2; t+=dts){ // cada pose la recorre en 2seg

                pr2tendons(pose[p][0], pose[p][1], targetVel);


                // Controller of velocity in M1

                double currentVelM1 = m1.GetVelocityTP();

                double velError1 = targetVel[0] - currentVelM1;

                // Control process
                double cS1 = cntrl1.OutputUpdate(velError1);

                if (!std::isnormal(cS1))
                {
                    cS1 = 0.0;
                }

                m1.SetVelocity(cS1);

                // Controller of velocity in M2

                double currentVelM2 = m2.GetVelocityTP();

                double velError2 = targetVel[1] - currentVelM2;

                // Control process
                double cS2 = cntrl2.OutputUpdate(velError2);

                if (!std::isnormal(cS2))
                {
                    cS2 = 0.0;
                }

                m2.SetVelocity(cS2);

                // Controller of velocity in M3

                double currentVelM3 = m3.GetVelocityTP();

                double velError3 = targetVel[2] - currentVelM3;

                // Control process
                double cS3 = cntrl3.OutputUpdate(velError3);

                if (!std::isnormal(cS3))
                {
                    cS3 = 0.0;
                }

                m3.SetVelocity(cS3);

                imu.GetPitchRollYaw(pitch,roll,yaw);

                // Sensor
                fprintf(file,"%.4f,%.4f,",  pose[p][0], - pitch);
                fprintf(file,"%.4f,%.4f\n",  pose[p][1], - roll);

                Ts.WaitSamplingTime();

            }


        } // for que recorre las 2 poses
    } // repite el ciclo 3 veces

    
    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    return 1;
}
