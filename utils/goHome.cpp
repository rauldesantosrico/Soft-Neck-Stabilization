#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include "fcontrol.h"
#include <iostream>

#define LG0 0.003 //meters
#define TRADIO 0.0075 // radio del tambor
#define PRADIO 0.05 // radio de la plataforma

/* Programa para tensar hilos */

// tense tendons:
//  true: it's stopped and tensed
bool tenseThread(CiA402Device motor){
    motor.Setup_Torque_Mode();
    motor.SetTorque(0.01); //0.07
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    while(motor.GetVelocity() > 0.2) //0.2
        printf("Tensing thread... vel: %f\n", motor.GetVelocity());
    motor.Setup_Velocity_Mode(0);
    return true;
}

//-------------------------------------------------------------------------
/* Calibrado de los hilos y control en posicion 0 de pitch y roll
 * haciendo uso de controladores en velocidad externos de cada uno de los drivers */

// pitch, roll to velocity in rad/s
void pr2tendons(double pitch, double roll, std::vector<double> & vel)
{
    double T  = PRADIO / TRADIO;
    vel[0] =  pitch * T;
    vel[1] =  roll * T * sin(2*M_PI/3) + pitch * T * cos(2*M_PI/3);
    vel[2] =  roll * T * sin(4*M_PI/3) + pitch * T * cos(4*M_PI/3);
}

int main ()
{
    // inputs
    vector<double> targetPose{0, 0}; // HOME
    std::vector<double> targetVel(3);

    // control loop sampling time
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

    bool tensed = false;

    // configuring IMU
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch=1,roll=1, yaw;

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);

    //m2 setup
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);

    //m3 setup
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);


    tensed &= tenseThread(m1);
    tensed &= tenseThread(m2);
    tensed &= tenseThread(m3);

    if( tensed )
        printf("tendons tensed successfully\n");

    // controller for motor
    PIDBlock cntrl1(0.015,36,0,dts);
    PIDBlock cntrl2(0.015,36,0,dts);
    PIDBlock cntrl3(0.015,36,0,dts);

    // -- Control fraccionario
    //test 9: FPD w=3, pm=50
    //        Pitch: alpha =-0.8500, kp = 2.5773, ka = 3.2325
    //        Roll:  alpha = -0.8600, kp = 2.6299, ka = 3.2395
    FPDBlock fcPitchVelocity(2.5773, 3.2325, -0.8500, dts);
    FPDBlock fcRollVelocity(2.6299, 3.2395, -0.8600, dts);

    for(int i; i<100; i++)
    {
        imu.GetPitchRollYaw(pitch,roll,yaw);
        //printf("pitch: %f roll: %f\n", pitch, roll);
        //printf("Inicializando IMU: %d\n", i);
    } 

    while((pitch>0.005 || pitch<-0.005) || (roll>0.005 || roll<-0.005)){

        // ---- General Control process ----
        imu.GetPitchRollYaw(pitch,roll,yaw);
        printf("pitch: %f roll: %f\n", pitch, roll);

        // cambio signo para igualar sentido de giro de los motores y del sensor
        pitch = -pitch;
        roll  = -roll;

        double pitchError = targetPose[0] - pitch;
        double rollError  = targetPose[1] - roll;

        double pitchCs = fcPitchVelocity.OutputUpdate(pitchError);
        if (!std::isnormal(pitchCs))
        {
            pitchCs = 0.0;
        }

        double rollCs = fcRollVelocity.OutputUpdate(rollError);
        if (!std::isnormal(rollCs))
        {
            rollCs = 0.0;
        }

        pr2tendons(pitchCs, rollCs, targetVel);

        // -----------------------------------

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

        Ts.WaitSamplingTime();        
    }

    printf("HOME position reached!\n");

    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    return 0;
}
