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
/* Calibrado de los hilos y control en posicion 0 de pitch y roll
 * haciendo uso de controladores en velocidad externos de cada uno de los drivers */

// pitch, roll to velocity in meters/sec
void pr2tendons(double pitch, double roll, std::vector<double> & vel)
{
    double T  = PRADIO / TRADIO;
    vel[0] =  pitch * T;
    vel[1] =  roll * T * sin(2*M_PI/3) + pitch * T * cos(2*M_PI/3);
    vel[2] =  roll * T * sin(4*M_PI/3) + pitch * T * cos(4*M_PI/3);

    //printf("Final angular velocities: %f %f %f\n", vel[0], vel[1], vel[2]);
}

// tense tendons:
//  true: it's stopped and tensed
bool tenseTendons(CiA402Device motor){
    motor.Setup_Torque_Mode();
    motor.SetTorque(0.01); //0.07
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    while(motor.GetVelocity() > 0.2) //0.2
        printf("Tensing thread... vel: %f\n", motor.GetVelocity());
    motor.Setup_Velocity_Mode(0);
    return true;
}

int main ()
{

    // inputs
    vector<double> targetPose{0, 0}; // {pitch, roll: 0.18}

    std::vector<double> targetVel(3);
    bool tensed = false;


    // control loop sampling time
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

    // configuring IMU
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    // file results
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file,"Time, TargetPitch, SensorPitch, TargetRoll, SensorRoll, VelM1, VelM2, VelM3\n");

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

    //Init: tensing tendons
    tensed &= tenseTendons(m1);
    tensed &= tenseTendons(m2);
    tensed &= tenseTendons(m3);

    if( tensed )
        printf("tendons tensed successfully\n");

    // controller for motor
    PIDBlock cntrl1(0.015,36,0,dts);
    PIDBlock cntrl2(0.015,36,0,dts);
    PIDBlock cntrl3(0.015,36,0,dts);

    // test 10: PID w=3, pm=50
    //          Pitch: Kp = 3.15 , Ki = 1.73  , Kd = 0.193
    //          Roll:  Kp = 3.17, Ki = 1.74, Kd = 0.198
    PIDBlock pController(3.15, 1.73, 0.193, dts);
    PIDBlock rController(3.17, 1.74, 0.198, dts);

    auto start = std::chrono::system_clock::now();
    std::chrono::duration<float,std::milli> duration;

    while(1){

        // ---- General Control process ----
        imu.GetPitchRollYaw(pitch,roll,yaw);

        // cambio signo para igualar sentido de giro de los motores y del sensor
        pitch = -pitch;
        roll  = -roll;

        double pitchError = targetPose[0] - pitch;
        double rollError  = targetPose[1] - roll;

        double pitchCs = pController.OutputUpdate(pitchError);
        if (!std::isnormal(pitchCs))
        {
            pitchCs = 0.0;
        }

        double rollCs = rController.OutputUpdate(rollError);
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

        auto current = std::chrono::system_clock::now();
        duration = current - start;

        // Escribimos en el archivo
        fprintf(file,"%.4f,", duration.count());
        fprintf(file,"%.4f, %.4f,",  targetPose[0], pitch); // pitch target, pitch sensor
        fprintf(file,"%.4f, %.4f,",  targetPose[1], roll); // roll target,  roll sensor
        fprintf(file,"%.4f, %.4f, %.4f\n", m1.GetVelocity(), m2.GetVelocity(), m3.GetVelocity()); // roll target,  roll sensor


        printf("pitch error: %f  roll error: %f \n", pitchError, rollError);
        Ts.WaitSamplingTime();        

    }

    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    m1.ForceSwitchOff();
    m2.ForceSwitchOff();
    m3.ForceSwitchOff();


    return 1;
}
