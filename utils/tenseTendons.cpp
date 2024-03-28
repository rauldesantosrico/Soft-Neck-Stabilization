#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>

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

int main ()
{
    bool tensed = false;
    
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

    sleep(0.1);

    return 0;
}
