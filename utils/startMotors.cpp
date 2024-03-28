#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"

using namespace std;



int main()
{
    std::cout << "HOLA"<< std::endl;
    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1); 
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);
    m1.PrintStatus();

    //m2 setup
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);
    m2.StartNode();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(10);
    m2.PrintStatus();

    //m3 setup
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);
    m3.StartNode();
    m3.SwitchOn();
    m3.Setup_Velocity_Mode(10);
    m3.PrintStatus();

    return 0;
}
