#include "imu3dmgx510.h"
#include <iostream>
#include <stdio.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

using yarp::os::Bottle;
using yarp::os::Network;
using yarp::os::Port;
using yarp::os::BufferedPort;
using namespace std;


class Sensor
{
    private:
        IMU3DMGX510 * imu;
        double pitch,roll, yaw;
    public:
        Sensor(double f);
        std::vector<double> read();
        void calibrate();
};

Sensor::Sensor(double f){
    imu = new IMU3DMGX510("/dev/ttyUSB0",f);
}

std::vector<double> Sensor::read(){
    imu->GetPitchRollYaw(pitch,roll,yaw);
    std::vector<double> result{pitch,roll};
    return result;
}

void Sensor::calibrate(){
    printf("Calibrating...\n");
    for(int i=0; i<100; i++){
        imu->GetPitchRollYaw(pitch,roll,yaw);        
    }
    printf("Calibrated\n");
}

int main(){
    Network yarp; 
    Port output;
    Bottle outBottle;

    BufferedPort<Bottle> input;
    input.open("/receiver");
    output.open("/sender");
    Sensor s(50);
    s.calibrate();

    while(1){
        std::vector<double> value = s.read();
        //printf("while > %f %f\n", value[0], value[1]);

        Bottle* inBottle = input.read(false);
        if(inBottle!=NULL){
            cout << inBottle->get(0).asString() << endl;
            if(inBottle->toString()=="sendme")
            {
                outBottle.clear();
                outBottle.addFloat64(value[0]);
                outBottle.addFloat64(value[1]);
                //sleep(0.1);
                output.write(outBottle);
                inBottle->clear();
                
            }
            if(inBottle->toString()=="exit")
                break;
        } 

    }
}


