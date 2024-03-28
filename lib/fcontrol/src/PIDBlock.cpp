#include "PIDBlock.h"

using  namespace std;
long PIDBlock::Initial(double new_Ts)
{

    Ts=new_Ts;
        std::cout << "Ts : " <<Ts << std::endl;
    iBlock = SystemBlock(
                std::vector<double> {Ts/2,Ts/2},
                std::vector<double> {-1,1});
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1});

        // LPF implementation
    double N = 20;    // LPFfilter N
    dBlock = SystemBlock(
                std::vector<double> {-1*N,1*N},
                std::vector<double> {-1,1+N*Ts*1});
//                std::vector<double> {-1,  1},
//                std::vector<double> {0,      Ts*1});
    return 0;
    cp=0;
    ci=0;
    cd=0;
    available=0;
    dci=0;
    di=0;

}

PIDBlock::PIDBlock()
{

    Initial(0.01);

    kp=1;
    ki=1;
    kd=1;

}

PIDBlock::PIDBlock(double new_kp, double new_ki, double new_kd, double new_Ts)
{
    Ts=new_Ts;
    Initial(new_Ts);

    kp=new_kp;
    ki=new_ki;
    kd=new_kd;


}

double PIDBlock::UpdateControl(double input)
{

    cp = input*kp;
    cd = kd*dBlock.OutputUpdate(input);

    dci=ki*Ts*input; //forecast value of ci change.
//    ci = ci+dci; //forecast value of ci.
    state=cp+ci+cd;//+dci; //forecast value of state.

    if(antiwindup == true)
    {

        if (state > maxWindup)
        {
            q=maxWindup/state; //looks like this strategy is better...
            eqinput = 2*q*q/(state+maxWindup); //it is a double needed? 2*2*q*q?
            ci = ki*iBlock.OutputUpdate(input*q);
//            ci=max(0.0,maxWindup-cp-cd); //get the available input for the integrator which is not negative
            cerr << "Preventing controller windup: " << state << " ,iBlock.GetState(): " << iBlock.GetState() << endl;

        }

        if (state < minWindup)//two if more efficient??
        {
            q=minWindup/state; //looks like this strategy is better...
            eqinput = 2*q*q/(state+minWindup);
            ci = ki*iBlock.OutputUpdate(input*q);
//            ci=min(0.0,minWindup-cp-cd); //get the available input for the integrator which is not positive
            cerr << "Preventing controller windup: " << state << " ,iBlock.GetState(): " << iBlock.GetState() << endl;

        }

        if ((state < maxWindup)&&(state > minWindup) )
        {
            ci = ki*iBlock.OutputUpdate(input);
        }

//        cerr << "Preventing controller windup: " << state << " ,iBlock.GetState(): " << iBlock.GetState() << endl;
//        cerr << "Preventing controller windup: " << state << " ,available: " << available << endl;
//        cerr << "Preventing controller windup: " << state << " ,iBlock.GetState(): " << iBlock.GetState() << endl;


    }
    else
    {
        ci = ki*iBlock.OutputUpdate(input);
    }

    state=cp+ci+cd; // Final controller state
//    cerr << "cp: " << cp << " ,cd: " << cd  << " ,ci: " << ci  << " ,state: " << state  << endl;


    //reset antiwindup, not use.
//    cout << ci << "; ";
//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }




    //apply saturation
    if(saturation == true)
    {
        if (state > maxOut)
        {
            state = maxOut;
            cerr << "Top saturation!! output: " << state << " ,maxOut" << maxOut << endl;
        }
        //apply saturation
        if (state < minOut)
        {
            state = minOut;
            cerr << "Bottom saturation!! output: " << state << " ,maxOut" << minOut << endl;
        }
    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "pid : " <<state << std::endl;

    return state;

}

double PIDBlock::UpdateControlILock(double input)
{

    cp = input*kp;
//    ci = ki*iBlock.OutputUpdate(input);
    cd = kd*dBlock.OutputUpdate(input);

//    cout << ci << "; ";
//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }


    state=cp+ci+cd;


    //apply saturation
    if(saturation == true)
    {
        if (state > maxOut)
        {
            state = maxOut;
            cerr << "Top saturation!! output: " << state << " ,maxOut" << maxOut << endl;
        }
        //apply saturation
        if (state < minOut)
        {
            state = minOut;
            cerr << "Bottom saturation!! output: " << state << " ,maxOut" << minOut << endl;
        }
    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "pid : " <<state << std::endl;

    return state;
}

double PIDBlock::OutputUpdate(double input)
{

    return UpdateControl(input);
}

long PIDBlock::AntiWindup(double minPlantSaturation, double maxPlantSaturation)
{
    minWindup = minPlantSaturation;
    maxWindup = maxPlantSaturation;
    antiwindup = true;
//    iBlock.SetSaturation(minPlantSaturation, maxPlantSaturation);
    cout << "Using integrator Anti-Windup." << endl;
    return 0;
}

double PIDBlock::GetState() const
{
    return state;
}


//void PIDBlock::operator>>(double &output)
//{
//    output = state;

//}

//void PIDBlock::operator>>(SystemBlock &output)
//{
//    output.OutputUpdate(state);

//}
