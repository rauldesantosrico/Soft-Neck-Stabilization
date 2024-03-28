#include "FPDBlock.h"

//FPDBlock::FPDBlock()
//{
//    FPDBlock(1, 1, 1, 0.001);
//}

///
/// \brief Block class implementing a fractional order controller following the transfer function
/// FOC(s)=kp+kd·s^fex.
/// \param new_kp: Proportional controller gain.
/// \param new_kd: Operator controller gain.
/// \param new_fex: Operator frational exponent. Negative exponents define fractional integrators while
/// positive exponent define fractional derivative operator.
/// \param new_dts: Sampling time considered in the controller loop.
///
FPDBlock::FPDBlock(double new_kp, double new_kd, double new_fex, double new_dts)
{
    kp=new_kp;
    kd=new_kd;
    fex=new_fex;
    dts= new_dts;
    s_e = FractionalController1DOF(fex,dts);
    maxOut=0;
    minOut=0;
    antiwindup=0,minWindup=0,maxWindup=0;

}

double FPDBlock::OutputUpdate(double new_input)
{
//    double cp,ci,cd;
    input=new_input;

    cp=input*kp;


    if(antiwindup == true)
    {
//        dci=kd*dts*input; //forecast value of ci change.
    //    ci = ci+dci; //forecast value of ci.
        state=cp+cd; //forecast value of state.

        if (state > maxWindup)
        {
            cd = kd*s_e.OutputUpdate(input*maxWindup/state);
//            cd = kd*s_e.OutputUpdate(input+dts*(maxWindup-state));

//            ci=max(0.0,maxWindup-cp-cd); //get the available input for the integrator which is not negative
            cerr << "Preventing controller windup: " << state << " ,s_e.GetState() " << s_e.GetState() << endl;

        }

        if (state < minWindup)//two if more efficient??
        {
            cd = kd*s_e.OutputUpdate(input*minWindup/state);
//            cd = kd*s_e.OutputUpdate(input+dts*(minWindup-state));

//            ci=min(0.0,minWindup-cp-cd); //get the available input for the integrator which is not positive
            cerr << "Preventing controller windup: " << state << " ,s_e.GetState() " << s_e.GetState() << endl;

        }

        if ((state < maxWindup)&&(state > minWindup) )
        {
            cd = kd*s_e.OutputUpdate(input);
//            cerr << "Preventing controller windup: " << state << " ,s_e.GetState() " << s_e.GetState() << endl;

        }

//        cerr << "Preventing controller windup: " << state << " ,iBlock.GetState(): " << iBlock.GetState() << endl;
//        cerr << "Preventing controller windup: " << state << " ,available: " << available << endl;
//        cerr << "Preventing controller windup: " << state << " ,iBlock.GetState(): " << iBlock.GetState() << endl;


    }
    else
    {
        cd = kd*s_e.OutputUpdate(input);
    }

    state=cp + cd;

    //apply saturation
    if (maxOut!=0)
    {
        state = std::min(state,maxOut);
    }
    //apply saturation
    if (minOut!=0)
    {
        state = std::max(state,minOut);
    }

//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "state : " <<state << std::endl;

    if (!isnormal(state)) return 0;
    return state;

}

long FPDBlock::AntiWindup(double minPlantSaturation, double maxPlantSaturation)
{
    minWindup = minPlantSaturation;
    maxWindup = maxPlantSaturation;
    if (fex<0)
    {
        antiwindup = true;
    }
    else
    {
        cerr << "Windup valid for integrators only (fractional exponent <0)" << endl;
    }
//    iBlock.SetSaturation(minPlantSaturation, maxPlantSaturation);
    cout << "Using integrator Anti-Windup." << endl;
    return 0;
}

///
/// \brief Update the controller parameters.
/// \param new_params: An std::vector containing the new controller parameters in the same order as
/// the constructor (kp,kd,fex).
/// \return 0 if correct.
///
long FPDBlock::ParameterUpdate(vector<double> new_params)
{
    kp=new_params[0];
    kd=new_params[1];
//    if ( abs(fex-new_params[2])>0.05 )
    {

        fex=new_params[2];
        s_e_state = s_e.GetState();
        std::cout << "s_e_state : " <<s_e_state << std::endl;

        s_e = FractionalController1DOF(fex,dts);
        s_e.Reset(s_e_state);

    }

    return 0;
}

double FPDBlock::GetState() const
{
    return state;
}

long FPDBlock::PrintParameters()
{
        std::cout << "kp : " << kp << ", kd: "<< kd << ", fex: "<< fex << std::endl;
        return 0;
}

long FPDBlock::GetParameters(double &out_kp, double &out_kd, double &out_fex)
{
    out_kp = kp;
    out_kd = kd;
    out_fex = fex;
    return 0;

}

