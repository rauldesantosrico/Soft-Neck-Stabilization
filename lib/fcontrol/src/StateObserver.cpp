#include "StateObserver.h"

StateObserver::StateObserver()
{

}

StateObserver::StateObserver(vector<vector<double> > G, vector<vector<double> > H, vector<vector<double> > C, vector<vector<double> > D, double dts):
    StateSpace(G, H, C, D, dts)
{

    hatx.setZero(n_state,1);
    haty.setZero(n_output,1);
    erry.setZero(n_output,1);
    gainsK.setOnes(n_state,1);
//    this->u.setZero(H.cols(),1);

}

long StateObserver::SetGainsK(vector<vector<double> > new_gainsK)
{
    gainsK=vector2matrix(new_gainsK);
    return 0;

}

double StateObserver::Update(double plantIn, double plantOut)
{


    haty(0)=output(plantIn);
    erry(0)=plantOut-haty(0);
    x=x+(gainsK*erry(0)); //Maybe wrong to access x from here. Should try to do better

    return 0;


}
