#ifndef STATEOBSERVER_H
#define STATEOBSERVER_H

#include "StateSpace.h"

using namespace std;
using namespace Eigen;


class StateObserver : public StateSpace
{
public:
    StateObserver();
    StateObserver(vector<vector<double>> G, vector<vector<double>> H, vector<vector<double>>C, vector<vector<double>>D, double dts);
    long SetGainsK(vector<vector<double>> new_gainsK);

    double Update(double plantIn, double plantOut);

private:
    VectorXd hatx;
    VectorXd haty;
    VectorXd erry;
    VectorXd gainsK;


};

#endif // STATEOBSERVER_H
