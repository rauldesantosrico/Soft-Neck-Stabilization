#ifndef STATESPACE_H
#define STATESPACE_H

#include <vector>
#include "LinearSystem.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
//#include <unsupported/Eigen/MatrixFunctions>

/**
 * @brief The StateSpace class:
 * This class encapsulates a system based in state space, defined by his matrix G, H, C y B or its transference function G.
 * .
 */

using namespace Eigen;
using namespace std;

class StateSpace :public LinearSystem
{
public:
    StateSpace();
    StateSpace(MatrixXd G,MatrixXd H,MatrixXd C,MatrixXd D,double dts);
    StateSpace(vector<vector<double> > G, vector<vector<double> > H, vector<vector<double> > C, vector<vector<double> > D, double dts);
    double output(double new_input);
    vector<vector<double>> getObservabilityMatrix();
    vector<vector<double>> getControllabiltyMatrix();
    double GetCurrentOut() const;
    vector<vector<double>> GetState();


    void printSystem();
    void setParam(MatrixXd G,MatrixXd H,MatrixXd C,MatrixXd D,double dts);

protected:

    long SetState(vector<vector<double> > new_state);
    VectorXd x;
    double dts;
    int n_state;
    int n_input;
    int n_output;
    MatrixXd vector2matrix(std::vector<vector<double>>);

private:
    std::vector<vector<double>> matrix2vector(MatrixXd);
    MatrixXd G;
    MatrixXd H;
    MatrixXd C;
    MatrixXd D;
    VectorXd y;
    RowVectorXd u;
    MatrixXd ObservabilityMatrix;
    MatrixXd ControllabilityMatrix;

};

#endif //STATESPACE_H
