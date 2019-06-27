#ifndef RBF_ADAPTATION
#define RBF_ADAPTATION
#include "Eigen/Core"


class RbfAdaptation
{
  private:
    int _nbGaussians;
    float _kernelWidth;
    float _gridWidth;
    float _adaptationRate;

    Eigen::VectorXf _weights;
    Eigen::VectorXf _widths;
    Eigen::MatrixXf _centers;
 
  public:
    RbfAdaptation(int nbGaussiansPerRowColumn, float kernelWidth, float gridWidth, float adaptationRate);

    float update(float error, Eigen::Vector3f x);

    float getAdaptationRate();

    Eigen::VectorXf getWidths();

    Eigen::VectorXf getWeights();

    Eigen::MatrixXf getCenters();

    void setNbGaussians(int nbGaussiansPerRowColumn);

    void setAdaptationRate(float adaptationRate);

    void setWidths(float kernelWidth);

  private:
    float model(Eigen::Vector3f x);

    Eigen::VectorXf gaussianRBF(Eigen::Vector3f x);
    
    Eigen::VectorXf gradientWRTWeights(Eigen::Vector3f x);

    int getNbGaussians();

    
};

#endif