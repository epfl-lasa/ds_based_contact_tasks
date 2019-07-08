#ifndef RBF_ADAPTATION
#define RBF_ADAPTATION
#include "Eigen/Core"


class RbfAdaptation
{


  private:
    int _nbGaussians;
    Eigen::Vector2i _nbGaussiansPerAxis;
    Eigen::Vector2f _gridSize;
    Eigen::Vector2f _gridCenter;
    float _kernelWidth;
    float _adaptationRate;

    Eigen::VectorXf _weights;
    Eigen::VectorXf _widths;
    Eigen::MatrixXf _centers;
 
  public:
    RbfAdaptation(Eigen::Vector2i nbGaussiansPerAxis, Eigen::Vector2f gridSize, Eigen::Vector2f gridCenter, float kernelWidth, float adaptationRate);

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