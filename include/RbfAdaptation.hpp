#ifndef RBF_ADAPTATION
#define RBF_ADAPTATION
#include "Eigen/Core"


class RbfAdaptation
{
  public:
    struct InputParams
    {
      std::vector<int> nbGaussiansPerAxis;
      std::vector<float> gridSize;
      std::vector<float> gridCenter;
      float kernelWidth;
      float adaptationRate;
      float velocityRange;
    };

  private:
    int _nbGaussians;
    std::vector<int> _nbGaussiansPerAxis;
    std::vector<float> _gridSize;
    std::vector<float> _gridCenter;
    float _kernelWidth;
    float _kernelVelocityWidth;
    float _adaptationRate;

    Eigen::VectorXf _weights;
    Eigen::VectorXf _widths;
    Eigen::MatrixXf _centers;
    float _velocityRange; 
 
  public:
    RbfAdaptation(){};

    void init (InputParams params);
    
    void init(std::vector<int> nbGaussiansPerAxis, std::vector<float> gridSize, std::vector<float> gridCenter, float kernelWidth, float adaptationRate);

    void init2(std::vector<int> nbGaussiansPerAxis, std::vector<float> gridSize, std::vector<float> gridCenter, float kernelWidth, float adaptationRate, float velocityRange);


    float update(float error, Eigen::Vector3f x);

    float update(float error, Eigen::Vector3f x, Eigen::Vector3f v);

    void resetWeights();
    
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

   float model(Eigen::Vector3f x, Eigen::Vector3f v);

    Eigen::VectorXf gaussianRBF(Eigen::Vector3f x, Eigen::Vector3f v);
    
    Eigen::VectorXf gradientWRTWeights(Eigen::Vector3f x, Eigen::Vector3f v);

    int getNbGaussians();

    
};

#endif