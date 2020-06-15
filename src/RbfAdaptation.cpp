#include <random>
#include <iostream>
#include <numeric>
#include <typeinfo>
#include <float.h>
#include "RbfAdaptation.hpp"
#include "Eigen/Dense"


void RbfAdaptation::init(InputParams params)
{
  if(params.nbGaussiansPerAxis.size()==2)
  {
    init(params.nbGaussiansPerAxis, params.gridSize, params.gridCenter, params.kernelWidth, params.adaptationRate);
    
  }
  else
  {
    init2(params.nbGaussiansPerAxis, params.gridSize, params.gridCenter, params.kernelWidth, params.adaptationRate, params.velocityRange);
  }
}


void RbfAdaptation::init(std::vector<int> nbGaussiansPerAxis, std::vector<float> gridSize, std::vector<float> gridCenter, float kernelWidth, float adaptationRate)
{

  std::cerr << nbGaussiansPerAxis[0] << " " << nbGaussiansPerAxis[1] << std::endl;
  std::cerr << gridSize[0] << " " << gridSize[1] << std::endl;
  std::cerr << gridCenter[0] << " " << gridCenter[1] << std::endl;
  std::cerr << adaptationRate << std::endl;
  std::cerr << "a" << std::endl;
  _nbGaussians = nbGaussiansPerAxis[0]*nbGaussiansPerAxis[1];
  _nbGaussiansPerAxis = nbGaussiansPerAxis;
  _gridSize = gridSize;
  _gridCenter = gridCenter;
  _kernelWidth = kernelWidth;
  _adaptationRate = adaptationRate;

  _weights.resize(_nbGaussians);
  _weights.setConstant(0.0f);
  _widths.resize(_nbGaussians);
  _widths.setConstant(_kernelWidth);
  _centers.resize(_nbGaussians,3);
  _centers.setConstant(0.0f);
 
  Eigen::ArrayXf tempX = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis[0], -_gridSize[0]/2.0f, _gridSize[0]/2.0f);
  Eigen::ArrayXf tempY = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis[1], -_gridSize[1]/2.0f, _gridSize[1]/2.0f);
  std::cerr << "b" << std::endl;

  
  int id = 0;
  for(int k = 0; k < _nbGaussiansPerAxis[0]; k++)
  {
    for(int m = 0; m < _nbGaussiansPerAxis[1]; m++)
    {

      if(_nbGaussiansPerAxis[0]==1)
      {
        _centers(id,0) = _gridCenter[0];
      }
      else
      {
        _centers(id,0) = tempX(k)+_gridCenter[0];
      }

      if(_nbGaussiansPerAxis[1]==1)
      {
        _centers(id,1) = _gridCenter[1];
      }
      else
      {
        _centers(id,1) = tempY(m)+_gridCenter[1];
      }
      id++;
    }
  }
}

void RbfAdaptation::init2(std::vector<int> nbGaussiansPerAxis, std::vector<float> gridSize, std::vector<float> gridCenter, float kernelWidth, float adaptationRate, float velocityRange)
{

  std::cerr << nbGaussiansPerAxis[0] << " " << nbGaussiansPerAxis[1] << " " << nbGaussiansPerAxis[2] << std::endl;
  std::cerr << gridSize[0] << " " << gridSize[1] << std::endl;
  std::cerr << gridCenter[0] << " " << gridCenter[1] << std::endl;
  std::cerr << adaptationRate << std::endl;
  std::cerr << velocityRange << std::endl;
  std::cerr << "b" << std::endl;
  _nbGaussians = nbGaussiansPerAxis[0]*nbGaussiansPerAxis[1]*nbGaussiansPerAxis[2]*nbGaussiansPerAxis[2];
  _nbGaussiansPerAxis = nbGaussiansPerAxis;
  _gridSize = gridSize;
  _gridCenter = gridCenter;
  _kernelWidth = kernelWidth;
  _adaptationRate = adaptationRate;
  _velocityRange = velocityRange;

  _weights.resize(_nbGaussians);
  _weights.setConstant(0.0f);
  _widths.resize(_nbGaussians);
  _widths.setConstant(_kernelWidth);
  _kernelVelocityWidth = _velocityRange/_nbGaussiansPerAxis[2];

  _centers.resize(_nbGaussians,5);
  _centers.setConstant(0.0f);
 
  Eigen::ArrayXf tempX = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis[0], -_gridSize[0]/2.0f, _gridSize[0]/2.0f);
  Eigen::ArrayXf tempY = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis[1], -_gridSize[1]/2.0f, _gridSize[1]/2.0f);
  Eigen::ArrayXf tempVx = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis[2], -_velocityRange/2.0f, _velocityRange/2.0f);
  Eigen::ArrayXf tempVy = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis[2], -_velocityRange/2.0f, _velocityRange/2.0f);
  std::cerr << "b" << std::endl;

  
  int id = 0;
  for(int k = 0; k < _nbGaussiansPerAxis[0]; k++)
  {
    for(int m = 0; m < _nbGaussiansPerAxis[1]; m++)
    {

      for(int n = 0; n < _nbGaussiansPerAxis[2]; n++)
      {
        for(int o = 0; o < _nbGaussiansPerAxis[2]; o++)
        {
          if(_nbGaussiansPerAxis[0]==1)
          {
            _centers(id,0) = _gridCenter[0];
          }
          else
          {
            _centers(id,0) = tempX(k)+_gridCenter[0];
          }

          if(_nbGaussiansPerAxis[1]==1)
          {
            _centers(id,1) = _gridCenter[1];
          }
          else
          {
            _centers(id,1) = tempY(m)+_gridCenter[1];
          }
          _centers(id,3) = tempVx(n);
          _centers(id,4) = tempVy(o);
          id++; 
        }
      }
    }
  }
}


float RbfAdaptation::update(float error, Eigen::Vector3f x)
{ 
  _weights += -_adaptationRate*error*gradientWRTWeights(x);
  return model(x);
}


float RbfAdaptation::model(Eigen::Vector3f x)
{  
  Eigen::VectorXf values;
  values = gaussianRBF(x);
  float total = values.sum();
  
  if(total<FLT_EPSILON)
  {
    return 0.0f;
  }

  values /= total;

  return (_weights.cwiseProduct(values)).sum();
}


Eigen::VectorXf RbfAdaptation::gaussianRBF(Eigen::Vector3f x)
{  
  Eigen::VectorXf values;
  values.resize(_nbGaussians);
  values.setConstant(0.0f);

  float d;

  for(int k = 0; k < _nbGaussians; k++)
  {
    d = (x-_centers.row(k).transpose()).squaredNorm();
    values(k) = exp(-(d/(2*pow(_widths(k),2))));

  }
  return values;
}


Eigen::VectorXf RbfAdaptation::gradientWRTWeights(Eigen::Vector3f x)
{  
  Eigen::VectorXf values;
  values = gaussianRBF(x);
  float total = values.sum();
  
  if(total<FLT_EPSILON)
  {
    values.setConstant(0.0f);
    return values;
  }

  values /= total;

  return values;
}


float RbfAdaptation::update(float error, Eigen::Vector3f x, Eigen::Vector3f v)
{ 
  _weights += -_adaptationRate*error*gradientWRTWeights(x,v);
  return model(x,v);
}


float RbfAdaptation::model(Eigen::Vector3f x, Eigen::Vector3f v)
{  
  Eigen::VectorXf values;
  values = gaussianRBF(x,v);
  float total = values.sum();
  
  if(total<FLT_EPSILON)
  {
    return 0.0f;
  }

  values /= total;

  return (_weights.cwiseProduct(values)).sum();
}


Eigen::VectorXf RbfAdaptation::gaussianRBF(Eigen::Vector3f x, Eigen::Vector3f v)
{  
  Eigen::VectorXf values;
  values.resize(_nbGaussians);
  values.setConstant(0.0f);

  float d;

  Eigen::VectorXf X;
  X.resize(5);
  X << x, v(0), v(1);
  Eigen::Matrix<float,5,5> S;
  S.resize(5,5);
  S.setConstant(0.0f);
  S(0,0) = 1.0f/pow(_kernelWidth,2.0f);
  S(1,1) = 1.0f/pow(_kernelWidth,2.0f);
  S(2,2) = 1.0f/pow(_kernelWidth,2.0f);
  S(3,3) = 1.0f/pow(_kernelVelocityWidth,2.0f);
  S(4,4) = 1.0f/pow(_kernelVelocityWidth,2.0f);


  for(int k = 0; k < _nbGaussians; k++)
  {
    d = ((X-_centers.row(k).transpose()).transpose())*S*(X-_centers.row(k).transpose());
    values(k) = exp(-d/2.0f);

  }
  return values;
}


Eigen::VectorXf RbfAdaptation::gradientWRTWeights(Eigen::Vector3f x, Eigen::Vector3f v)
{  
  Eigen::VectorXf values;
  values = gaussianRBF(x,v);
  float total = values.sum();
  
  if(total<FLT_EPSILON)
  {
    values.setConstant(0.0f);
    return values;
  }

  values /= total;


  return values;
}

void RbfAdaptation::resetWeights()
{
  _weights.setConstant(0.0f);
}

int RbfAdaptation::getNbGaussians()
{
  return _nbGaussians;
}

float RbfAdaptation::getAdaptationRate()
{
  return _adaptationRate;
}

Eigen::VectorXf RbfAdaptation::getWidths()
{
  return _widths;
}

Eigen::VectorXf RbfAdaptation::getWeights()
{
  return _weights;
}

Eigen::MatrixXf RbfAdaptation::getCenters()
{
    return _centers;
}


void RbfAdaptation::setNbGaussians(int nbGaussiansPerRowColumn)
{
  _nbGaussians = nbGaussiansPerRowColumn*nbGaussiansPerRowColumn;
}


void RbfAdaptation::setAdaptationRate(float adaptationRate)
{
  _adaptationRate = adaptationRate;
}


void RbfAdaptation::setWidths(float kernelWidth)
{
  _kernelWidth = kernelWidth;
  _widths.setConstant(_kernelWidth);
}


