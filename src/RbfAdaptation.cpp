#include <random>
#include <iostream>
#include <numeric>
#include <typeinfo>
#include <float.h>
#include "RbfAdaptation.hpp"


RbfAdaptation::RbfAdaptation(Eigen::Vector2i nbGaussiansPerAxis, Eigen::Vector2f gridSize, Eigen::Vector2f gridCenter, float kernelWidth, float adaptationRate):
_nbGaussians(nbGaussiansPerAxis(0)*nbGaussiansPerAxis(1)),
_nbGaussiansPerAxis(nbGaussiansPerAxis),
_gridSize(gridSize),
_gridCenter(gridCenter),
_kernelWidth(kernelWidth),
_adaptationRate(adaptationRate)
{
  _weights.resize(_nbGaussians);
  _weights.setConstant(0.0f);
  _widths.resize(_nbGaussians);
  _widths.setConstant(_kernelWidth);
  _centers.resize(_nbGaussians,3);
  _centers.setConstant(0.0f);
 
  Eigen::ArrayXf tempX = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis(0), -_gridSize(0)/2.0f, _gridSize(0)/2.0f);
  Eigen::ArrayXf tempY = Eigen::ArrayXf::LinSpaced(_nbGaussiansPerAxis(1), -_gridSize(1)/2.0f, _gridSize(1)/2.0f);
  
  int id = 0;
  for(int k = 0; k < _nbGaussiansPerAxis(0); k++)
  {
    for(int m = 0; m < _nbGaussiansPerAxis(1); m++)
    {

      if(_nbGaussiansPerAxis(0)==1)
      {
        _centers(id,0) = _gridCenter(0);
      }
      else
      {
        _centers(id,0) = tempX(k)+_gridCenter(0);
      }

      if(_nbGaussiansPerAxis(1)==1)
      {
        _centers(id,1) = _gridCenter(1);
      }
      else
      {
        _centers(id,1) = tempY(k)+_gridCenter(1);
      }
      id++;
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
  values /= values.sum();

  return values;
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


