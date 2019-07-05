#include <random>
#include <iostream>
#include <numeric>
#include <typeinfo>
#include <float.h>
#include "RbfAdaptation.hpp"


RbfAdaptation::RbfAdaptation(int nbGaussiansPerRowColumn, float kernelWidth, float gridWidth, float gridXOffset, float gridYOffset, float adaptationRate):
_nbGaussians(nbGaussiansPerRowColumn*nbGaussiansPerRowColumn),
_kernelWidth(kernelWidth),
_gridWidth(gridWidth),
_adaptationRate(adaptationRate)
{
  _weights.resize(_nbGaussians);
  _weights.setConstant(0.0f);
  _widths.resize(_nbGaussians);
  _widths.setConstant(_kernelWidth);
  _centers.resize(_nbGaussians,3);
  _centers.setConstant(0.0f);
 
  Eigen::ArrayXf temp = Eigen::ArrayXf::LinSpaced(nbGaussiansPerRowColumn, -_gridWidth/2.0f, _gridWidth/2.0f);
  
  int id = 0;
  for(int k = 0; k < nbGaussiansPerRowColumn; k++)
  {
    for(int m = 0; m < nbGaussiansPerRowColumn; m++)
    {
      _centers(id,0) = temp(k)+gridXOffset;
      _centers(id,1) = temp(m)+gridYOffset;
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
  values /= values.sum();

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


