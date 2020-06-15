//
//  Biquad.cpp
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the Biquad code:
//  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//

#include <math.h>
#include "CascadedBiquad.h"

CascadedBiquad::CascadedBiquad()
{

}

void CascadedBiquad::init(float a10, float a11, float a12, float b10, float b11, float b12, 
                          float a20, float a21, float a22, float b20,float  b21, float b22, float G)
{
    _a10 = a10;
    _a11 = a11;
    _a12 = a12;
    _a20 = a20;
    _a21 = a21;
    _a22 = a22;
    _b10 = b10;
    _b11 = b11;
    _b12 = b12;
    _b20 = b20;
    _b21 = b21;
    _b22 = b22;
    _G = G;

    _a = _b20*_b10;
    _b = (_b20*_b11+_b21*_b10);
    _c = (_b20*_b12+_b21*_b11+_b22*_b10);
    _d = (_b21*_b12+_b22*_b11);
    _e = _b22*_b12;

    _f = _a20*_a10;
    _g = (_a20*_a11+_a21*_a10);
    _h = (_a20*_a12+_a21*_a11+_a22*_a10);
    _i = (_a21*_a12+_a22*_a11);
    _j = _a22*_a12;


}

void CascadedBiquad::init(Eigen::VectorXf coeff)
{
    _coeff.resize(coeff.size());
    _coeff = coeff;
}

void CascadedBiquad::update(std::deque<Eigen::Vector3f> x, std::deque<Eigen::Vector3f> &yprev) 
{
    Eigen::Vector3f y;

    // std::cerr << x[0].transpose() << std::endl;
    // std::cerr << x[1].transpose() << std::endl;
    // std::cerr << x[2].transpose() << std::endl;
    // std::cerr << x[3].transpose() << std::endl;
    // std::cerr << x[4].transpose() << std::endl;
    // std::cerr << yprev[0].transpose() << std::endl;
    // std::cerr << yprev[1].transpose() << std::endl;
    // std::cerr << yprev[2].transpose() << std::endl;
    // std::cerr << yprev[3].transpose() << std::endl;
    // std::cerr << yprev[4].transpose() << std::endl;

    // std::cerr << _a << " " << _b << " " << _c << " " << _d << " " << _e << " " << _f << " " << _g << " " << _h << " " << _i <<  " "  << _j << std::endl;
    y = _G*(_a*x[0]+_b*x[1]+_c*x[2]+_d*x[3]+_e*x[4])-(_g*yprev[1]+_h*yprev[2]+_i*yprev[3]+_j*yprev[4]);
    y /= _f;
    // std::cerr << "y: " << y.transpose() << std::endl;
    yprev.push_front(y);
    yprev.pop_back();
    // std::cerr << yprev.size() << std::endl;

}

Eigen::Vector3f CascadedBiquad::update(std::deque<Eigen::Vector3f> x) 
{
    Eigen::Vector3f y;

    y.setConstant(0.0f);
    for(int k = 0; k < _coeff.rows(); k++)
    {
        y += _coeff(k)*x[k];
    }

    return y;
}
