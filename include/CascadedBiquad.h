//
//  CascadedBiquad.h
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the CascadedBiquad code:
//  http://www.earlevel.com/main/2012/11/25/biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//

#ifndef CASCADED_BIQUAD
#define CASCADED_BIQUAD

#include <deque>
#include <iostream>
#include "Eigen/Eigen"


class CascadedBiquad {
public:

    CascadedBiquad();
     
    void init(float a10, float a11, float a12, float b10, float b11, float b12, 
              float a20, float a21, float a22, float b20,float  b21, float b22, float g);

    void update(std::deque<Eigen::Vector3f> x, std::deque<Eigen::Vector3f> &yprev); 

	void init(Eigen::VectorXf coeff);

    Eigen::Vector3f update(std::deque<Eigen::Vector3f> x); 

private:

    float _a10, _a11, _a12, _a20, _a21, _a22;
    float _b10, _b11, _b12, _b20, _b21, _b22;
    float _G;

    float _a, _b, _c, _d, _e, _f, _g, _h, _i, _j;

    Eigen::VectorXf _coeff;
};



#endif // CASCADED_BIQUAD
