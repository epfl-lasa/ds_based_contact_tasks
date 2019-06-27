#include <random>
#include <iostream>
#include <numeric>
#include "adaptation_rbf.hpp"
#include <typeinfo>
#include <float.h>

using namespace Eigen;



Rbf_parameter::Rbf_parameter (int nbG, int nbE, float sigma, float epsilon, float epsilon_sigma, float uCenter=1.5, float lCenter = -1.5, float xCenter = 0.0, float yCenter = 0.0){

    m_nbG = nbG;
    m_nbE = nbE;
    m_sigma = MatrixXd::Constant(nbG*nbG,1,sigma);
    m_epsilon = epsilon;
    m_epsilon_sigma = epsilon_sigma;


    m_vecError = VectorXd::Zero(nbE);
    m_vecRbf = MatrixXd::Zero(nbE, nbG*nbG);
    m_weights = VectorXd::Zero(nbG*nbG);

    m_vecMoment = VectorXd::Zero(nbG*nbG);
    m_vecAdapSigma = VectorXd::Zero(nbG*nbG);
 
    ArrayXf center = ArrayXf::LinSpaced(nbG, lCenter, uCenter);
   // std::cout << m_centersX;
    
    // m_centersX = MatrixXd::Zero(nbG, nbG);
    // m_centersY = MatrixXd::Zero(nbG, nbG);


    // for (int i = 0; i<nbG; i++){
    //     for (int j= 0; j<nbG;j++){
    //         m_centersX(i,j) = center[i]+xCenter;
    //         m_centersY(i,j) = center[j]+yCenter;
    //     }
    // }


    m_centers = MatrixXd::Zero(nbG*nbG,2);

    int row = 0;
    for (int i = 0; i<nbG; i++){
        for (int j= 0; j<nbG;j++){

            m_centers(row,0) = center[i]+xCenter;
            m_centers(row,1) = center[j]+yCenter;
            row += 1;
        }
    }



    m_weights_error = MatrixXd::Zero(nbE,nbE);

    for (int i = 0;i<nbE;i++){
        m_weights_error(nbE-i-1,nbE-i-1) = pow(0.95, nbE-i-1);
    }

}

// float Rbf_parameter::update (float error,float x, float y){

//     m_vecError << m_vecError.tail(m_nbE-1), error;
//     VectorXd rbf_values = rbf(x,y);

    
//     m_vecRbf  << m_vecRbf.bottomRows(m_nbE-1), rbf_values.transpose();


//     float beta1 = 0.8;
//     float beta2 = 0.8;

//     MatrixXd gamma = MatrixXd::Constant(m_nbG*m_nbG,1,0.1);

//     VectorXd sigma_corr = sigma_adaptation(x,y);
//     m_vecMoment = beta1*m_vecMoment + (1-beta1)*sigma_corr;
//     m_vecAdapSigma = beta2*m_vecAdapSigma + (1-beta2)*sigma_corr.cwiseAbs2();


//     m_weights = m_weights - m_epsilon *m_vecRbf.transpose()*m_weights_error* m_vecError;

//     // ! MatrixXd grad = gradient_rbf(x,y);

//     // std::cerr << m_weights << std::endl;
    

    
//     m_sigma = m_sigma - m_epsilon_sigma* error*((gamma+m_vecAdapSigma.cwiseSqrt()).cwiseInverse()).cwiseProduct(m_vecMoment);
//     std::cerr << m_sigma.transpose() << std::endl;
//     // for(int k = 0; k < m_sigma.size();k++)
//     // {
//     //     if(m_sigma(k)<1e-4f)
//     //     {
//     //         m_sigma(k) = 1e-4f;
//     //     }
//     // }


//     // std::cerr << m_sigma << std::endl;


//     // ! m_centers.col(0) = m_centers.col(0) - m_epsilon*error*grad.col(0);
   
//     // ! m_centers.col(1) = m_centers.col(1) - m_epsilon*error*grad.col(1);



//     return m_weights.transpose()*rbf_values;
// }

float Rbf_parameter::update (float error,float x, float y){

    m_vecError << m_vecError.tail(m_nbE-1), error;
    VectorXd rbf_values = rbf(x,y);
    
    m_vecRbf  << m_vecRbf.bottomRows(m_nbE-1), rbf_values.transpose();


    int optimizer = 0;
    float beta1 = 0.5;
    float beta2 = 0.5;

    MatrixXd gamma = MatrixXd::Constant(m_nbG*m_nbG,1,0.1);

    VectorXd sigma_corr = sigma_adaptation(x,y);

    switch(optimizer){
        case 1:
            m_vecMoment = beta1*m_vecMoment + (1-beta1)*sigma_corr;
            m_vecAdapSigma = beta2*m_vecAdapSigma + (1-beta2)*sigma_corr.cwiseAbs2();
        break;

        default:
        break;
    }
   
    m_weights = m_weights - m_epsilon *m_vecRbf.transpose()*m_weights_error* m_vecError;

    MatrixXd grad = gradient_rbf(x,y);
    

    switch(optimizer){
    case 1 :
        m_sigma = m_sigma - m_epsilon_sigma* error*((gamma+m_vecAdapSigma.cwiseSqrt()).cwiseInverse()).cwiseProduct(m_vecMoment);
        break;
    default:
        m_sigma = m_sigma - m_epsilon_sigma *error* sigma_corr;
        break;
    }
    m_sigma = m_sigma.cwiseAbs();
    // for(int k = 0; k < m_nbG*m_nbG;k++)
    // {
    //     if(m_sigma(k)<1e-6f)
    //     {
    //         m_sigma(k) = 1e-6f;
    //     }
    // }

    // m_centers.col(0) = m_centers.col(0) - m_epsilon/1000*error*grad.col(0);
   
    // m_centers.col(1) = m_centers.col(1) - m_epsilon/1000*error*grad.col(1);

    //    for(int k = 0; k < m_nbG*m_nbG;k++)
    // {
    //     if(m_centers(k,0)<-0.1)
    //     {
    //         m_centers(k,0)=-0.1;
    //     }
    //     else if(m_centers(k,0)>0.1)
    //     {
    //         m_centers(k,0)=0.1;
    //     }
    //     if(m_centers(k,1)<-0.1)
    //     {
    //         m_centers(k,1)=-0.1;
    //     }
    //     else if(m_centers(k,1)>0.1)
    //     {
    //         m_centers(k,1)=0.1;
    //     }
    // } 

    return m_weights.transpose()*rbf_values;
}


// MatrixXd Rbf_parameter::gradient_rbf(float x, float y){

//     VectorXd d = VectorXd::Zero(m_nbG*m_nbG);
//     MatrixXd grad = MatrixXd::Zero(m_nbG*m_nbG,2);

    
//     float den = 0;
//     float der = 0;
//     float num = 0;


//     for (int row = 0; row<m_nbG*m_nbG; row++){
//        d(row)= pow((x-m_centers(row,0)),2)+ pow((y-m_centers(row,1)),2);
//        den+=exp(-d(row)/2*(1.0f/pow(m_sigma(row),2)));
//     }
    
//     if(std::fabs(den)<FLT_EPSILON)
//     {
//         return grad;
//     }

//     for (int row = 0; row<m_nbG*m_nbG; row++){
        
//         num = exp(-d(row)/2*(1.0f/m_sigma(row)));
//         der = (-num*den+pow(num,2))/pow(den,2);
//         grad(row,0) = m_weights(row)*(x-m_centers(row,0))*(1.0f/pow(m_sigma(row),2))*der; 
//         grad(row,1) = m_weights(row)*(y-m_centers(row,1))*(1.0f/pow(m_sigma(row),2))*der;
//     }
//     return grad;
// }

MatrixXd Rbf_parameter::gradient_rbf(float x, float y){

    VectorXd d = VectorXd::Zero(m_nbG*m_nbG);
    MatrixXd grad = MatrixXd::Zero(m_nbG*m_nbG,2);

    float den = 0;
    float der = 0;
    float num = 0;
    float phi = 0;
    float a = 0;
    for (int row = 0; row<m_nbG*m_nbG; row++){
       a = pow((x-m_centers(row,0)),2)+ pow((y-m_centers(row,1)),2);
       a = exp(-a/2*(1.0f/pow(m_sigma(row),2)));

       den += a;
       num += m_weights(row)* a;
    }

    if(std::fabs(den)<FLT_EPSILON)
    {
        return grad;
    }

    for (int row = 0; row<m_nbG*m_nbG; row++){
        
        phi = 1/pow(m_sigma(row),2)*exp(-d(row)/pow(m_sigma(row),2));
        grad(row,0) = std::fabs(x-m_centers(row,0))*(-m_weights(row)*phi*den + num*phi)/pow(den,2); 
        grad(row,1) = std::fabs(y-m_centers(row,1))*(-m_weights(row)*phi*den + phi*num)/pow(den,2); 
        //grad(row,1) = m_weights(row)*(y-m_centers(row,1))*(1.0f/pow(m_sigma(row),2))*der;
        //grad(row,1) = (x(1,:)-centers(row))*(-weight(row)*phi*den +phi*num)/den^2;
    }
    return grad;
}


VectorXd Rbf_parameter::sigma_adaptation(float x, float y){

    float a = 0;
    float den = 0;
    float num = 0;
    // float correction = 0;
    VectorXd correction = VectorXd::Zero(m_nbG*m_nbG);

    for (int row = 0; row<m_nbG*m_nbG; row++){
       a = pow((x-m_centers(row,0)),2)+ pow((y-m_centers(row,1)),2);
       a = exp(-a/2*(1.0f/pow(m_sigma(row),2)));

       den += a;
       num += m_weights(row)* a;
    }
    // std::cerr << "den:" <<den << std::endl;
    if(std::fabs(den)<=FLT_EPSILON)
    {
        std::cerr << "bou" << std::endl;
        return correction;
    }

    for (int row = 0; row<m_nbG*m_nbG; row++){
        a = pow((x-m_centers(row,0)),2)+ pow((y-m_centers(row,1)),2);
        a = a*exp(-a/2*(1.0f/pow(m_sigma(row),2)));
        correction(row) = std::fabs(pow(1/m_sigma(row),3))*(m_weights(row)*a*den - a*num)/pow(den,2);
        // correction += pow(1/m_sigma(row),3)*(m_weights(row)*a*den - a*num)/pow(den,2);
    }
    // float total_correction = correction.array().sum();
    // correction.setConstant(total_correction);
    return correction;
    // VectorXd correction = VectorXd::Zero(m_nbG*m_nbG);
    // float a;
    // float deltaSigma = 0.001;
    // VectorXd sigmaPlus, sigmaMinus;
    // sigmaPlus.resize(m_nbG*m_nbG);
    // sigmaMinus.resize(m_nbG*m_nbG);
    // for (int row = 0; row<m_nbG*m_nbG; row++){
    //     sigmaPlus = m_sigma;
    //     sigmaMinus = m_sigma;
    //     sigmaPlus(row)+=deltaSigma;
    //     sigmaMinus(row)-=deltaSigma;
    //     VectorXd temp = (rbf(x,y,sigmaPlus)-rbf(x,y,sigmaMinus))/(2.0f*deltaSigma);
    //     correction(row) = m_weights.transpose()*temp;
    // }
    // // float total_correction = correction.array().sum();
    // // correction.setConstant(total_correction);
    // return correction;
}


VectorXd Rbf_parameter::rbf(float x, float y){
    VectorXd values = VectorXd::Zero(m_nbG*m_nbG,1);
   
    float total = 0;


    VectorXd d = VectorXd::Zero(m_nbG * m_nbG);

    for (int row = 0; row<m_nbG*m_nbG; row++){

       d(row)= pow((x-m_centers(row,0)),2)+ pow((y-m_centers(row,1)),2);
    }


    for(int row = 0; row<m_nbG*m_nbG; row++){
        values(row) = exp(-(1.0f/pow(m_sigma(row),2))/2*d(row));
        // std::cerr << "values: " << values(row) << std::endl;
        total+= values(row);
    }
    if(std::fabs(total)< FLT_EPSILON)
    {
        return VectorXd::Zero(m_nbG * m_nbG);
    }
    values/=total;
    return values;
}

VectorXd Rbf_parameter::rbf(float x, float y, VectorXd sigmas){
    VectorXd values = VectorXd::Zero(m_nbG*m_nbG,1);
   
    float total = 0;


    VectorXd d = VectorXd::Zero(m_nbG * m_nbG);

    for (int row = 0; row<m_nbG*m_nbG; row++){

       d(row)= pow((x-m_centers(row,0)),2)+ pow((y-m_centers(row,1)),2);
    }


    for(int row = 0; row<m_nbG*m_nbG; row++){
        values(row) = exp(-(1.0f/pow(sigmas(row),2))/2*d(row));
        // std::cerr << "values: " << values(row) << std::endl;
        total+= values(row);
    }
    if(std::fabs(total)< FLT_EPSILON)
    {
        return VectorXd::Zero(m_nbG * m_nbG);
    }
    values/=total;
    return values;
}

// ******** GET FUNCTIONS



int Rbf_parameter::get_nbE(){
    return m_nbE;
}

int Rbf_parameter::get_ngG(){
    return m_nbG;
}

float Rbf_parameter::get_epsilon(){
    return m_epsilon;
}

float Rbf_parameter::get_epsilon_sigma(){
    return m_epsilon_sigma;
}

MatrixXd Rbf_parameter::get_sigma(){
    return m_sigma;
}

MatrixXd Rbf_parameter::get_visualization(){
    return m_weights*m_vecRbf.row(m_nbE-1);
}

MatrixXd Rbf_parameter::get_weights()
{
    return m_weights;
}

MatrixXd Rbf_parameter::get_centers()
{
    return m_centers;
}
// ********  SET FUNCTIONS

void Rbf_parameter::set_nbE(int nbE){
    m_nbE = nbE;
}

void Rbf_parameter::set_nbG(int nbG){
    m_nbG = nbG;
}

void Rbf_parameter::set_epsilon(float epsilon){
    m_epsilon = epsilon;
    //m_vecError.setConstant(0.0f);
}

void Rbf_parameter::set_epsilon_sigma(float epsilon_sigma){
    m_epsilon_sigma = epsilon_sigma;
    //m_vecError.setConstant(0.0f);
}

void Rbf_parameter::set_sigma(float sigma){
    m_sigma = MatrixXd::Constant(m_nbG*m_nbG,1,sigma);
    //m_vecError.setConstant(0.0f);
    // m_vecRbf.setConstant(0.0f);
    // m_weights.setConstant(0.0f);
}


