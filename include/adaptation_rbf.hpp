#ifndef ADAPTATION_RBF
#define ADAPTATION_RBF
#include "Eigen/Core"

using namespace Eigen;

class Rbf_parameter{
    int m_nbG; //number of Gaussian
    int m_nbE; // number of parameters to consider for the error
    // ? float m_sigma;    //gaussian parameter : standard deviation
    float m_epsilon; //learning rate
    float m_epsilon_sigma;

    Eigen::MatrixXd m_sigma; //gaussian parameter : standard deviation
    Eigen::VectorXd m_vecError;
    Eigen::VectorXd m_weights;

    Eigen::MatrixXd m_vecRbf;
    Eigen::MatrixXd m_weights_error;

    Eigen::MatrixXd m_centersX;
    Eigen::MatrixXd m_centersY;
    Eigen::MatrixXd m_centers;

    Eigen::VectorXd m_vecMoment;
    Eigen::VectorXd m_vecAdapSigma;

   
    Eigen::VectorXd rbf(float x, float y);
    Eigen::VectorXd rbf(float x, float y, VectorXd sigmas);

    Eigen::MatrixXd gradient_rbf(float x, float y);
    Eigen::VectorXd sigma_adaptation(float x, float y);



    //! Carefull there, a new parameter epsilon_sigma appears, we need to change the initialization of the method 
    public:
    Rbf_parameter(int nbG, int nbE, float sigma, float epsilon, float epsilon_sigma, float uCenter, float lCenter, float xCenter, float yCenter);
    float update(float error,float x, float y);
    

    int get_nbE();
    int get_ngG();
    float get_epsilon();
    float get_epsilon_sigma();
    Eigen::MatrixXd get_sigma();

    MatrixXd get_weights();

    MatrixXd get_centers();

    // ! there too, the method to get sigma is different ^^'
    Eigen::MatrixXd get_visualization();


    void set_nbE(int nbE);
    void set_nbG(int nbG);
    void set_epsilon(float epsilon);
    void set_epsilon_sigma(float epsilon_sigma);
    void set_sigma(float sigma);

    
} ;

#endif