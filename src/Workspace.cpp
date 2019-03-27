#include "Workspace.h"


Workspace::Workspace()
{

}


bool Workspace::init() 
{
    std::string dataPath = ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_workspace/";
    std::string pathToMu = dataPath + "LWR_workspace_Model_mu.txt";
    std::string pathToPriors = dataPath + "LWR_workspace_Model_prior.txt";
    std::string pathToSigmas = dataPath + "/LWR_workspace_Model_Sigma.txt";
    std::string pathToThreshold = dataPath + "/LWR_workspace_Model_Threshold.txt";

    _muFile.open(pathToMu);
    _priorsFile.open(pathToPriors);
    _sigmasFile.open(pathToSigmas);
    _thresholdFile.open(pathToThreshold);

    if(!_muFile)
    {
        ROS_ERROR("[Workspace]: Unable to open file mu.txt, the data_workspace directory might be missing");
        return false;
    }
    if(!_priorsFile)
    {
        ROS_ERROR("[Workspace]: Unable to open file prior.txt, the data_workspace directory might be missing");
        return false;
    }
    if(!_sigmasFile)
    {
        ROS_ERROR("[Workspace]: Unable to open file sigma.txt, the data_workspace directory might be missing");
        return false;
    }
    if(!_thresholdFile)
    {
        ROS_ERROR("[Workspace]: Unable to open file threshold.txt, the data_workspace directory might be missing");
        return false;
    }

    // The following two parameters are hardcoded according to the GMM model built for the workspace


    Eigen::Vector3f mu;
    Eigen::Matrix3f sigma;
    float prior, threshold;

    std::vector<Eigen::Vector3f> surfaceData;
    std::string line;
    while(std::getline(_priorsFile,line))  
    {
      std::stringstream ss;
      ss << line;
      ss >> prior;
      _priors.push_back(prior);
    }

    for(int k = 0; k < _priors.size(); k++)
    {
      _muFile >> mu(0) >> mu(1) >> mu(2);
      _mus.push_back(mu);
    }

    for(int k = 0; k < _priors.size(); k++)
    {
      _sigmasFile >> sigma(0,0) >> sigma(0,1) >> sigma(0,2) >>
                     sigma(1,0) >> sigma(1,1) >> sigma(1,2) >>
                     sigma(2,0) >> sigma(2,1) >> sigma(2,2);
      _sigmas.push_back(sigma);
    }

    _thresholdFile >> _threshold;

    _muFile.close();
    _priorsFile.close();
    _sigmasFile.close();
    _thresholdFile.close();

    return true;
}


bool Workspace::isReachable(Eigen::Vector3f x)
{
  float probability = 0.0f;

  for(int k = 0; k < _priors.size(); k++)
  {
    probability += _priors[k]*getPdf(x, _mus[k], _sigmas[k]);
  }

  if(probability>=_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}


float Workspace::getPdf(Eigen::Vector3f x, Eigen::Vector3f mu, Eigen::Matrix3f sigma)
{
    float a = pow(sigma.determinant(), -0.5f) / pow(2.0f*M_PI, 1.5f);
    float b = -0.5f * ((x-mu).transpose()) * sigma.inverse() * (x - mu);
    return a * exp(b);
}
