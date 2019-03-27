#ifndef __WORKSPACE_H__
#define __WORKSPACE_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include "Eigen/Eigen"


class Workspace 
{
	private:

		// GMM parameters model of the robot workspace
    std::vector<Eigen::Vector3f> _mus;			// Mu parameters
    std::vector<Eigen::Matrix3f> _sigmas;		// Sigma parameters
    std::vector<float> _priors;							// Priors on the gaussians
    float _threshold;												// reaching probability threshold		

    // Other variables
    std::ifstream _muFile;
    std::ifstream _priorsFile;
    std::ifstream _sigmasFile;
    std::ifstream _thresholdFile;

	public:

		// Class constructor
		Workspace();

		// Init
		bool init();

		// Check if tool position is reachable
		bool isReachable(Eigen::Vector3f);

	private:
	
		// Compute probablity of the queried position to be inside the robot workspace
		float getPdf(Eigen::Vector3f x, Eigen::Vector3f mu, Eigen::Matrix3f sigma);
};
#endif
