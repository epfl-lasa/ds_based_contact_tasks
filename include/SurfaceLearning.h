#ifndef __SURFACE_LEARNING_H__
#define __SURFACE_LEARNING_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "armadillo"
#include "svm_grad.h"

#define NB_FT_SAMPLES 50      		// Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_OPTITRACK_SAMPLES 100  // Number of optitrack samples used for initial objects' pose estimation
#define NB_TRACKED_OBJECT 4       // Number of objects tracked by the motion capture system (optitrack)
#define DATASET_SIZE 30000        // Size of the dataset used to learn the surface model

class SurfaceLearning 
{
	public:
		// Execution mode
		// COLLECTING_DATA: The user brings the robot end effector with force torque sensor mounted
		//                  in contact with the surface and swap the surface while applying a bit of force
		// LEARNING: Learn the surface model using SVM
		// TESTING: Test the learned model: The z axis of the end effector should align with the normal to the surface
		//                                  The normal distance is printed in the terminal with the normal vector learned                                     		
		enum Mode {COLLECTING_DATA = 0, LEARNING = 1, TESTING = 2};
		// Optitrack makers ID
    enum MarkersID {ROBOT_BASIS = 0, P1 = 1, P2 = 2, P3 = 3};

	private:
		///////////////////
		// ROS variables //
		///////////////////
		ros::NodeHandle _nh;   // Ros node handle
		ros::Rate _loopRate;  // Ros loop rate [Hz]
		float _dt;  					// Loop timestep [s]

		/////////////////////////////
		// Subscribers declaration //
		/////////////////////////////
		ros::Subscriber _subRobotPose;												 // Subscribe to robot's pose
		ros::Subscriber _subRobotTwist;												 // Subscribe to robot's twist
		ros::Subscriber _subForceTorqueSensor;								 // Subscribe to force torque sensor
		ros::Subscriber _subOptitrackPose[NB_TRACKED_OBJECT];	 // Subscribe to optitrack markers' pose

		////////////////////////////
		// Publishers declaration //
		////////////////////////////
		ros::Publisher _pubDesiredTwist;			  // Publish desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench;		  // Publish filtered measured wrench
		ros::Publisher _pubMarker;						  // Publish marker (RVIZ) 
		
		//////////////////////////
		// Messages declaration //
		//////////////////////////
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		visualization_msgs::Marker _msgArrowMarker;
    geometry_msgs::WrenchStamped _msgFilteredWrench;

    //////////////////////////
		// Tool characteristics //
		//////////////////////////
		float _toolMass;														 // Tool mass [kg]
		float _toolOffsetFromEE;										 // Tool offset along z axis of end effector [m]							
		Eigen::Vector3f _toolComPositionFromSensor;  // Offset of the tool [m]	(3x1)
		Eigen::Vector3f _gravity;										 // Gravity vector [m/s^2] (3x1)

		//////////////////////////
		// Tool state variables //
		//////////////////////////
		Eigen::Vector3f _x;												 // Position [m] (3x1)
		Eigen::Vector4f _q;												 // Quaternion (4x1)
		Eigen::Matrix3f _wRb;											 // Orientation matrix (3x1)
		Eigen::Vector3f _v;												 // Velocity [m/s] (3x1)
		Eigen::Vector3f _w;												 // Angular velocity [rad/s] (3x1)
		Eigen::Matrix<float,6,1> _wrench;					 // Wrench [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _wrenchBias;			 // Wrench bias [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _filteredWrench;	 // Filtered wrench [N and Nm] (6x1)
		int _wrenchCount;													 // Counter used to pre-process the force data
    float _normalForce;												 // Normal force to the surface [N]
    float _normalDistance;										 // Normal distance to the surface [m]

    ////////////////////////////
		// Tool control variables //
		////////////////////////////
		Eigen::Vector3f _xd;		  // Desired position [m] (3x1)
		Eigen::Vector4f _qd;		  // Desired quaternion (4x1)
		Eigen::Vector3f _omegad;  // Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _vd;		  // Desired modulated velocity [m/s] (3x1)
		float _targetForce;			  // Target force in contact [N]
		float _targetVelocity;	  // Velocity norm of the nominal DS [m/s]
		float _Fd;							  // Desired force profile

		////////////////////
		// Task variables //
		////////////////////
		Mode _mode;							// Execution mode
    Eigen::Vector3f _n;			// Normal vector to the surface (pointing towards the surface) (3x1)
    Eigen::Matrix3f _wRs;		// Orientation matrix from surface frame to world frame
		uint32_t _datapointID;  // ID of the point logged during the collecting data phase

    ////////////////////
		// SVM parameters //
		////////////////////
		SVMGrad _svm;						 // Instance of the SVMGrad class used to evaluate the learned surface state (distance+normal vector)
		float _C;								 // C value (penalty factor)
		float _sigma;						 // Width of the gaussian kernel [m]
		float _epsilonTube;			 // Epsilon tube width
  	bool _generateDataset;	 // Generate dataset (input = position in the surface frame / output = normal distance)
  	bool _addDataOnSurface;	 // Add data samples collected on the surface to the dataset
  	float _forceThreshold;   // Force threshold used to generate the dataset [N]
  	float _heightThreshold;  // Height threshold used to generate the dataset [m]
  	float _heightOffset; 		 // Height offset used to generate the dataset [m]

  	//////////////
    // Booleans //
    //////////////
		bool _firstRobotPose;												  // Monitor the first robot pose update
		bool _firstRobotTwist;											  // Monitor the first robot twist update
		bool _firstWrenchReceived;									  // Monitor first force/torque data update
    bool _firstOptitrackPose[NB_TRACKED_OBJECT];  // Monitor first optitrack markers update
		bool _optitrackOK;													  // Check if all markers position is received
		bool _wrenchBiasOK;													  // Check if computation of force/torque sensor bias is OK
		bool _stop;																	  // Check for CTRL+C

		/////////////////////////
    // Optitrack variables //
    /////////////////////////
    Eigen::Matrix<float,3,NB_TRACKED_OBJECT> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,NB_TRACKED_OBJECT> _markersPosition0;			 // Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,NB_TRACKED_OBJECT,1> _markersSequenceID;	 // Markers sequence ID
    Eigen::Matrix<uint16_t,NB_TRACKED_OBJECT,1> _markersTracked;		 // Markers tracked state
		Eigen::Vector3f _p1;																						 // First marker position in the robot frame
		Eigen::Vector3f _p2;																						 // Second marker position in the robot frame
		Eigen::Vector3f _p3;																						 // Third marker position in the robot frame
		uint32_t _optitrackCount;                                        // Counter used to pre-process the optitrack data

		/////////////////////
		// Other variables //
		/////////////////////
		float _filteredForceGain;  // Filtering gain for force/torque sensor
		std::string _fileName;      // Filename used to log data
		std::ifstream _inputFile;   // Input stream object to read a text file
		std::ofstream _outputFile;  // Output stream object to right to a text file
		std::mutex _mutex;          // Mutex variable

		static SurfaceLearning* me;  // Pointer on the instance of the class

	public:

		// Class constructor
		SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName, 
			              Mode mode, float C, float sigma, float epsilonTube,
			              bool generateDataset, bool addDataOnSurface);
		
		// Initialize node
		bool init();

		// Run node
		void run();

	private:		
		
		// Callback called when CTRL is detected to stop the node
		static void stopNode(int sig);
		
		// Update the surface frame from optitrack
		void updateSurfaceFrame();

		// Compute command to be sent to the DS-impedance controller
    void computeCommand();

		// Compute desired orientation
		void computeDesiredOrientation();

		// Learn SVR model
		void learnSurfaceModel();

	  // Generate input file needed by SVMGrad library from output file generated by libsvm
		void generateSVMGradModelFile();
    
    // Log raw data
    void collectData();

    // Generate dataset from collected raw data
    void generateDataset();

    // Test the learned surface model
    void testSurfaceModel();

    // Publish data to topics
    void publishData();

    // Compute inital markers positon
    void optitrackInitialization();

    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // Callback to update markers pose from Optitrack
		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

		// Check if the marker is tracked
		uint16_t checkTrackedMarker(float a, float b);
};


#endif
