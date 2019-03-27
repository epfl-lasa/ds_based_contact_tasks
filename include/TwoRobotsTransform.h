#ifndef __TWO_ROBOTS_TRANSFORM_H__
#define __TWO_ROBOTS_TRANSFORM_H__

#include <signal.h>
#include <mutex>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <boost/shared_ptr.hpp>
#include "Eigen/Eigen"


#define NB_ROBOTS 2
#define AVERAGE_COUNT 100

using namespace std;

class TwoRobotsTransform
{
  public:
    // Mode, either for simulation or reality
    enum Mode {SIM = 0, REAL = 1};

  private:
    // Robot ID, left or right
    enum ROBOT {LEFT = 0, RIGHT = 1};

    // ROS variables
    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

    // Subscribers declaration
    ros::Subscriber _subOptitrackPose[NB_ROBOTS]; // optitrack markers pose            
    
    // Optitrack variables
    Eigen::Matrix<float,3,NB_ROBOTS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,NB_ROBOTS> _markersPosition0;      // Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,NB_ROBOTS,1> _markersSequenceID;  // Markers sequence ID
    Eigen::Matrix<uint16_t,NB_ROBOTS,1> _markersTracked;     // Markers tracked state

    // Booleans
    bool _firstOptitrackPose[NB_ROBOTS];         // Monitor first optitrack markers update
    bool _optitrackOK;                                  // Check if all markers position is received
    bool _stop;                                         // Check for CTRL+C

    // Tf transform variables
    tf::TransformBroadcaster _br;
    tf::Transform _transform;
    
    // Other variables
    Mode _mode;
    uint32_t _sequenceID;
    uint32_t _averageCount = 0;
    static TwoRobotsTransform* me;
    
  public:
    // Class constructor
    TwoRobotsTransform(ros::NodeHandle &n, double frequency, Mode mode);

    // Class destructor
    ~TwoRobotsTransform();

    // Initialized node
    bool  init();

    // Run node
    void run();

  
  private:
  
    // update transform from optitrack robots' pose
    void updateTf();

    // Callback called when CTRL is detected to stop the node
    static void stopNode(int sig);

    // Compute inital markers positon
    void optitrackInitialization();

    // Callback to update markers pose from Optitrack
    void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    // Check if the marker is tracked
    uint16_t checkTrackedMarker(float a, float b);
};
#endif  // __TWO_ROBOTS_TRANSFORM_H__
