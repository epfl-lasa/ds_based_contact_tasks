#include "ObjectGrasping.h"
#include "Utils.h"

ObjectGrasping* ObjectGrasping::me = NULL;

ObjectGrasping::ObjectGrasping(ros::NodeHandle &n, double frequency, std::string filename, Mode mode, float targetForce, bool adaptNormalModulation):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _filename(filename),
  _mode(mode),
  _targetForce(targetForce),
  _adaptNormalModulation(adaptNormalModulation),
  _xCFilter(3,3,6,1.0f/frequency),
  _xDFilter(3,3,6,1.0f/frequency),
  _zDirFilter(3,3,6,1.0f/frequency)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.035f;
  _toolOffsetFromEE = 0.13f;
  _toolMass = 0.2f;  // TO CHANGE !!!!
  _objectDim << 0.41f, 0.22f, 0.22f;

  _smax = 4.0f;
  for(int k= 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _wrenchBias[k].setConstant(0.0f);
    _wrench[k].setConstant(0.0f);
    _filteredWrench[k].setConstant(0.0f);
    _normalForce[k] = 0.0f;
    _wrenchCount[k] = 0;
    _wrenchBiasOK[k] = false;
    _d1[k] = 1.0f;
    
    _xd[k].setConstant(0.0f);
    _fxc[k].setConstant(0.0f);
    _fxr[k].setConstant(0.0f);
    _fx[k].setConstant(0.0f);
    _fxn[k].setConstant(0.0f);
    _fxp[k].setConstant(0.0f);
    _fxnp[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _Fd[k] = 0.0f;
    _Fdp[k] = 0.0f;

    _normalForceAverage[k] = 0.0f;

    _deltaF[k] = 0.0f;
    
    _s[k] = _smax;
    
    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _firstDampingMatrix[k] = false;
  }
  _c = 0.0f;

  _objectReachable = false;
  _objectGrasped = false;
  _firstObjectPose = false;
  _stop = false;
  _goHome = false;

  _objectAttractor << -0.4f, 0.5f, 0.6f;
  _xhC << -0.4f, 0.45f, 0.7f;
  _xhD << 0.0f,-1.0f,0.0f;
  _xhD *= 0.7f; 
  _vdC.setConstant(0.0f);
  _vdD.setConstant(0.0f);
  _eD = 0.0f;
  _eoD = 0.0f;
  _eC = 0.0f;
  _eoC = 0.0f;

  _epsilonF = 1.0f;
  _epsilonF0 = 5.0f;
  _gammaF = 0.5f;

  for(int k = 0; k < NB_TRACKED_OBJECTS; k++)
  {
    _firstOptitrackPose[k] = false;
    _markersPosition.setConstant(0.0f);
    _markersPosition0.setConstant(0.0f);
  }
  _optitrackOK = false;
  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);
  _leftRobotOrigin << 0.066f, 0.9f, 0.0f;
  _optitrackCount = 0;

  _xoC << -0.3f, _leftRobotOrigin(1)/2.0f, _objectDim(2)/2.0f;
  _xdC = _xoC;
  Eigen::Vector3f x, y, z;
  x << 1.0f, 0.0f, 0.0f;
  y << 0.0f, 1.0f, 0.0f;
  z << 0.0f, 0.0f, 1.0f;
  _p1 = _xdC-_objectDim(0)/2.0f*x+_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _p2 = _xdC+_objectDim(0)/2.0f*x+_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _p3 = _xdC+_objectDim(0)/2.0f*x-_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _p4 = _xdC-_objectDim(0)/2.0f*x-_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _xoD = (_p3+_p4-_p1-_p2)/2.0f;
  _xdD = _xoD;

  _filteredForceGain = 0.9f;
  _graspingForceThreshold = 4.0f;
  _offset.setConstant(0.0f);

  _msgObjectMarker.header.frame_id = "world";
  _msgObjectMarker.header.stamp = ros::Time();
  _msgObjectMarker.ns = "object_shape";
  _msgObjectMarker.id = 0;
  _msgObjectMarker.type = visualization_msgs::Marker::CUBE;
  _msgObjectMarker.action = visualization_msgs::Marker::ADD;
  _msgObjectMarker.pose.position.x = _xdC(0);
  _msgObjectMarker.pose.position.y = _xdC(1);
  _msgObjectMarker.pose.position.z = _xdC(2);
  _msgObjectMarker.pose.orientation.x = 0.0;
  _msgObjectMarker.pose.orientation.y = 0.0;
  _msgObjectMarker.pose.orientation.z = 0.0;
  _msgObjectMarker.pose.orientation.w = 1.0;
  _msgObjectMarker.scale.x = _objectDim(0);
  _msgObjectMarker.scale.y = _objectDim(1);
  _msgObjectMarker.scale.z = _objectDim(2);
  _msgObjectMarker.color.a = 1.0;

  _msgObjectMarker.color.r = 0.1f;
  _msgObjectMarker.color.g = 0.3f;
  _msgObjectMarker.color.b = 0.9f;
  _msgObjectMarker.color.a = 1.0;
}


bool ObjectGrasping::init() 
{
  // Subscriber definitions
  _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&ObjectGrasping::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/lwr/joint_controllers/twist", 1, boost::bind(&ObjectGrasping::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[RIGHT] = _nh.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&ObjectGrasping::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&ObjectGrasping::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[LEFT] = _nh.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&ObjectGrasping::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _nh.subscribe<geometry_msgs::Twist>("/lwr2/joint_controllers/twist", 1, boost::bind(&ObjectGrasping::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[LEFT] = _nh.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&ObjectGrasping::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&ObjectGrasping::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackPose[ROBOT_BASIS_RIGHT] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_right/pose", 1, boost::bind(&ObjectGrasping::updateOptitrackPose,this,_1,ROBOT_BASIS_RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS_LEFT] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_left/pose", 1, boost::bind(&ObjectGrasping::updateOptitrackPose,this,_1,ROBOT_BASIS_LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p1/pose", 1, boost::bind(&ObjectGrasping::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p2/pose", 1, boost::bind(&ObjectGrasping::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p3/pose", 1, boost::bind(&ObjectGrasping::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P4] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p4/pose", 1, boost::bind(&ObjectGrasping::updateOptitrackPose,this,_1,P4),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench[RIGHT] = _nh.advertise<geometry_msgs::WrenchStamped>("ObjectGrasping/filteredWrenchRight", 1);
  _pubNormalForce[RIGHT] = _nh.advertise<std_msgs::Float32>("ObjectGrasping/normalForceRight", 1);

  _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench[LEFT] = _nh.advertise<geometry_msgs::WrenchStamped>("ObjectGrasping/filteredWrenchLeft", 1);
  _pubNormalForce[LEFT] = _nh.advertise<std_msgs::Float32>("ObjectGrasping/normalForceLeft", 1);

  _pubMarker = _nh.advertise<visualization_msgs::Marker>("ObjectGrasping/cube", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&ObjectGrasping::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ObjectGrasping::stopNode);

  _outputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_grasping/"+_filename+".txt");
  if(!_outputFile.is_open())
  {
    ROS_ERROR("[ObjectGrasping]: Cannot open output data file, the data_grasping directory might be missing");
    return false;
  }

  if(_mode == REACHING_GRASPING)
  {
    ROS_INFO("[ObjectGrasping]: Mode: REACHING_GRASPING");
  }
  else if(_mode == REACHING_GRASPING_MANIPULATING)
  {
    ROS_INFO("[ObjectGrasping]: Mode: REACHING_GRASPING_MANIPULATING");
  }
  else
  {
    ROS_ERROR("[ObjectGrasping]: Mode not recognized");
    return false;
  }

  if(_targetForce>0.0f)
  {
    ROS_INFO("[ObjectGrasping]: Target force: %f", _targetForce);
  }
  else
  {
    ROS_ERROR("[ObjectGrasping]: Target force should be positive");
    return false;
  }

  ROS_INFO("[ObjectGrasping]: Adapt normal modulation: %d", (int)_adaptNormalModulation);

  if(!_nh.getParamCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]))
  {
    ROS_ERROR("[ObjectGrasping]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }

  if(!_nh.getParamCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]))
  {
    ROS_ERROR("[ObjectGrasping]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }

  if(!_workspace.init())
  {
    ROS_ERROR("[ObjectGrasping]: Cannot initialize robots' workspace");
    return false;
  }

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[ObjectGrasping]: The object grabbing node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[ObjectGrasping]: The ros node has a problem.");
    return false;
  }
}


void ObjectGrasping::run()
{
  while (!_stop) 
  {
    if(_firstRobotPose[RIGHT] && _firstRobotPose[LEFT] && _firstRobotTwist[RIGHT] && _firstRobotTwist[LEFT] &&
       _wrenchBiasOK[RIGHT] && _wrenchBiasOK[LEFT] && _firstDampingMatrix[RIGHT] && _firstDampingMatrix[LEFT] &&
       _firstOptitrackPose[ROBOT_BASIS_RIGHT] && _firstOptitrackPose[ROBOT_BASIS_LEFT] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3] && _firstOptitrackPose[P4])
    {
      _mutex.lock();

      // Check for update of passive ds controller eigen value
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_d1[RIGHT]);
      ros::param::getCached("/lwr2/ds_param/damping_eigval0",_d1[LEFT]);

      // Initialize optitrack
      if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {
        // Compute object pose from marker positions
        computeObjectPose();

        // Compute control command
        if(_firstObjectPose)
        {
          // Check if object is reachable
          isObjectReachable();

          // Compute control command
          computeCommand();
        }

        // Publish data to topics
        publishData();          

        // Log data
        logData();
      }

      _mutex.unlock();

    }

    ros::spinOnce();
    _loopRate.sleep();
  }

  // Send zero command
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k] = _q[k];
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  ros::shutdown();
}


void ObjectGrasping::stopNode(int sig)
{
  me->_stop = true;
}


void ObjectGrasping::computeObjectPose()
{
  // Check if all markers on the object are tracked
  // The four markers are positioned on the corner of the upper face:
  // P2 ----- P3
  // |        |
  // |        |
  // P1 ----- P4
  if(_markersTracked.segment(NB_ROBOTS,NB_TRACKED_OBJECTS-NB_ROBOTS).sum() == NB_TRACKED_OBJECTS-NB_ROBOTS)
  {

    if(!_firstObjectPose)
    {
      _firstObjectPose = true;
    }

    // Compute markers position in the right robot frame
    _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p4 = _markersPosition.col(P4)-_markersPosition0.col(ROBOT_BASIS_RIGHT);

    // Compute object center position
    _xoC = (_p1+_p2+_p3+_p4)/4.0f;
    // Compute object dimension vector
    // The dimension obtained from the markers is adjusted to match the real
    // object dimension
    _xoD = (_p3+_p4-_p1-_p2)/2.0f;
    _xoD = 0.20f*_xoD.normalized(); 

    // Filter center position and dimension vector of the object
    SGF::Vec temp(3);
    _xCFilter.AddData(_xoC);
    _xCFilter.GetOutput(0,temp);
    _xoC = temp;
    Eigen::Vector3f xDir = _p4-_p1;
    xDir.normalize();
    Eigen::Vector3f yDir = _p2-_p1;
    yDir.normalize();
    Eigen::Vector3f zDir = xDir.cross(yDir);
    zDir.normalize();
    _zDirFilter.AddData(xDir.cross(yDir));
    _zDirFilter.GetOutput(0,temp);
    zDir = temp;
    zDir.normalize();   
    _xoC -= 1.0f*(_objectDim(2)/2.0f)*zDir;
      
    // Filter object direction
    _xDFilter.AddData(_xoD);
    _xDFilter.GetOutput(0,temp);
    _xoD = 0.20f*temp.normalized();

    // std::cerr <<"real" << _xdD.norm() << " " <<_xdD.transpose() << std::endl;
    // std::cerr << "filter" <<  _xoD.norm() << " " <<_xoD.transpose() << std::endl;

    // Update marker object position and orientation for RVIZ
    _msgObjectMarker.pose.position.x = _xoC(0);
    _msgObjectMarker.pose.position.y = _xoC(1);
    _msgObjectMarker.pose.position.z = _xoC(2);
    Eigen::Matrix3f R;
    R.col(0) = xDir;
    R.col(1) = yDir;
    R.col(2) = zDir;
    Eigen::Vector4f q = Utils<float>::rotationMatrixToQuaternion(R);
    _msgObjectMarker.pose.orientation.x = q(1);
    _msgObjectMarker.pose.orientation.y = q(2);
    _msgObjectMarker.pose.orientation.z = q(3);
    _msgObjectMarker.pose.orientation.w = q(0);
  }

  // Compute robots center + distance vector;
  _xC = (_x[LEFT]+_x[RIGHT])/2.0f;
  _xD = (_x[RIGHT]-_x[LEFT]);

  // Compute errors to object center position and dimension vector 
  _eoD = (_xD-_xoD).dot(_xoD.normalized());
  _eoC = (_xoC-_xC).norm();

    // Compute normal vector to the surface object for each robot
  _n[LEFT] = _xoD.normalized();
  _n[RIGHT] = -_xoD.normalized();
  for(int k = 0; k <NB_ROBOTS; k++)
  {
    _normalForce[k] = fabs((_wRb[k]*_filteredWrench[k].segment(0,3)).dot(_n[k]));
  }


}

void ObjectGrasping::isObjectReachable()
{
  // Evaluate workspace model of both robots to check if object is reachable
  bool l = _workspace.isReachable(_xoC-_leftRobotOrigin);
  bool r = _workspace.isReachable(_xoC);
  if(r && l)
  {
    _objectReachable = true;
  }
  else
  {
    _objectReachable = false;
  }

  std::cerr << "[ObjectGrasping]: Reachable: " << (int) _objectReachable << " left: " << (int) l << " right: " << (int) r << " object: " << _markersTracked.segment(NB_ROBOTS,NB_TRACKED_OBJECTS-NB_ROBOTS).sum() << std::endl;
}


void ObjectGrasping::computeCommand()
{
  // Update contact state
  updateContactState();

  // Compute nominal DS
  computeNominalDS();

  // Compute desired contact force profile
  computeDesiredContactForceProfile();

  // Compute modulation terms
  computeModulationTerms();

  // Compute modulated DS
  computeModulatedDS();

  // Compute desired orientation
  computeDesiredOrientation();
}


void ObjectGrasping::updateContactState()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_normalForceWindow[k].size()<MOVING_FORCE_WINDOW_SIZE)
    {
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
    }
    else
    {
      _normalForceWindow[k].pop_front();
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
      for(int m = 0; m < MOVING_FORCE_WINDOW_SIZE; m++)
      {
        _normalForceAverage[k]+=_normalForceWindow[k][m];
      }
      _normalForceAverage[k] /= MOVING_FORCE_WINDOW_SIZE;
    }
  }

  if(_normalForceAverage[LEFT] > 3.0f && _normalForceAverage[RIGHT] > 3.0f &&  _eoD < 0.05f && _eoC < 0.1f)
  {
    _contactState = CONTACT;
    _c = 1.0f;
  }
  else if(!(_normalForceAverage[LEFT] > 3.0f && _normalForceAverage[RIGHT] > 3.0f) && _eoD < 0.05f && _eoC < 0.1f)
  {
    _contactState = CLOSE_TO_CONTACT;
    _c = 0.0f;
  }
  else
  {
    _contactState = NO_CONTACT;
    _c = 0.0f;
  }

  std::cerr << "[ObjectGrasping]: contact state: " << (int) _contactState << " c: " << _c << std::endl;
}


void ObjectGrasping::computeNominalDS()
{
  _goHome = false;
  // Compute desired center position and distance vector
  // based on reachable and grasping states

  if(_objectReachable)
  {
    if(_contactState==CONTACT) // Object reachable and grasped
    {
      _xdC = _xC;
    }
    else // Object reachable but not grasped
    {
      _xdC = _xoC;
    }

    if(_mode == REACHING_GRASPING_MANIPULATING)
    {
      _xdD << 0.0f,-1.0f,0.0f;
      _xdD *= 0.20f;
    }
    else
    {
      _xdD = _xoD; 
    }
  }
  else
  {
    if(_contactState==CONTACT)  // Object not reachable but grasped (due to workspace model inaccuracies)
    {
      _xdC = _xC;
      _xdD = _xoD; 
    }
    else // Object not reachable and not grasped
    {
      _goHome = true;
      _xdC = _xhC;
      _xdD = _xhD;
    }
  }

  // Compute errors to object center position and dimension vector 
  _eD = (_xD-_xdD).dot(_xdD.normalized());
  _eC = (_xdC-_xC).norm();

  std::cerr <<"[ObjectGrasping]: goHome: " << (int) _goHome << std::endl;
  std::cerr << "[ObjectGrasping]: Reachable: " << (int) _objectReachable << " eD: " << _eD << " eC: " <<  _eC << std::endl;

  if(_eD<0.0f)
  {
    _eD = 0.0f;
  }

  // Compute desired robots' center position and distance vector dynamics
  // from execution mode
  if(_mode == REACHING_GRASPING)
  {
    _vdC = 4.0f*(_xdC-_xC);
  }
  else if(_mode == REACHING_GRASPING_MANIPULATING)
  {
    if(_contactState==CONTACT)
    {
      _vdC = (_objectAttractor+_offset-_xC);
    }
    else
    {
      _vdC = 4.0f*(_xdC-_xC);
    }
  }

  _vdD = 2.0f*(_xdD-_xD);

  // Compute robots' nominal DS 
  _fxc[RIGHT] = _vdD/2.0f;
  _fxc[LEFT] = -_vdD/2.0f;
  _fxr[RIGHT].setConstant(0.0f);
  _fxr[LEFT].setConstant(0.0f);

  for(int k = 0; k < NB_ROBOTS; k++)
  { 

    // Maybe remove that
    if(_fxc[k].dot(_n[k])<0.0f && _contactState==CONTACT)
    {
      _fxc[k].setConstant(0.0f);
    } 
    _fx[k] = _fxc[k]+_fxr[k]; 
  }
}

void ObjectGrasping::computeDesiredContactForceProfile()
{
  if(_goHome)
  {
    _Fd[LEFT] = 0.0f;
    _Fd[RIGHT] = 0.0f;
  }
  else
  {
    if(_contactState==CONTACT)
    {
      _Fd[LEFT] = _targetForce;
      _Fd[RIGHT] = _targetForce;
    }
    else if(_contactState==CLOSE_TO_CONTACT)
    {
      _Fd[LEFT] = 3.0f;
      _Fd[RIGHT] = 3.0f;
    }
    else
    {
      _Fd[LEFT] = 0.0f;
      _Fd[RIGHT] = 0.0f;
    }
  }
}

void ObjectGrasping::computeModulationTerms()
{

  if(!_adaptNormalModulation)
  {
    for(int k = 0; k < NB_ROBOTS; k++)
    { 
      _fxn[k] = (_Fd[k]/_d1[k])*_n[k];
    }
  }
  else
  {
    for(int k = 0; k <NB_ROBOTS; k++)
    {
      float ddeltaF = _epsilonF*(_c*(_Fd[k]-_normalForce[k]))-_epsilonF0*(1-_c)*_deltaF[k];
      _deltaF[k] += _dt*ddeltaF;
      if(_deltaF[k] > _gammaF*_Fd[k])
      {
        _deltaF[k] = _gammaF*_Fd[k];
      }
      else if(_deltaF[k] < -_gammaF*_Fd[k])
      {
        _deltaF[k] = -_gammaF*_Fd[k];
      }

      _fxn[k] = ((_Fd[k]+_c*_deltaF[k])/_d1[k])*_n[k]; 
    }
  }

  std::cerr << "[ObjectGrasping]: " << "deltaF LEFT: " << _deltaF[LEFT] << " deltaF RIGHT: " << _deltaF[RIGHT] << std::endl;
  
}

void ObjectGrasping::updateTankScalars()
{
  float dp = 0.01f;
  float ds = 0.1f*_smax;

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _alpha[k] = Utils<float>::smoothFall(_s[k],_smax-ds,_smax);

    _pr[k] = _d1[k]*_v[k].dot(_fxr[k]);

    // _betar[k] = 1-Utils<float>::smoothRise(_pr[k],-2*dp,-1*dp)*Utils<float>::smoothFall(_s[k],0.0f,ds)
    //           -Utils<float>::smoothFall(_pr[k],1*dp,2*dp)*Utils<float>::smoothRise(_s[k],_smax-ds,_smax);

    // _betarp[k] = 1-Utils<float>::smoothRise(_pr[k],-2*dp,-1*dp)*Utils<float>::smoothFall(_s[k],0.0f,ds);

    // _betat[k] = 1-Utils<float>::smoothRise(_pt[k],-2*dp,-1*dp)*Utils<float>::smoothFall(_s[k],0.0f,ds)
    //           -Utils<float>::smoothFall(_pt[k],1*dp,2*dp)*Utils<float>::smoothRise(_s[k],_smax-ds,_smax);

    // _betatp[k] = 1-Utils<float>::smoothRise(_pt[k],-2*dp,-1*dp)*Utils<float>::smoothFall(_s[k],0.0f,ds);

    _pn[k] = _d1[k]*_v[k].dot(_fxn[k]);

    // _betan[k] = 1-Utils<float>::smoothRise(_pn[k],-1*dp,-0*dp)*Utils<float>::smoothFall(_s[k],0.0f,ds)
    //           -Utils<float>::smoothFall(_pn[k],0*dp,1*dp)*Utils<float>::smoothRise(_s[k],_smax-ds,_smax);

    // _betanp[k] = 1-Utils<float>::smoothRise(_pn[k],-1*dp,-0*dp)*Utils<float>::smoothFall(_s[k],0.0f,ds)               // -Utils<float>::smoothFall(_pf[k],1*dp,2*dp)*Utils<float>::smoothRise(_s[k],_smax-ds,_smax)*Utils<float>::smoothRise(_pf[k],-2*dp,-1*dp);
    //            -Utils<float>::smoothRise(_pn[k],-1*dp,0*dp)*Utils<float>::smoothFall(_pn[k],0*dp,1*dp)*Utils<float>::smoothRise(_s[k],_smax-ds,_smax);

    if(_s[k] < -FLT_EPSILON && _pr[k] > FLT_EPSILON)
    {
      _betar[k] = 0.0f;
    }
    else if(_s[k] > _smax && _pr[k] < -FLT_EPSILON)
    {
      _betar[k] = 0.0f;
    }
    else
    {
      _betar[k] = 1.0f;
    }
    
    if(_s[k] < FLT_EPSILON && _pn[k] > FLT_EPSILON)
    {
      _betan[k] = 0.0f;
    }
    else if(_s[k] > _smax && _pn[k] < -FLT_EPSILON)
    {
      _betan[k] = 0.0f;
    }
    else
    {
      _betan[k] = 1.0f;
    }

    if(_pr[k]<-FLT_EPSILON)
    {
      _betarp[k] = 1.0f;
    }
    else
    {
      _betarp[k] = _betar[k];
    }

    if(_pn[k]<-FLT_EPSILON)
    {
      _betanp[k] = 1.0f;
    }
    else
    {
      _betanp[k] = _betan[k];
    }
  }
}


void ObjectGrasping::computeModulatedDS()
{
  // Update tanks' scalar variables
  updateTankScalars();

  // Compute modualted DS
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // Compute corrected nominal DS and modulation terms
    _fxp[k] = _fxc[k]+_betarp[k]*_fxr[k];
    _fxnp[k] = _betanp[k]*_fxn[k];

    // Update tank dynamics !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // _pd[k] = _v.transpose()*(Eigen::Matrix3f::Identity()-_gain*_n*_n.transpose())*_D*_v;
    _pd[k] = _v[k].transpose()*_D[k]*_v[k];
    float ds = _dt*(_alpha[k]*_pd[k]-_betar[k]*_pr[k]-_betan[k]*_pn[k]);

    if(_s[k]+ds>_smax)
    {
      _s[k] = _smax;
    }
    else if(_s[k]+ds<-FLT_EPSILON)
    {
      _s[k] = 0.0f;
    }
    else
    {
      _s[k]+=ds;
    }

    // Update robot's power flow
    _dW[k] = (_betarp[k]-_betar[k])*_pr[k]+(_betanp[k]-_betan[k])*_pn[k]-(1-_alpha[k])*_pd[k];

    // Compute modulated DS
    _vd[k] = _fxp[k]+_fxnp[k]+_vdC;

    // Bound modulated DS for safety 
    if(_vd[k].norm()>_velocityLimit)
    {
      _vd[k] *= _velocityLimit/_vd[k].norm();
    } 

    std::cerr << "[ObjectGrasping]: Robot " << k << " Tank: " << _s[k]  <<" dW: " << _dW[k] <<std::endl;

    if(!_adaptNormalModulation)
    {
      std::cerr << "[ObjectGrasping]: Robot " << k << " F: " << _normalForce[k] << " Fd corrected:  " << _betanp[k]*_Fd[k] << std::endl;
    }
    else
    {
      std::cerr << "[ObjectGrasping]: Robot " << k << " F: " << _normalForce[k] << " Fd corrected:  " << _betanp[k]*(_Fd[k]+_c*_deltaF[k]) << std::endl;
    }

    std::cerr << "[ObjectGrasping]: Robot " << k << " vd: " << _vd[k].norm() << " v: " << _v[k].norm() <<std::endl;
  }

}


void ObjectGrasping::computeDesiredOrientation()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Vector3f ref;
    if(k == (int) RIGHT)
    {
      ref = -_xdD.normalized();
    }
    else
    {
      ref = _xdD.normalized();
    }
      
    ref.normalize();

    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f u;
    u = (_wRb[k].col(2)).cross(ref);
    float c = (_wRb[k].col(2)).transpose()*ref;  
    float s = u.norm();
    u /= s;
    
    Eigen::Matrix3f K;
    K << Utils<float>::getSkewSymmetricMatrix(u);

    Eigen::Matrix3f Re;
    if(fabs(s)< FLT_EPSILON)
    {
      Re = Eigen::Matrix3f::Identity();
    }
    else
    {
      Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    }
    
    // Convert rotation error into axis angle representation
    Eigen::Vector3f omega;
    float angle;
    Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
    Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

    // Compute final quaternion on plane
    Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q[k]);

    // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the object surface
    _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,1.0f-std::tanh(3.0f*_eD));

    if(_qd[k].dot(_qdPrev[k])<0.0f)
    {
      _qd[k] *=-1.0f;
    }

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 
    _qdPrev[k] = _qd[k];
  }
}


void ObjectGrasping::logData()
{
  _outputFile << ros::Time::now() << " "
              << _x[LEFT].transpose() << " "
              << _v[LEFT].transpose() << " "
              << _fxp[LEFT].transpose() << " "
              << _fxnp[LEFT].transpose() << " "
              << _vd[LEFT].transpose() << " "
              << _normalForce[LEFT] << " "
              << _Fd[LEFT] << " "
              << _deltaF[LEFT] << " "
              << _x[RIGHT].transpose() << " "
              << _v[RIGHT].transpose() << " "
              << _fxp[RIGHT].transpose() << " "
              << _fxnp[RIGHT].transpose() << " "
              << _vd[RIGHT].transpose() << " "
              << _normalForce[RIGHT] << " "
              << _Fd[RIGHT] << " "
              << _deltaF[RIGHT] << " "
              << _n[LEFT].transpose() << " "
              << _vdC.transpose() << " "
              << _xoC.transpose() << " "
              << _xoD.transpose() << " "
              << _xdC.transpose() << " "
              << _xdD.transpose() << " "
              << (int) _objectGrasped << " "
              << _c << " "
              << _s[LEFT] << " " 
              << _pd[LEFT] << " " 
              << _pr[LEFT] << " " 
              << _pn[LEFT] << " " 
              << _alpha[LEFT] << " "
              << _betar[LEFT] << " "
              << _betarp[LEFT] << " "
              << _betan[LEFT] << " "
              << _betanp[LEFT] << " "
              << _dW[LEFT] << " "
              << _s[RIGHT] << " "
              << _pd[RIGHT] << " " 
              << _pr[RIGHT] << " " 
              << _pn[RIGHT] << " " 
              << _alpha[RIGHT] << " "
              << _betar[RIGHT] << " "
              << _betarp[RIGHT] << " "
              << _betan[RIGHT] << " "
              << _betanp[RIGHT] << " "
              << _dW[RIGHT] << std::endl;
}


void ObjectGrasping::publishData()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // Publish desired twist (passive ds controller)
    _msgDesiredTwist.linear.x  = _vd[k](0);
    _msgDesiredTwist.linear.y  = _vd[k](1);
    _msgDesiredTwist.linear.z  = _vd[k](2);

    // Convert desired end effector frame angular velocity to world frame
    _msgDesiredTwist.angular.x = _omegad[k](0);
    _msgDesiredTwist.angular.y = _omegad[k](1);
    _msgDesiredTwist.angular.z = _omegad[k](2);

    _pubDesiredTwist[k].publish(_msgDesiredTwist);

    // Publish desired orientation
    _msgDesiredOrientation.w = _qd[k](0);
    _msgDesiredOrientation.x = _qd[k](1);
    _msgDesiredOrientation.y = _qd[k](2);
    _msgDesiredOrientation.z = _qd[k](3);

    _pubDesiredOrientation[k].publish(_msgDesiredOrientation);

    _msgFilteredWrench.header.frame_id = "world";
    _msgFilteredWrench.header.stamp = ros::Time::now();
    _msgFilteredWrench.wrench.force.x = _filteredWrench[k](0);
    _msgFilteredWrench.wrench.force.y = _filteredWrench[k](1);
    _msgFilteredWrench.wrench.force.z = _filteredWrench[k](2);
    _msgFilteredWrench.wrench.torque.x = _filteredWrench[k](3);
    _msgFilteredWrench.wrench.torque.y = _filteredWrench[k](4);
    _msgFilteredWrench.wrench.torque.z = _filteredWrench[k](5);
    _pubFilteredWrench[k].publish(_msgFilteredWrench);

    std_msgs::Float32 msg;
    msg.data = _normalForce[k];
    _pubNormalForce[k].publish(msg); 
  }

  _msgObjectMarker.header.frame_id = "world";
  _msgObjectMarker.header.stamp = ros::Time();
  _pubMarker.publish(_msgObjectMarker);
}


void ObjectGrasping::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffsetFromEE*_wRb[k].col(2);

  if(k==(int)LEFT)
  {
    _x[k] += _leftRobotOrigin;
  }

  if(!_firstRobotPose[k])
  {
    _firstRobotPose[k] = true;
    _xd[k] = _x[k];
    _qd[k] = _q[k];
    _qdPrev[k] = _q[k];
    _vd[k].setConstant(0.0f);
    _qinit[k] = _q[k];
  }
}


void ObjectGrasping::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void ObjectGrasping::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrenchBias[k].segment(0,3) -= loadForce;
    _wrenchBias[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias[k] += raw; 
    _wrenchCount[k]++;
    if(_wrenchCount[k]==NB_FT_SENSOR_SAMPLES)
    {
      _wrenchBias[k] /= NB_FT_SENSOR_SAMPLES;
      _wrenchBiasOK[k] = true;
      std::cerr << "[ObjectGrasping]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    _wrench[k] = raw-_wrenchBias[k];
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrench[k].segment(0,3) -= loadForce;
    _wrench[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  }
}


void ObjectGrasping::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void ObjectGrasping::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(k == (int) ROBOT_BASIS_RIGHT || k == (int) ROBOT_BASIS_LEFT)
  {
    _markersPosition.col(k)(2) -= 0.03f;
  }
}


uint16_t ObjectGrasping::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


void ObjectGrasping::optitrackInitialization()
{
  if(_optitrackCount< NB_OPTITRACK_SAMPLES)
  {
    if(_markersTracked(ROBOT_BASIS_RIGHT) && _markersTracked(ROBOT_BASIS_LEFT))
    {
      _markersPosition0 = (_optitrackCount*_markersPosition0+_markersPosition)/(_optitrackCount+1);
      _optitrackCount++;
    }
    std::cerr << "[ObjectGrasping]: Optitrack Initialization count: " << _optitrackCount << std::endl;
    if(_optitrackCount == 1)
    {
      ROS_INFO("[ObjectGrasping]: Optitrack Initialization starting ...");
    }
    else if(_optitrackCount == NB_OPTITRACK_SAMPLES)
    {
      ROS_INFO("[ObjectGrasping]: Optitrack Initialization done !");
      _leftRobotOrigin = _markersPosition0.col(ROBOT_BASIS_LEFT)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    }
  }
  else
  {
    _optitrackOK = true;
  }
}


void ObjectGrasping::dynamicReconfigureCallback(ds_based_contact_tasks::objectGrasping_paramsConfig &config, uint32_t level)
{
  ROS_INFO("[ObjectGrasping]: Reconfigure request. Updatig the parameters ...");
  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _graspingForceThreshold = config.graspingForceThreshold;
  _offset(0) = config.xOffset;
  _offset(1) = config.yOffset;
  _offset(2) = config.zOffset;
}