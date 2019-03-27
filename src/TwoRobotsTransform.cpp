#include "TwoRobotsTransform.h"


TwoRobotsTransform* TwoRobotsTransform::me = NULL;

TwoRobotsTransform::TwoRobotsTransform(ros::NodeHandle &n, double frequency, Mode mode): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_mode(mode)
{
	me=this;
	_stop = false;
}

TwoRobotsTransform::~TwoRobotsTransform()
{
	me->_n.shutdown();
}

bool TwoRobotsTransform::init()
{
  _averageCount = 0;

  for(int k = 0; k < NB_ROBOTS; k++)
  {
  	if(_mode == SIM)
  	{
    	_firstOptitrackPose[k] = true;
  	}
  	else
  	{
  		_firstOptitrackPose[k] = false;
  	}
  	_markersPosition.setConstant(0.0f);
  	_markersPosition0.setConstant(0.0f);
  }

  if(_mode == SIM)
  {
  	_optitrackOK = true;
    _transform.setOrigin(tf::Vector3(0.078f,0.9f,0.0f));
    _transform.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
  }
  else
  {
  	_optitrackOK = false;
  }

	//Subscriber definitions
	_subOptitrackPose[RIGHT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_right/pose", 1, boost::bind(&TwoRobotsTransform::updateOptitrackPose,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
	_subOptitrackPose[LEFT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_left/pose", 1, boost::bind(&TwoRobotsTransform::updateOptitrackPose,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());


	signal(SIGINT,TwoRobotsTransform::stopNode);
	
	if (_n.ok()) 
	{ 
		ros::spinOnce();
		ROS_INFO("[TwoRobotsTransform]: The node is about to start");
		return true;
	}
	else 
	{
		ROS_ERROR("[TwoRobotsTransform]: The node has a problem.");
		return false;
	}
}


void TwoRobotsTransform::stopNode(int sig)
{
    me->_stop= true;
}

void TwoRobotsTransform::run()
{
  while (!_stop) 
  {
	 	if(_firstOptitrackPose[LEFT] && _firstOptitrackPose[RIGHT]) 
	 	{
			if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {
				updateTf();
			}
	 	}
    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::spinOnce();
  _loopRate.sleep();
  
  ros::shutdown();
}


void TwoRobotsTransform::updateTf()
{
  _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world", "world_2"));
}


void TwoRobotsTransform::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  _markersPosition.col(k)(2) -= 0.03f;
}


uint16_t TwoRobotsTransform::checkTrackedMarker(float a, float b)
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


void TwoRobotsTransform::optitrackInitialization()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    if(_markersTracked(RIGHT) && _markersTracked(LEFT))
    {
      _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
      _averageCount++;
    }
    std::cerr << "[TwoRobotsTransform]: Optitrack Initialization count: " << _averageCount << std::endl;
    if(_averageCount == 1)
    {
      ROS_INFO("[TwoRobotsTransform]: Optitrack Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      ROS_INFO("[TwoRobotsTransform]: Optitrack Initialization done !");

      Eigen::Vector3f temp = _markersPosition0.col(LEFT)-_markersPosition0.col(RIGHT);
      tf::Vector3 origin(temp(0),temp(1),temp(2));

      ROS_INFO("[TwoRobotsTransform]: Transform robot left to right: x: %f y: %f z: %f", temp(0), temp(1), temp(2));
      _transform.setOrigin(origin);
      _transform.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
    }
  }
  else
  {
    _optitrackOK = true;
  }
}