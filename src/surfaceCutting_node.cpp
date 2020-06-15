#include "SurfaceCutting.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_cutting");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;
  float targetVelocity;
  float targetForce;


  std::ostringstream ss;
  std::string temp;

  if(argc == 6) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-v" && atof(argv[3])> 0.0f)
    {
      targetVelocity = atof(argv[3]);
    }
    else
    {
      ROS_ERROR("Wrong target velocity argument, the command line arguments should be: fileName -v(target velocity) value -f(target force) value");
      return 0;
    }  

    if(std::string(argv[4]) == "-f" && atof(argv[5])> 0.0f)
    {
      targetForce = atof(argv[5]);
    }
    else
    {
      ROS_ERROR("Wrong target force argument, the command line arguments should be: fileName -v(target velocity) value -f(target force) value");
      return 0;
    } 
  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName -v(target velocity) value -f(target force) value");
    return 0;
  }

  ss << "_" << targetVelocity << "_" << targetForce;
  fileName += ss.str();
  
  SurfaceCutting surfaceCutting(n,frequency,fileName,targetVelocity,targetForce);

  if(!surfaceCutting.init()) 
  {
    return -1;
  }
  else
  { 
    surfaceCutting.run();
  }

  return 0;
}

