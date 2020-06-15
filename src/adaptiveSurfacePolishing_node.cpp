#include "AdaptiveSurfacePolishing.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_polishing");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;
  AdaptiveSurfacePolishing::SurfaceType surfaceType;  
  float targetVelocity;
  float targetForce;

  bool adaptNormalModulation;

  std::ostringstream ss;
  std::string temp;

  // rosrun force_based_ds_modulation surface_polishing fileName -s p/n  
  if(argc == 4) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "p")
    {
      surfaceType = AdaptiveSurfacePolishing::SurfaceType::PLANAR;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "n")
    {
      surfaceType = AdaptiveSurfacePolishing::SurfaceType::NON_FLAT;
    }
    else
    {
      ROS_ERROR("Wrong surface type argument, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    }
    targetVelocity = 0.2f;
    targetForce = 10.0f;
    adaptNormalModulation = false;
  }
  else if(argc == 10) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "p")
    {
      surfaceType = AdaptiveSurfacePolishing::SurfaceType::PLANAR;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "n")
    {
      surfaceType = AdaptiveSurfacePolishing::SurfaceType::NON_FLAT;
    }
    else
    {
      ROS_ERROR("Wrong surface type argument, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    }

    if(std::string(argv[4]) == "-v" && atof(argv[5])> 0.0f)
    {
      targetVelocity = atof(argv[5]);
    }
    else
    {
      ROS_ERROR("Wrong target velocity argument, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    }  

    if(std::string(argv[6]) == "-f" && atof(argv[7])> 0.0f)
    {
      targetForce = atof(argv[7]);
    }
    else
    {
      ROS_ERROR("Wrong target force argument, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    } 

    if(std::string(argv[8]) == "-an" && std::string(argv[9]) == "y")
    {
      adaptNormalModulation = true;
    }
    else if(std::string(argv[8]) == "-an" && std::string(argv[9]) == "n")
    {
      adaptNormalModulation = false;
    }
    else
    {
      ROS_ERROR("Wrong adapt tangential modulation argument, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    } 
  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
    return 0;
  }

  ss << "_" << targetVelocity << "_" << targetForce;
  fileName += "_"+std::string(argv[3])+ss.str()+"_"+std::string(argv[9]);
  
  AdaptiveSurfacePolishing adaptiveSurfacePolishing(n,frequency,fileName,surfaceType,targetVelocity,targetForce,adaptNormalModulation);

  if(!adaptiveSurfacePolishing.init()) 
  {
    return -1;
  }
  else
  { 
    adaptiveSurfacePolishing.run();
  }

  return 0;
}

