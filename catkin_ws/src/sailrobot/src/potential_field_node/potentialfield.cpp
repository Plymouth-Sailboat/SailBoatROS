#include "potential_field_node/potentialfield.hpp"
#include "math.h"
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void PotentialField::setup(ros::NodeHandle* n){
    std::string path = ros::package::getPath("sailrobot");
    std::ifstream f(path + "/data/waypoints.txt");
    if(f.good())
        waypoints = Utility::ReadGPSCoordinates(path + "/data/waypoints.txt");
    else{
        std::cerr << "Waypoints Coordinates File not Found" << std::endl;
        exit(0);
    }
    f.close();
    f.open(path + "/data/obstacles.txt");
    if(f.good())
        obstacles = Utility::ReadGPSCoordinates(path + "/data/obstacles.txt");
    else{
        std::cerr << "Obstacles Coordinates File not Found" << std::endl;
        exit(0);
    }
    f.close();
}


vec3 PotentialField::distanceVector(vec3 dest, vec3 pos){
    double d = Utility::GPSDist(pos.getX(), pos.getY(), dest.getX(), dest.getY());
    double bearing = Utility::GPSBearing(pos.getX(), pos.getY(), dest.getX(), dest.getY());
    
    return vec3(d*cos(bearing), d*sin(bearing), 0);
}

geometry_msgs::Twist PotentialField::control(){
    int rowsWaypoints =  sizeof(waypoints) / sizeof(waypoints[0]);
    int rowsObstacles =  sizeof(obstacles) / sizeof(obstacles[0]);
    
    if(rowsWaypoints < 1)
        return geometry_msgs::Twist();
    vec3 current(gpsMsg.latitude, gpsMsg.longitude, 0);
    
    geometry_msgs::Twist cmd;
    vec3 heading;
    
    for(int i = 0; i < rowsWaypoints; ++i){
        heading += distanceVector(current, vec3(waypoints[i][0], waypoints[i][1], 0));
    }
    heading = normalize(heading);
    
    for(int i = 0; i < rowsObstacles; ++i){
        vec3 res = distanceVector(current,vec3(obstacles[i][0], obstacles[i][1], 0));
        
        if(float dist = length2(res) < 100){
            res /= dist*dist;
            heading += res;
        }
    }
    
    cmd.linear.x = (double)heading.x;
    cmd.linear.y = (double)heading.y;
    cmd.linear.z = (double)heading.z;
    return cmd;
}

