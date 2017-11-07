//
// Created by administrator on 11/7/17.
//

#ifndef PROJECT_ZONE_H
#define PROJECT_ZONE_H

#include "geometry_msgs/Pose2D.h"

class Zone {
    private:
    //    Variables
        const std::string z_name;
        const geometry_msgs::Pose2D z_pose;
    //    Functions
        void setOccupancy(bool open){
            z_held = open;
        }
        bool getOccupancy(){
            return z_held;
        }
        std::string getName(){
            return z_name;
        }
        geometry_msgs::Pose2D getPose(){
            return z_pose;
        }


    public:
        Zone(const std::string name, const geometry_msgs::Pose2D location) :
                z_name(name), z_pose(location), z_held(false) {}
        bool z_held;
};
#endif //PROJECT_ZONE_H
