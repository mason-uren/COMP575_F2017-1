//
// Created by administrator on 11/7/17.
//

#ifndef PROJECT_ZONE_H
#define PROJECT_ZONE_H

#include "geometry_msgs/Pose2D.h"

#define R1 1.018

class Zone {
    private:
    //    Variables
        std::string z_name;
        geometry_msgs::Pose2D z_pose;
        bool z_held;
//        const double r1 = 1.018;
    protected:
    //    Variables
        geometry_msgs::Pose2D zA;
        geometry_msgs::Pose2D zB;
        geometry_msgs::Pose2D zC;
        geometry_msgs::Pose2D zD;

    public:
        Zone ();
        Zone(std::string name, geometry_msgs::Pose2D location) :
                z_name(name), z_pose(location), z_held(false) {
            // Zone A
            zA.x = -R1;
            zA.y = 0;
            zA.theta = 0;
            // Zone B
            zB.x = 0;
            zB.y = R1;
            zB.theta = 0;
            // Zone C
            zC.x = R1;
            zC.y = 0;
            zC.theta = 0;
            // Zone D
            zD.x = 0;
            zD.y = -R1;
            zD.theta = 0;
        }
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
};
#endif //PROJECT_ZONE_H
