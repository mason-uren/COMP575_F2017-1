//
// Created by administrator on 11/7/17.
//

#ifndef PROJECT_ZONE_H
#define PROJECT_ZONE_H

#include <std_msgs/String.h>
#include "geometry_msgs/Pose2D.h"

#define R1 0.718
#define RZ 1.018
#define R2 1.318
#define R3 1.769

typedef struct {
    std::string zone_ID;
    bool traverse;
    geometry_msgs::Pose2D goal_pose;
} GOAL_ZONE_POSE;

class Zone {
private:
    std::string z_name;
    geometry_msgs::Pose2D z_pose;
    bool z_held;
public:
    Zone (std::string name, geometry_msgs::Pose2D location) :
        z_name(name), z_pose(location), z_held(false) {}
    Zone () : z_held(false) {};

    /*
     * Setters
     */
    void setOccupancy(bool open) {
        this->z_held = open;
    }

    /*
     * Getters
     */
    bool getOccupancy() {
        return this->z_held;
    }
    std::string getName() {
        return this->z_name;
    }
    geometry_msgs::Pose2D getPose() {
        return this->z_pose;
    }
};
#endif //PROJECT_ZONE_H
