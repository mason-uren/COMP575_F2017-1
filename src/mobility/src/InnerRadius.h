//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_INNERRADIUS_H
#define PROJECT_INNERRADIUS_H

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>

#include "Map.h"
#include "Zone.h"
#include "CriticalPoints.h"

#define CP 1.1 // Roughly calculated to have the rover traverse in the center of the driveway
#define CPR 1.544 // Traversal Path

template <typename T, class S>
class InnerRadius : private Map<T,S>, private CriticalPoints {
private:
    std::string chosen_zone;
    geometry_msgs::Pose2D poseL;
    geometry_msgs::Pose2D poseM;
    geometry_msgs::Pose2D poseR;
    std::string name;

    geometry_msgs::Pose2D defaultPose(double x, double y, double theta) {
        geometry_msgs::Pose2D pose;
        pose.x = x;
        pose.y = y;
        pose.theta = theta;
        return pose;
    }

public:
    InnerRadius () {
//        ZoneMap<std_msgs::String, Zone>::getSize()
        new Map<T,S>();
        for (int zones = 0; zones < 4; zones++) { // TODO: don't like that '4' is static and not a variable
//            this->name.data.clear();
            this->name.clear();
            switch (zones) {
                case 0:
//                    this->name.data = "a";
                    this->name = "a";
                    this->poseL = this->defaultPose(-CP, CP, 0);
                    this->poseM = this->defaultPose(-RZ, 0, 0);
                    this->poseR = this->defaultPose(-CP, -CP, 0);
                    break;
                case 1:
//                    name.data = "b";
                    this->name = "b";
                    this->poseL = this->defaultPose(-CP, -CP, 0);
                    this->poseM = this->defaultPose(0, -RZ, 0);
                    this->poseR = this->defaultPose(CP, -CP, 0);
                    break;
                case 2:
//                    name.data = "c";
                    this->name = "c";
                    this->poseL = this->defaultPose(CP, -CP, 0);
                    this->poseM = this->defaultPose(RZ, 0, 0);
                    this->poseR = this->defaultPose(CP, CP, 0);
                    break;
                case 3:
//                    name.data = "d";
                    this->name = "d";
                    this->poseL = this->defaultPose(CP, CP, 0);
                    this->poseM = this->defaultPose(0, RZ, 0);
                    this->poseR = this->defaultPose(-CP, CP, 0);
                    break;
                default:
                std::cout << "ERROR: number of zones does not match number currently allowed (INNERRADIUS)." << std::endl;
                    break;
            }
            this->addToMap(this->name, CriticalPoints::CriticalPoints(this->poseL, this->poseM, this->poseR));
        }
    }

    /*
     * Getters
     */
    CriticalPoints getCP(T key) {
        return this->getValue(key);
    }
};


#endif //PROJECT_INNERRADIUS_H
