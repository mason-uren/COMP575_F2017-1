//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_INNERRADIUS_H
#define PROJECT_INNERRADIUS_H

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>

#include "Map.h"
#include "Zone.h"
#include "ZoneMap.h"

#define CP 1.1 // Roughly calculated to have the rover traverse in the center of the driveway
#define CPR 1.544 // Traversal Path

typedef struct {
    geometry_msgs::Pose2D left_cp;
    geometry_msgs::Pose2D middle_cp;
    geometry_msgs::Pose2D right_cp;
} CRITICAL_POINTS;

template <typename T, class S>
class InnerRadius : private Map<T,S> {
private:
    std_msgs::String chosen_zone;
    CRITICAL_POINTS cps;
    geometry_msgs::Pose2D pose;
    std_msgs::String name;

    geometry_msgs::Pose2D defaultPose(double x, double y, double theta) {
        geometry_msgs::Pose2D pose;
        pose.x = x;
        pose.y = y;
        pose.theta = theta;
        return pose;
    }

public:
    InnerRadius () {
        new Map<T,S>();
        for (int zones = 0; zones < ZoneMap<std_msgs::String, Zone>::getSize(); zones++) {
            name.data.clear();
            switch (zones) {
                case 0:
                    name.data = "a";
                    this->addToMap(name,
                                   this->setCP(
                                           this->defaultPose(-CP, CP, 0),
                                           this->defaultPose(-RZ, 0, 0),
                                           this->defaultPose(-CP, -CP, 0)
                                   )
                    );
                    break;
                case 1:
                    name.data = "b";
                    this->addToMap(name,
                                   this->setCP(
                                           this->defaultPose(-CP, -CP, 0),
                                           this->defaultPose(0, -RZ, 0),
                                           this->defaultPose(CP, -CP, 0)
                                   )
                    );
                    break;
                case 2:
                    name.data = "c";
                    this->addToMap(name,
                                   this->setCP(
                                           this->defaultPose(CP, -CP, 0),
                                           this->defaultPose(RZ, 0, 0),
                                           this->defaultPose(CP, CP, 0)
                                   )
                    );
                    break;
                case 3:
                    name.data = "d";
                    this->addToMap(name,
                                   this->setCP(
                                           this->defaultPose(CP, CP, 0),
                                           this->defaultPose(0, RZ, 0),
                                           this->defaultPose(-CP, CP, 0)
                                   )
                    );
                    break;
                default:
                std::cout << "ERROR: number of zones does not match number currently allowed (INNERRADIUS)." << std::endl;
                    break;
            }
        }

    }

    /*
     * Mutators
     */
    CRITICAL_POINTS setCP(geometry_msgs::Pose2D cpl, geometry_msgs::Pose2D cpm, geometry_msgs::Pose2D cpr) {
        this->cps.left_cp = cpl;
        this->cps.middle_cp = cpm;
        this->cps.right_cp = cpr;
        return cps;
    }
    CRITICAL_POINTS getCP(T key) {
        return this->getValue(key);
    }
};


#endif //PROJECT_INNERRADIUS_H
