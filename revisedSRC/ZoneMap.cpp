//
// Created by Mason U'Ren on 11/8/17.
//

#include "ZoneMap.h"

template <typename T, class S>
void ZoneMap<T,S>::checkAvailable() {
    availability.clear();
    // Check for available zones
    // Pass copy of map so ZoneMap can develop iterator
    std::map<std::string, Zone> map_copy = this->getMapCopy();
    for (typename std::map<std::string, Zone>::iterator it = map_copy.begin(); it != map_copy.end(); ++it){
        if (!this->getValue(it->first).getOccupancy()) {
            availability.push_back(it->first);
        }
    }
}

template <typename T, class S>
Agent ZoneMap<T,S>::getClosestZone(Agent robot) {
    Zone zone;
    // No available zones
    if (availability.empty()) {
        POSE pose = (POSE) {0, 0 , 0};
        robot.setGZPose("NULL", true, pose); // Add target pose ot robot object
    }
    else{
        // Only one zone_ID open
        if (availability.size() > 0 && availability.size() < 2) {
            std::string key = availability.front();
            zone = this->getValue(key);
            robot.setGZPose(zone.getName(), true, zone.getPose()); // Add target pose ot robot object
        }
        // Find closest available zone_ID
        else {
            double goal_angle = -M_PI; // Left lower bound
            Zone temp_zone;
            for (std::vector<std::string>::iterator it = availability.begin(); it != availability.end(); ++it) {
                temp_zone = this->getValue(*it);
                POSE targ_pose = this->getValue(*it).getPose();
                double angle = this->angleCalc(targ_pose, robot.getCurrPose());
                /*
                 * TODO:
                 * 'angle' calculated from downward facing direction;
                 * need to find shorted angle between robot's heading and 'angle',
                 * using ros::angles package.
                 * Then find the largest angle
                 */
                // Zone is directly in front of agent
                if ((-ERROR_MARGIN > angle && angle < ERROR_MARGIN) || angle == M_PI) {
                    zone = temp_zone;
                    robot.setGZPose(*it, false, targ_pose); // Add target pose ot robot object
                    break;
                }
                else if (goal_angle < angle) {
                    goal_angle = angle;
                    zone = temp_zone;
                    robot.setGZPose(*it, true, targ_pose);
                }
            }
        }
        zone.setOccupancy(true);
        this->updateMap(zone.getName(), zone);  // Update ZoneMap
    }
    return robot;
}
