////
//// Created by administrator on 11/7/17.
////
//
//#include "ZoneMap.h"
//
//template <typename T, class S>
//void ZoneMap<T,S>::checkAvailable() {
//    this->availability.clear();
//    /*
//     * Check for available zones
//     * Pass copy of map so ZoneMap can develop iterator
//     */
//    std::map<std_msgs::String, Zone> map_copy = this->getMapCopy();
//    for (typename std::map<std_msgs::String, Zone>::iterator it = map_copy.begin(); it != map_copy.end(); ++it) {
//        if (!this->getValue(it->first).getOccupancy()) {
//            availability.push_back(it->first);
//        }
//    }
//}
//
//template <typename T, class S>
//Agent ZoneMap<T,S>::getClosestZone(Agent robot) {
//    Zone zone;
//    /*
//     * No available zones
//     */
//    if (this->availability.empty()) {
//        geometry_msgs::Pose2D pose;
//        pose.x = 0;
//        pose.y = 0;
//        pose.theta = 0;
//        std_msgs::String null_string;
//        null_string.data = "NULL";
//        robot.setGZPose(null_string, true, pose); // Add target pose to robot object
//    }
//    else {
//        /*
//         * Only one zone open
//         */
//        if (this->availability.size() > 0 && this->availability.size() < 2) {
//            std_msgs::String key = this->availability.front();
//            zone = this->getValue(key);
//            robot.setGZPose(zone.getName(), true, zone.getPose()); // Add target pose to robot object
//        }
//        /*
//         * Find closest available zone
//         */
//        else {
//            double goal_angle = -M_PI;
//            Zone temp_zone;
//            for (std::vector<std_msgs::String>::iterator it = this->availability.begin(); it != this->availability.end(); ++it) {
//                temp_zone = this->getValue(*it);
//                geometry_msgs::Pose2D targ_pose = temp_zone.getPose();
//                /*
//                 * Need to find the shortest angle between rovers heading and goal heading.
//                 * Then most positive value is the closest open zone to the rover in a ccw direction
//                 */
//                double angle = this->angleCalc(targ_pose, robot.getCurrPose());
//                angle = angles::shortest_angular_distance(angle, robot.getCurrPose().theta);
//                /*
//                 * Zone is directly in front of rover
//                 */
//                if ((-ERROR_MARGIN > angle && angle < ERROR_MARGIN) || angle == M_PI) {
//                    zone = temp_zone;
//                    robot.setGZPose(*it, false, targ_pose); // Add target pose to robot object; leave traverse value set to false
//                    break;
//                }
//                else if (goal_angle < angle) {
//                    goal_angle = angle;
//                    zone = temp_zone;
//                    robot.setGZPose(*it, true, targ_pose);
//                }
//            }
//        }
//        zone.setOccupancy(true);
//        this->updateMap(zone.getName(), zone); // Update ZoneMap
//    }
//    return robot;
//}