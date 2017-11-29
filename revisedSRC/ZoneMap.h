//
// Created by Mason U'Ren on 11/8/17.
//

#ifndef PROJECT_ZONEMAP_H
#define PROJECT_ZONEMAP_H

#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>


#include "Map.h"
#include "Zone.h"
#include "AgentMap.h"

#define ERROR_MARGIN 0.1 // RADIANS

template <typename T, class S>
class ZoneMap : public Map<T, S>, private Zone {
private:
    std::vector<std::string> availability;
public:
    ZoneMap () : Map<T,S>() {
    // Add zones A -> D
    this->addToMap("a", Zone("a", (POSE) {-RZ, 0, 0}));
    this->addToMap("b", Zone("b", (POSE) {0, RZ, 0}));
    this->addToMap("c", Zone("c", (POSE) {RZ, 0, 0}));
    this->addToMap("d", Zone("d", (POSE) {0, -RZ, 0}));
    }
//    ZoneMap () {
//        new Map<T, S>();
//    }

    /*
     * Setters
     */
    Agent getClosestZone(Agent robot);
    void checkAvailable();
    void freeZone(Zone zone) {
        zone.setOccupancy(false);
        this->updateMap(zone.getName(), zone);
    }

    /*
     * Getters
     */
    double angleCalc(POSE z_pose, POSE r_pose) {
        return std::atan2((z_pose.y - r_pose.y), (z_pose.x - r_pose.x));
    }

};

#include "ZoneMap.cpp"
#endif //PROJECT_ZONEMAP_H
