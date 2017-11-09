//
// Created by administrator on 11/7/17.
//

#ifndef PROJECT_ZONEMAP_H
#define PROJECT_ZONEMAP_H

#include "Zone.h"

class ZoneMap : public Zone {

    private:
    //    Variables
        std::map<std::string, Zone> zone_map;
    public:
    //    Constructor
        ZoneMap () {
            zone_map["a"] = Zone("a", zA);
            zone_map["b"] = Zone("b", zB);
            zone_map["c"] = Zone("c", zC);
            zone_map["d"] = Zone("d", zD);
        }
    //    Functions
        std::vector<std::string> getAvail();
        std::string getClosest(std::vector<std::string>);

};


#endif //PROJECT_ZONEMAP_H
