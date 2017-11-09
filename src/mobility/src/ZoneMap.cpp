//
// Created by administrator on 11/7/17.
//

#include "ZoneMap.h"



std::vector<std::string> ZoneMap::getAvail(){
    std::vector<std::string> available;
    for (std::map<std::string, Zone>::iterator it = zone_map.begin(); it != zone_map.end(); ++it){
        if (zone_map[it->first].getOccupancy()){
            available.push_back(it->first);
        }
    }
    return available;
}

std::string ZoneMap::getClosest(std::vector<std::string> available){
    for (std::map<std::string, Zone>::iterator it = zone_map.begin(); it != zone_map.end(); ++it){

    }

}