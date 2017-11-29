//
// Created by Mason U'Ren on 11/13/17.
//

#ifndef PROJECT_MAP_H
#define PROJECT_MAP_H


#include <map>

typedef enum {
    ZONE = 0, AGENT
} MAP_TYPE;

template <typename T, class S>
class Map {
private:
    std::map<T, S> map;
public:
    Map () : map(std::map<T,S>()) {}

    /*
     * Setters
     */
    void addToMap(T key, S obj) {
        try {
            if (this->map.find(key) != this->map.end()) { // If key already exists
                throw key;
            }
            this->map[key] = obj;
        }
        catch (T key){
            std::cout << "ERROR: key '" << key << "' already added to map" << std::endl;
        }
    }
    void removeFromMap(T key) {
        try {
            if (this->map.find(key) == this->map.end()) { // If key doesn't exist
                throw key;
            }
            this->map.erase(key);
        }
        catch (T key) {
            std::cout << "ERROR: key '" << key << "' not found in map." << std::endl;
        }

    }
    void updateMap(T key, S obj) {
        try {
            if (this->map.find(key) == this->map.end()) { // If key doesn't exist
                throw key;
            }
            this->map[key] = obj;
        }
        catch (T key) {
            std::cout << "ERROR: did not find matching key '" << key <<"' to update." << std::endl;
        }
    }
    void updateEntireMap(std::map<T,S> map) {
        this->clearMap(); // clear original map
        for (std::map<T,S>::iterator it = map.begin(); it != map.end(); ++it){
            this->addToMap(it->first, it->second);
        }
    }
    void clearMap() {
        try {
            if (this->map.size() < 1) {
                throw ;
            }
            this->map.clear();
        }
        catch (...) {
            std::cout << "ERROR; map is empty." << std::endl;
        }
    }

    /*
     * Getters
     */
    S getValue(T key) {
        S type;
        try {
            if (this->map.find(key) == this->map.end()) { // If key doesn't exist
                throw key;
            }
            return this->map[key];
        }
        catch (...) {
            std::cout << "ERROR: could not get value, no match key '" << key << "'." << std::endl;
            return type;
        }
    }


    std::map<T,S> getMapCopy() {
        std::map<T,S> map_copy = this->map;
        try {
            if (map_copy.empty()){
                throw map_copy;
            }
            return map_copy;
        }
        catch (std::map<T,S> temp_map) {
            std::cout << "ERROR: bad iterator value; map not built." << std::endl;
            return temp_map;
        }
    }





};


#endif //PROJECT_MAP_H
