//
// Created by Mason U'Ren on 11/8/17.
//

#ifndef PROJECT_ZONE_H
#define PROJECT_ZONE_H

#define R1 0.718
#define RZ 1.018
#define R2 1.318
#define R3 1.769

typedef struct {
    double x;
    double y;
    double theta;
} POSE;

typedef struct {
    std::string zone_ID;
    bool traverse;
    POSE goal_pose;
} GOAL_ZONE_POSE;

class Zone {
private:
    std::string z_name;
    POSE z_pose;
    bool z_held;

public:
    Zone (std::string name, POSE location) :
            z_name(name), z_pose(location), z_held(false) {}
    Zone() {};

    /*
     * Setters
     */
    void setOccupancy(bool open){
        this->z_held = open;
    }

    /*
     * Getters
     */
    bool getOccupancy(){
        return z_held;
    }
    std::string getName(){
        return z_name;
    }
    POSE getPose(){
        return z_pose;
    }
};


#endif //PROJECT_ZONE_H
