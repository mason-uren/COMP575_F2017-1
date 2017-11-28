//
// Created by Mason U'Ren on 11/8/17.
//

#ifndef PROJECT_AGENT_H
#define PROJECT_AGENT_H

#include "Localization.h"
#include "Zone.h"
#include "AgentStates.h"

class Agent : private Localization, private AgentStates {
private:
    // Variables
    int agent_ID;
    AgentStates *state;
    POSE current_pose;
    Localization *localization;
    GOAL_ZONE_POSE goal_zone;
    double avg_global_theta;
    double avg_local_theta;
    std::vector<int> neighbors;
    int driveway_state; // Default no driveway state
    bool hasResource;
    bool reachedCPS;
    bool initTraversal;

public:
    Agent (int ID, POSE pose) :
            agent_ID(ID), current_pose(pose), driveway_state(0),
            reachedCPS(false), hasResource(false), initTraversal(false) {
        this->state = new AgentStates();
        this->goal_zone = (GOAL_ZONE_POSE) {"N/A", true, 0,0,0};
        this->localization = new Localization(this->current_pose, (POSE) {0,0,0});
    }
    Agent () {};
    /*
     * Setters
     */
    void setCurrPose(POSE p) {
        this->current_pose = p;
    }
    void setGZPose(std::string name, bool value, POSE gz) {
        this->goal_zone.zone_ID = name;
        this->goal_zone.traverse = value;
        this->goal_zone.goal_pose = gz;
    }
    void setGlobalTheta(double theta) {
        this->avg_global_theta = theta;
    }
    void setLocalTheta(double theta) {
        this->avg_local_theta = theta;
    }
    void setNeighbors(std::vector<int> neighb) {
        // Clear current neighbors
        this->neighbors.clear();
        // Add in new neighbors vector
        this->neighbors = neighb;
    }
    void setDrivewayState(int state) {
        this->driveway_state = state;
    }
    void setReachedCPS(bool value) {
        this->reachedCPS = value;
    }
    void setResource(bool value) {
        this->hasResource = value;
    }
    void setInitTraversal(bool value) {
        this->initTraversal = value;
    }



    /*
     * Getters
     */
    int getID() {
        return this->agent_ID;
    }
    AgentStates *getState() {
        return this->state;
    }
    POSE getCurrPose() {
        return current_pose;
    }
    GOAL_ZONE_POSE getGZPose() {
        return goal_zone;
    }
    double getGlobalTheta() {
        return avg_global_theta;
    }
    double getLocalTheta() {
        return avg_local_theta;
    }
    std::vector<int> getNeighbors() {
        return neighbors;
    }
    int getDrivewayState() {
        return driveway_state;
    }
    double distFromAnchor() {
        return std::hypot((this->current_pose.x - this->localization->getAnchor().x),
                          (this->current_pose.y - this->localization->getAnchor().y));
    }
    bool getReachedCPS() {
        return this->reachedCPS;
    }
    Localization *getLocalization() {
        return this->localization;
    }
    bool getResource() {
        return this->hasResource;
    }
    bool getInitTraversal() {
        return this->initTraversal;
    }
};


#endif //PROJECT_AGENT_H
