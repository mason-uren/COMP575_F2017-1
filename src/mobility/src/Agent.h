//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_AGENT_H
#define PROJECT_AGENT_H


#include <cmath>
#include "Localization.h"
#include "Zone.h"
#include "AgentStates.h"

typedef enum {
    ACHILLES = 0, AENEAS, AJAX, DIOMEDES, HECTOR, PARIS
} AGENT_ID;

class Agent : private Localization, public AgentStates {
private:
    int agent_ID;
    geometry_msgs::Pose2D current_pose;
    Localization *localization;
    GOAL_ZONE_POSE goal_zone;
    double avg_global_theta;
    double avg_local_theta;
    double avg_local_pose;
    std::vector<int> neighbors;
    int driveway_state; // Default no driveway state
    bool hasResource;
    bool reachedCPS;
    bool initTraversal;

public:
    Agent (int ID, geometry_msgs::Pose2D pose) : AgentStates(STATE_INIT), agent_ID(ID), current_pose(pose),
                                                 driveway_state(0), reachedCPS(false), hasResource(false),
                                                 initTraversal(false), avg_global_theta(0), avg_local_theta(0) {
        geometry_msgs::Pose2D temp_pose;
        std::string init_string;
        temp_pose.x = 0;
        temp_pose.y = 0;
        temp_pose.theta = 0;
//        init_string.data = "N/A";
        this->goal_zone = (GOAL_ZONE_POSE) {"N/A", true, temp_pose};
        this->localization = new Localization(this->current_pose, temp_pose);
    }
    Agent () {};
//    ~Agent() {}

    /*
     * Setters
     */
    void setCurrPose(geometry_msgs::Pose2D p) {
        this->current_pose = p;
    }
    void setGZPose(std::string name, bool traversal, geometry_msgs::Pose2D gz) {
        this->goal_zone.zone_ID = name;
        this->goal_zone.traverse = traversal;
        this->goal_zone.goal_pose = gz;
    }
    void setGlobalTheta(double theta) {
        this->avg_global_theta = theta;
    }
    void setLocalTheta(double theta) {
        this->avg_local_theta = theta;
    }
    void setLocalPose(double theta) {
        this->avg_local_pose = theta;
    }
    void addNeighbors (int nbr) {
        this->neighbors.push_back(nbr);
    }
    void setNeighbors(std::vector<int> neighb) {
        this->neighbors = neighb;
    }
    void clearNeighbors() {
        this->neighbors.clear();
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
    geometry_msgs::Pose2D getCurrPose() {
        return this->current_pose;
    }
    double getGlobalTheta() {
        return this->avg_global_theta;
    }
    double getLocalTheta() {
        return this->avg_local_theta;
    }
    double getLocalPose() {
        return this->avg_local_pose;
    }
    std::vector<int> getNeighbors() {
        return this->neighbors;
    }
    int getDrivewayState() {
        return this->driveway_state;
    }
    double distFromAnchor() {
        return hypot((this->current_pose.x - this->localization->getAnchor().x),
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
    GOAL_ZONE_POSE getGZPose() {
        return this->goal_zone;
    }

 };


#endif //PROJECT_AGENT_H
