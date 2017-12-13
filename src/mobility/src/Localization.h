//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_LOCALIZATION_H
#define PROJECT_LOCALIZATION_H

#include "Zone.h"

typedef enum {
    ANCHORING = 0, WEIGHTING, BEGINNING
} LOC_SUBSTATE;

class Localization {
private:
    std::vector<geometry_msgs::Pose2D> midpoints;
    geometry_msgs::Pose2D estimated_pose;
    geometry_msgs::Pose2D anchor_node;
    bool anchor_set;
    double anchor_confidence;
    double goal_confidence;
    double vel_confidence;
    int iteration;
    LOC_SUBSTATE substate;

public:
    Localization (geometry_msgs::Pose2D curr, geometry_msgs::Pose2D anchor) :
            anchor_node(anchor), anchor_confidence(0), goal_confidence(0), vel_confidence(0),
            iteration(0), substate(ANCHORING), anchor_set(false) {}
    Localization () {}

    /*
     * Setters
     */
    void setEstimated(double xAvg, double yAvg) {
        geometry_msgs::Pose2D pose;
        pose.x = xAvg;
        pose.y = yAvg;
        this->anchor_node = pose;
    }
    void setAnchor(double x, double y) {
        geometry_msgs::Pose2D pose;
        pose.x = x;
        pose.y = y;
        this->anchor_node = pose;
        this->anchor_set = true;
    }
    void setAnchConfidence(double dist_toAnchor) {
        if (dist_toAnchor < 0.01) {
            this->anchor_confidence = 1;
        }
        else {
            this->anchor_confidence = 1 / dist_toAnchor;
        }
    }
    void incrmtIter() {
        this->iteration++;
    }
    void resetIter() {
        this->iteration = 0;
    }
    void addMidpoint(double xAvg, double yAvg) {
        geometry_msgs::Pose2D pose;
        pose.x = xAvg;
        pose.y = yAvg;
        this->midpoints.push_back(pose);
    }
    void advanceSubstate() {
        switch (this->substate) {
            case ANCHORING:
                this->substate = WEIGHTING;
                break;
            case WEIGHTING:
                this->substate = BEGINNING;
                break;
        }
    }
    void setSubstate (LOC_SUBSTATE sub) {
        this->substate = sub;
    }

    /*
     * Getters
     */
    geometry_msgs::Pose2D getEstimated() {
        return this->estimated_pose;
    }
    geometry_msgs::Pose2D getAnchor() {
        return this->anchor_node;
    }
    double getAnchConfidence() {
        return this->anchor_confidence;
    }
    int getIter() {
        return this->iteration;
    }
    std::vector<geometry_msgs::Pose2D> getMidpoints() {
        return this->midpoints;
    }
    LOC_SUBSTATE getSubstate() {
        return this->substate;
    }
    bool isAnchor() {
        return this->anchor_set;
    }
};


#endif //PROJECT_LOCALIZATION_H
