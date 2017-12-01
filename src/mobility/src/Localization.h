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
    double confidence;
    int iteration;
    LOC_SUBSTATE substate;

public:
    Localization (geometry_msgs::Pose2D curr, geometry_msgs::Pose2D anchor) :
            anchor_node(anchor), confidence(0), substate(ANCHORING), iteration(0) {}
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
    }
    void setConfidence(double dist_toAnchor) {
        if (dist_toAnchor < 0.01) {
            this->confidence = 1;
        }
        else {
            this->confidence = 1 / dist_toAnchor;
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

    /*
     * Getters
     */
    geometry_msgs::Pose2D getEstimated() {
        return this->estimated_pose;
    }
    geometry_msgs::Pose2D getAnchor() {
        return this->anchor_node;
    }
    double getConfidence() {
        return this->confidence;
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
};


#endif //PROJECT_LOCALIZATION_H
