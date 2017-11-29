//
// Created by Mason U'Ren on 11/15/17.
//

#ifndef PROJECT_LOCAL_H
#define PROJECT_LOCAL_H

#include "Zone.h"

typedef struct {
    int iterations;
    double x_avg;
    double y_avg;
} LOCALIZATION_INIT;

class Localization {
private:
    // Variables
    std::vector<POSE> midpoints;
    POSE estimated_pose;
    POSE anchor_node;
    double weight;
    LOCALIZATION_INIT localization_init;
public:
    Localization (POSE curr, POSE anchor) : anchor_node(anchor), weight(0) {
        this->localization_init.iterations = 0;
        this->localization_init.x_avg = 0;
        this->localization_init.y_avg = 0;
    }
    Localization () {}

    /*
     * Setters
     */
    void setEstimated(double xAvg, double yAvg) {
        POSE pose;
        pose.x = xAvg;
        pose.y = yAvg;
        this->estimated_pose = pose;
    }
    void setAnchor(double x, double y) {
        POSE pose;
        pose.x = x;
        pose.y = y;
        this->anchor_node = pose;
    }
    void setWeight(double w) {
        this->weight = w;
    }
    void setIter(int iter) {
        this->localization_init.iterations = iter;
    }
    void setAverages(double x, double y) {
        this->localization_init.x_avg = x;
        this->localization_init.y_avg = y;
    }
    void addMidpoint(double xAvg, double yAvg) {
        POSE pose;
        pose.x = xAvg;
        pose.y = yAvg;
        this->midpoints.push_back(pose);
    }

    /*
     * Getters
     */
    POSE getEstimated() {
        return this->estimated_pose;
    }
    POSE getAnchor() {
        return this->anchor_node;
    }
    double getWeight() {
        return this->weight;
    }
    LOCALIZATION_INIT getInit() {
        return this->localization_init;
    }
    std::vector<POSE> getMidpoints() {
        return this->midpoints;
    }


};


#endif //PROJECT_LOCAL_H
