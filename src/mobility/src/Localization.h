//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_LOCALIZATION_H
#define PROJECT_LOCALIZATION_H

#include "Zone.h"

typedef struct {
    int iteration;
    double x_avg;
    double y_avg;
} LOCALIZATION_INIT;

class Localization {
private:
    std::vector<geometry_msgs::Pose2D> midpoints;
    geometry_msgs::Pose2D estimated_pose;
    geometry_msgs::Pose2D anchor_node;
    double weight;
    LOCALIZATION_INIT localization_init;

public:
    Localization (geometry_msgs::Pose2D curr, geometry_msgs::Pose2D anchor) : anchor_node(anchor), weight(0) {
        this->localization_init.iteration = 0;
        this->localization_init.x_avg = 0;
        this->localization_init.y_avg = 0;
    }
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
    void setWeight(double w) {
        this->weight = w;
    }
    void setIter(int iter) {
        this->localization_init.iteration = iter;
    }
    void setAverages(double x, double y) {
        this->localization_init.x_avg = x;
        this->localization_init.y_avg = y;
    }
    void addMidpoint(double xAvg, double yAvg) {
        geometry_msgs::Pose2D pose;
        pose.x = xAvg;
        pose.y = yAvg;
        this->midpoints.push_back(pose);
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
    double getWeight() {
        return this->weight;
    }
    LOCALIZATION_INIT getInit() {
        return this->localization_init;
    }
    std::vector<geometry_msgs::Pose2D> getMidpoints() {
        return this->midpoints;
    }
};


#endif //PROJECT_LOCALIZATION_H
