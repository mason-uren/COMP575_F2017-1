//
// Created by administrator on 10/12/17.
//

#ifndef PROJECT_ROVERPOSE_H
#define PROJECT_ROVERPOSE_H

#include <geometry_msgs/Pose2D.h>

typedef enum {
    F = 0, T, UNKNOWN
};

class RoverPose {


    public:
        geometry_msgs::Pose2D rover_pose;
        double avg_global_theta;
        double avg_local_theta;
        double avg_local_pose;
        double separation;
        int new_lead;
        std::vector<int> possible_lead;
        std::vector<int> neighbors;

        explicit RoverPose (geometry_msgs::Pose2D pose) : rover_pose(pose), new_lead(UNKNOWN){
            static const int arr[] = {-1,-1,-1}; // Default bad values
            this->neighbors = std::vector<int>(arr, arr + sizeof(arr) / sizeof(arr[0]));
        }
        RoverPose() {}

};

#endif //PROJECT_ROVERPOSE_H
