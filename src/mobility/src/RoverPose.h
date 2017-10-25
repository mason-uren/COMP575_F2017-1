//
// Created by administrator on 10/12/17.
//

#ifndef PROJECT_ROVERPOSE_H
#define PROJECT_ROVERPOSE_H

#include <geometry_msgs/Pose2D.h>

class RoverPose {


    public:
        geometry_msgs::Pose2D rover_pose;
        double avg_global_theta;
        double avg_local_theta;
        std::vector<int> neighbors;
    
        explicit RoverPose (geometry_msgs::Pose2D pose) : rover_pose(pose){
            static const int arr[] = {-1,-1,-1}; // Default bad values
            this->neighbors = std::vector<int>(arr, arr + sizeof(arr) / sizeof(arr[0]));
        }
        RoverPose() {}

};

#endif //PROJECT_ROVERPOSE_H
