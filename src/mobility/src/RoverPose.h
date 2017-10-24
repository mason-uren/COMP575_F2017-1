//
// Created by administrator on 10/12/17.
//

#ifndef PROJECT_ROVERPOSE_H
#define PROJECT_ROVERPOSE_H

class RoverPose {


    public:
        int name;
        double x;
        double y;
        double theta;
        double avg_global_theta;
        double avg_local_theta;
        std::vector<int> neighbors;

        explicit RoverPose (int name, std::vector<double> pose) : name(name), x(pose[0]), y(pose[1]), theta(pose[2]){
            static const int arr[] = {-1,-1,-1}; // Default bad values
            this->neighbors = std::vector<int>(arr, arr + sizeof(arr) / sizeof(arr[0]));
        }
        RoverPose() {}

};

#endif //PROJECT_ROVERPOSE_H
