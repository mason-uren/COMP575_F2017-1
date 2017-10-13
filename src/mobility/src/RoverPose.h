//
// Created by administrator on 10/12/17.
//

#ifndef PROJECT_ROVERPOSE_H
#define PROJECT_ROVERPOSE_H

class RoverPose {


    public:
        double x;
        double y;
        double theta;
        std::vector<std::vector<int> > neighbors;

        RoverPose (std::vector<double> pose) : x(pose[0]), y(pose[1]), theta(pose[2]){
            this->x = 0;
            this->y = 0;
            this->theta = 0;
        }
        RoverPose() {}

};

#endif //PROJECT_ROVERPOSE_H
