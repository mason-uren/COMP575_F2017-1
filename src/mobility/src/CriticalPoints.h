//
// Created by administrator on 11/29/17.
//

#ifndef PROJECT_CRITICALPOINTS_H
#define PROJECT_CRITICALPOINTS_H

class CriticalPoints {
private:
    geometry_msgs::Pose2D left_cp;
    geometry_msgs::Pose2D middle_cp;
    geometry_msgs::Pose2D right_cp;

public:
    CriticalPoints (geometry_msgs::Pose2D left, geometry_msgs::Pose2D mid, geometry_msgs::Pose2D right) :
            left_cp(left), middle_cp(mid), right_cp(right) {}
    CriticalPoints () {};

    /*
     * Setters
     */
    void setCPS() {
        std::vector<geometry_msgs::Pose2D> cp_vec;
        cp_vec.push_back(this->left_cp);
        cp_vec.push_back(this->middle_cp);
        cp_vec.push_back(this->right_cp);
    }

    /*
     * Getters
     */
    geometry_msgs::Pose2D leftCP() {
        return this->left_cp;
    }
    geometry_msgs::Pose2D middleCP() {
        return this->middle_cp;
    }
    geometry_msgs::Pose2D rightCP() {
        return this->right_cp;
    }

};

#endif //PROJECT_CRITICALPOINTS_H
