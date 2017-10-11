//
// Created by administrator on 10/10/17.
//

#ifndef PROJECT_MOBILITY_H
#define PROJECT_MOBILITY_H

// achilles, aeneas, ajax <-- *** CORRECTLY SPELLED NAMES ***
typedef enum {
    ACHILLES = 0, AENEAS, AJAX
} ROVER_NAME;


typedef struct rover_refs {
    ROVER_NAME name;
    geometry_msgs::Pose2D current_pose;
    double global_heading;
} ROVER_REFS;



#endif //PROJECT_MOBILITY_H
