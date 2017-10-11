//
// Created by administrator on 10/11/17.
//

#ifndef PROJECT_ROVERATTR_H
#define PROJECT_ROVERATTR_H

#include <geometry_msgs/Pose2D.h>

#define NUM_ROVERS 3

// achilles, aeneas, ajax <-- *** CORRECTLY SPELLED NAMES ***
typedef enum {
    ACHILLES = 0, AENEAS, AJAX
} AGENT_NAME;


typedef struct agent_refs {
    AGENT_NAME name;
    geometry_msgs::Pose2D current_pose;
    double global_heading;
} AGENT_REFS;

//std::vector<AGENT_REFS> swarm(NUM_ROVERS);

class Swarm {
    public:
        std::vector<AGENT_REFS> swarm_vector;
//        AGENT_REFS agent_refs;
};

#endif //PROJECT_ROVERATTR_H
