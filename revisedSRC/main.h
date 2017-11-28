//
// Created by Mason U'Ren on 11/8/17.
//

#ifndef PROJECT_MAIN_H
#define PROJECT_MAIN_H

#include <map>
#include <vector>
#include <string>
#include <queue>
#include <iostream>

#include "ZoneMap.h"
#include "AgentMap.h"
#include "Driveway.h"
//#include "Map.h"

#define TRAVERSE_STD 0.15
#define TRAVERSE_ERR 0.1
#define ORIENTATION_ERR 0.05
#define MAX_ITER 30

#define LOCALIZATION_ERROR 0.5

//typedef enum {
//    SEARCH = 0, PICK_UP, FIND_HOME, OBSTACLE_AVOIDANCE
//} STATES;

typedef enum {
    ACHILLES = 0, AJAX, MASON, TURD_SANDO, AENEUS, PETER
} ROVER_NAME;

double tangentialDist(POSE a, POSE b) {
    return std::hypot((a.x - b.x), (a.y - b.y));
}

double angle


#endif //PROJECT_MAIN_H
