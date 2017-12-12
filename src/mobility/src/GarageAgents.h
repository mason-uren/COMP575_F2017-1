//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_GARAGEAGENTS_H
#define PROJECT_GARAGEAGENTS_H

#include "AgentQueueInterface.h"

template <typename T>
class GarageAgents : public AgentQueueInterface<T> {
public:
    GarageAgents () {
        new AgentQueueInterface<T>();
    }
};


#endif //PROJECT_GARAGEAGENTS_H
