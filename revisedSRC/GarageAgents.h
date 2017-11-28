//
// Created by Mason U'Ren on 11/13/17.
//

#ifndef PROJECT_DELIVERINGAGENTS_H
#define PROJECT_DELIVERINGAGENTS_H

#include "AgentQueueInterface.h"

template <typename T>
class GarageAgents : public AgentQueueInterface<T> {
public:
    GarageAgents () {
        new AgentQueueInterface<int>();
    }
};


#endif //PROJECT_DELIVERINGAGENTS_H
