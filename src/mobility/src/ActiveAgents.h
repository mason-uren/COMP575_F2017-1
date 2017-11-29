//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_ACTIVEAGENTS_H
#define PROJECT_ACTIVEAGENTS_H

#include "AgentVectorInterface.h"

template <typename T>
class ActiveAgents : protected AgentVectorInterface<T> {
public:
    ActiveAgents () {
        new AgentVectorInterface<T>();
    }
};


#endif //PROJECT_ACTIVEAGENTS_H
