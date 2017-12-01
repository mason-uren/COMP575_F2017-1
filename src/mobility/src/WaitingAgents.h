//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_WAITINGAGENTS_H
#define PROJECT_WAITINGAGENTS_H

#include "AgentQueueInterface.h"

template <typename T>
class WaitingAgents : public AgentQueueInterface<T> {
public:
    WaitingAgents () {
        new AgentQueueInterface<T>();
    }
};


#endif //PROJECT_WAITINGAGENTS_H
