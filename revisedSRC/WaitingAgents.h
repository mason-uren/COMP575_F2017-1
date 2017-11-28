//
// Created by Mason U'Ren on 11/13/17.
//

#ifndef PROJECT_WAITINGAGENTS_H
#define PROJECT_WAITINGAGENTS_H

#include "AgentQueueInterface.h"

template <typename T>
class WaitingAgents : public AgentQueueInterface<T> {
public:
    WaitingAgents () {
        new AgentQueueInterface<int>();
    }
};


#endif //PROJECT_WAITINGAGENTS_H
