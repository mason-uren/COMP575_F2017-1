//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_AGENTMAP_H
#define PROJECT_AGENTMAP_H

#include <cmath>
#include "Map.h"
#include "Agent.h"

template <typename T, class S>
class AgentMap : private Map<T,S>, private Agent {
private:

public:
    AgentMap () {
        new Map<T,S>();
    }
};


#endif //PROJECT_AGENTMAP_H
