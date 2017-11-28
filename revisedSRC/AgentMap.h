//
// Created by Mason U'Ren on 11/8/17.
//

#ifndef PROJECT_AGENTMAP_H
#define PROJECT_AGENTMAP_H

#include "Map.h"
#include "Agent.h"
//#include "ZoneMap.h"

template <typename T, class S>
class AgentMap : public Map<T, S>, private Agent {
private:

public:
    AgentMap () : Map<T,S>() {}
//    AgentMap () {
//        new Map<T, S>();
//    }
};


#endif //PROJECT_AGENTMAP_H
