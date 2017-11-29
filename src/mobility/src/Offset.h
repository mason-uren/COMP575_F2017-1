//
// Created by administrator on 11/29/17.
//

#ifndef PROJECT_OFFSET_H
#define PROJECT_OFFSET_H

#include "Map.h"
#include "Agent.h"

typedef struct {
    double x;
    double y;
} OFF_POSE;


template <typename T, class S>
class Offset : private Map<T,S>, private Agent {
private:
    const OFF_POSE achilles = {0,1};
    const OFF_POSE aeneas = {-1,0};
    const OFF_POSE ajax = {1,0};
    const OFF_POSE diomedes = {1,1};
    const OFF_POSE hector = {-1,-1};
    const OFF_POSE paris = {1,-1};
public:
    Offset () {
        new Map<T,S>();
        this->addToMap(ACHILLES,  achilles);
        this->addToMap(AENEAS, aeneas);
        this->addToMap(AJAX, ajax);
        this->addToMap(DIOMEDES, diomedes);
        this->addToMap(HECTOR, hector);
        this->addToMap(PARIS, paris);
    }
};


#endif //PROJECT_OFFSET_H
