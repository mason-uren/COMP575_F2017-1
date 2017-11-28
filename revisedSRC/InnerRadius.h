//
// Created by Mason U'Ren on 11/15/17.
//

#ifndef PROJECT_INNERRADIUS_H
#define PROJECT_INNERRADIUS_H

#include "Map.h"

#define CP 1.1 // Roughly calculated to have the rover traverse in the center of the driveway
#define CPR 1.544 // Traversal path

typedef struct {
    POSE left_CP;
    POSE middle_CP;
    POSE right_CP;
} CRITICAL_POINTS;

template <typename T, class S>
class InnerRadius : private Map<T,S> {
private:
    std::string chosen_zone;
    CRITICAL_POINTS cps;
public:
    InnerRadius () : Map<T,S>() {
        this->addToMap("a", this->setCP((POSE) {-CP, CP, 0}, (POSE) {-RZ, 0, 0}, (POSE) {-CP, -CP, 0}));
        this->addToMap("b", this->setCP((POSE) {-CP, -CP, 0}, (POSE) {0, -RZ, 0}, (POSE) {CP, -CP, 0}));
        this->addToMap("c", this->setCP((POSE) {CP, -CP, 0}, (POSE) {RZ, 0, 0}, (POSE) {CP, CP, 0}));
        this->addToMap("d", this->setCP((POSE) {CP, CP, 0}, (POSE) {0, RZ, 0}, (POSE) {-CP, CP, 0}));
    }

    /*
     * Setters
     */
    CRITICAL_POINTS setCP(POSE cpl, POSE cpm, POSE cpr) {
        this->cps.left_CP = cpl;
        this->cps.middle_CP = cpm;
        this->cps.right_CP = cpr;
        return cps;
    }

    /*
     * Getters
     */
    CRITICAL_POINTS getCP (T key) {
        return this->getValue(key);
    }

};


#endif //PROJECT_INNERRADIUS_H
