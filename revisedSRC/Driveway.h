//
// Created by Mason U'Ren on 11/8/17.
//

#ifndef PROJECT_DRIVEWAY_H
#define PROJECT_DRIVEWAY_H

#include <queue>
#include <iostream>

#include "AgentMap.h"

#include "ActiveAgents.h"
#include "WaitingAgents.h"
#include "GarageAgents.h"
#include "InnerRadius.h"

typedef enum {
    INIT = 0, ACTIVE, WAITING, GARAGE, DELIVERY
} DRIVEWAY_TYPE;

template <typename T>
class Driveway :
        private ActiveAgents<T>, private WaitingAgents<T>,
        private GarageAgents<T>, public InnerRadius<std::string, CRITICAL_POINTS>{
private:
    // Variables
    double r_out;
//    bool enter;
    bool exit;
    int delivery; // Set to non-existent agent
    int spins;
    InnerRadius<std::string, CRITICAL_POINTS> *r_in;
    ActiveAgents<T> *activeAgents;
    WaitingAgents<T> *waitingAgents;
    GarageAgents<T> *garageAgents;

public:
    Driveway () :
            activeAgents(new ActiveAgents<T>()), waitingAgents(new WaitingAgents<T>),
            garageAgents(new GarageAgents<T>), r_in(new InnerRadius<std::string, CRITICAL_POINTS>),
            delivery(-1), r_out(R3), exit(false), spins(0) {}

//    Driveway () : r_out(R3), exit(false), spins(0) {
//        activeAgents = new ActiveAgents<T>();
//        waitingAgents = new WaitingAgents<T>();
//        garageAgents = new GarageAgents<T>;
//        r_in = new InnerRadius<std::string, CRITICAL_POINTS>();
//        delivery = -1;
//    }
    /*
     * Setters
     */
    Agent addToDriveway(Agent robot, DRIVEWAY_TYPE type) {
         switch (type){
             case ACTIVE:
                 this->activeAgents->addToVector(robot.getID());
                 robot.setDrivewayState(ACTIVE);
                 break;
             case WAITING:
                 this->waitingAgents->addToQueue(robot.getID());
                 robot.setDrivewayState(WAITING);
                 break;
             default:
                 std::cout << "ERROR: can only add directly to 'ACTIVE' and 'WAITING'." << std::endl;
                 break;
         }
        return robot;
    }
    Agent removeFromDriveway(Agent robot, DRIVEWAY_TYPE type) {
        switch (type) {
            case ACTIVE:
                this->activeAgents->removeFromVector(robot.getID());
                break;
            case WAITING:
                this->waitingAgents->removeQueueFront();
            case GARAGE:
                this->garageAgents->removeQueueFront();
                break;
            case DELIVERY:
                this->delivery = -1;
                break;
            default:
                std::cout << "ERROR: not a valid driveway." << std::endl;
                break;
        }
        robot.setDrivewayState(0);
        return robot;
    }

    Agent moveAgent(Agent, DRIVEWAY_TYPE, DRIVEWAY_TYPE);


    /*
     * Getters
     */

    bool canEnter(DRIVEWAY_TYPE type) {
        switch (type) {
            case INIT:
            case WAITING:
                return this->activeAgents->isEmpty();
            case ACTIVE:
                break;
            case GARAGE:
                return this->delivery == -1;
            case DELIVERY:
                break;
            default:
                break;
        }
        return false;
    }

    bool canExit() {

        return exit;
    }
};

#include "Driveway.cpp"
#endif //PROJECT_DRIVEWAY_H
