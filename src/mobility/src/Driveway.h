//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_DRIVEWAY_H
#define PROJECT_DRIVEWAY_H

#include "AgentMap.h"
#include "ActiveAgents.h"
#include "WaitingAgents.h"
#include "GarageAgents.h"
#include "InnerRadius.h"

typedef enum {
    INIT = 0, ACTIVE, WAITING, GARAGE, DELIVERY
} DRIVEWAY_TYPE;

template <typename T>
class Driveway : private  ActiveAgents<T>, private WaitingAgents<T>,
                    private  GarageAgents<T> { //, private InnerRadius<T,S> {
private:
    double r_out;
    bool exit;
    int delivery;
    int spins;
//    InnerRadius<T,S> *r_in;
    ActiveAgents<T> *activeAgents;
    WaitingAgents<T> *waitingAgents;
    GarageAgents<T> *garageAgents;

public:
    Driveway () : activeAgents(new ActiveAgents<int>()), waitingAgents(new WaitingAgents<int>()),
                  garageAgents(new GarageAgents<int>()), //r_in(new InnerRadius<std_msgs::String, CRITICAL_POINTS>()),
                  delivery(-1), r_out(R3), exit(false), spins(0) {}

    /*
     * Setters
     */
    Agent addToDriveway(Agent robot, DRIVEWAY_TYPE type) {
        switch (type) {
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
                this->waitingAgents->removeFromQueue(robot.getID());
                break;
            case GARAGE:
                this->garageAgents->removeFromQueue(robot.getID());
                break;
            case DELIVERY:
                this->delivery = -1;
                break;
            default:
                std::cout << "ERROR: not a valid driveway." << std::endl;
                break;
        }
        robot.setDrivewayState(INIT);
        return robot;
    }
    void setExitState(bool state) {
        this->exit = state;
    }
    Agent moveAgent(Agent robot, DRIVEWAY_TYPE from, DRIVEWAY_TYPE to) {
        try {
            int removed_agent;
            switch (from) {
                case ACTIVE:
                    removed_agent = this->activeAgents->removeFromVector(robot.getID());
                    break;
                case WAITING:
                    removed_agent = this->waitingAgents->removeQueueFront();
                    break;
                case GARAGE:
                    removed_agent = this->garageAgents->removeQueueFront();
                    break;
                case DELIVERY:
                    removed_agent = this->delivery;
                    this->delivery = -1; // TODO: I'm not so sure about setting 'delivery' to -1 here
                    break;
                default:
                    removed_agent = -1;
                    break;
            }
            if (removed_agent < 0) {
                throw robot.getID();
            }
            else {
                switch (to) {
                    case ACTIVE:
                        robot.setDrivewayState(ACTIVE);
                        activeAgents->addToVector(robot.getID());
                        break;
                    case WAITING:
                        robot.setDrivewayState(WAITING);
                        waitingAgents->addToQueue(robot.getID());
                        break;
                    case GARAGE:
                        robot.setDrivewayState(GARAGE);
                        garageAgents->addToQueue(robot.getID());
                        break;
                    case DELIVERY:
                        robot.setDrivewayState(DELIVERY);
                        this->delivery = robot.getID();
                        break;
                    default:
                        break;
                }
            }
        } catch (int ID) {
            std::cout << "ERROR: failed to move agent '" << robot.getID() << "' from " << from << " to " << to << "." << std::endl;
        }
        return robot;
    }

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
