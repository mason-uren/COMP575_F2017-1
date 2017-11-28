//
// Created by Mason U'Ren on 11/8/17.
//

#include "Driveway.h"

template <typename T>
Agent Driveway<T>::moveAgent(Agent robot, DRIVEWAY_TYPE from, DRIVEWAY_TYPE to) {
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
                delivery = -1;
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
                    delivery = robot.getID();
                    break;
                default:
                    break;
            }
        }
    }
    catch (int ID) {
        std::cout << "ERROR: failed to move agent '" << robot.getID() << "' from " << from << " to " << to << "." << std::endl;
    }
    return robot;
}
