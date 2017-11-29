//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_AGENTSTATES_H
#define PROJECT_AGENTSTATES_H

typedef enum {
    STATE_INIT = 0, STATE_SEARCH, STATE_PICK_UP, STATE_FIND_HOME, STATE_OBSTACLE_AVOIDANCE, STATE_DROP_OFF, STATE_LEAVE_HOME
} STATE_ID;

class AgentStates {
private:
    STATE_ID state_id;

public:
    AgentStates () : state_id(STATE_INIT) {}

    /*
     * Setters
     */
    void setState(STATE_ID id) {
        this->state_id = id;
    }

    /*
     * Getters
     */
    STATE_ID getState() {
        return this->state_id;
    }
};


#endif //PROJECT_AGENTSTATES_H
