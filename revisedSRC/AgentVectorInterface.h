//
// Created by Mason U'Ren on 11/13/17.
//

#ifndef PROJECT_AGENTVECTOR_H
#define PROJECT_AGENTVECTOR_H


#include <vector>
#include <iostream>

template <typename T>
class AgentVectorInterface {
private:
    std::vector<T> vector;
public:
    AgentVectorInterface () {
        this->vector = std::vector<T>();
    }

    /*
     * Setters
     */
    void addToVector (T key) {
        try {
            if (std::find(this->vector.begin(), this->vector.end(), key) != this->vector.end()) { // Check to see if key is already in vector
                throw key;
            }
            this->vector.push_back(key);
        }
        catch (T key) {
            std::cout << "ERROR: agent '" << key << "' already in vector." << std::endl;
        }
    }

    /*
     * Getters
     */
    T removeFromVector (T key) {
        try {
            if (std::find(this->vector.begin(), this->vector.end(), key) == this->vector.end()) {
                throw key;
            }
            this->vector.erase(std::remove(this->vector.begin(), this->vector.end(), key), this->vector.end());
            return key;
        }
        catch (T key) {
            std::cout << "ERROR: agent '" << key << "' not in vector." << std::endl;
            return -1; // Flagged bad value
        }

    }

    bool isEmpty() {
        return this->vector.empty();
    }


};


#endif //PROJECT_AGENTVECTOR_H
