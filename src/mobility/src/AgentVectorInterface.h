//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_AGENTVECTORINTERFACE_H
#define PROJECT_AGENTVECTORINTERFACE_H

#include <vector>
#include <iostream>
#include <algorithm>

template <typename T>
class AgentVectorInterface {
private:
    std::vector<T> vector;
public:
    AgentVectorInterface () : vector(std::vector<T>()) {}

    /*
     * Setters
     */
    void addToVector(T key) {
        try {
            if (std::find(this->vector.begin(), this->vector.end(), key) != this->vector.end()) { // Check to see if key is already in vector
                throw key;
            }
            this->vector.push_back(key);
        } catch (T key) {
            std::cout << "ERROR: agent '" << key << "' already in vector." << std::endl;
        }
    }


    /*
     * Getters
     */
    T removeFromVector(T key) {
        try {
            if (std::find(this->vector.begin(), this->vector.end(), key) == this->vector.end()) {
                throw key;
            }
            this->vector.erase(std::remove(this->vector.begin(), this->vector.end(), key), this->vector.end());
        } catch (T key) {
            std::cout << "ERROR: agent '" << key << "' not in vector." << std::endl;
            return -1; // Flagged bad value
        }
    }
    bool isEmpty() {
        return this->vector.empty();
    }
};


#endif //PROJECT_AGENTVECTORINTERFACE_H
