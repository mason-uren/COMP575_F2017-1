//
// Created by administrator on 11/28/17.
//

#ifndef PROJECT_AGENTQUEUEINTERFACE_H
#define PROJECT_AGENTQUEUEINTERFACE_H

#include <queue>
#include <iostream>
#include <vector>
#include <algorithm>

template <typename T>
class AgentQueueInterface {
private:
    std::queue<T> queue;
    std::vector<T> vector;

public:
    AgentQueueInterface () : queue(std::queue<T>()), vector(std::vector<T>()) {}

    /*
     * Setters
     */
    void addToQueue(T key) {
        try {
            if (std::find(this->vector.begin(), this->vector.end(), key) != this->vector.end()) { // Check to see if key is already in queue
                throw key;
            }
            this->vector.push_back(key);
            this->queue.push(key);
        } catch (T key) {
            std::cout << "ERROR: agent '" << key << "' already in queue." << std::endl;
        }
    }

    /*
     * Getters
     */
    T removeFromQueue(T key) {
        try {
            if (std::find(this->vector.begin(), this->vector.end(), key) != this->vector.end()) { // Check to see if key is already in queue
                throw key;
            }
            // Iterate through queue to find value, then rebuild queue
            std::queue<T> temp_queue = this->queue;
            int fID;
            int ID;
            while (true) {
                ID = temp_queue.front();
                if (temp_queue.empty()) { // If queue is empty then done
                    break;
                }
                else if (ID == key) { // Check if appropriate value
                    fID = ID;
                    temp_queue.pop();
                }
                else { // Keep iterating
                    this->queue.push(ID);
                    temp_queue.pop();
                }
            }
            return fID;
        } catch (T key) {
            std::cout << "ERROR: agent '" << key << "' is not next in the queue." << std::endl;
        }
    }
    T removeQueueFront() {
        try {
            if (this->queue.empty()) {
                throw;
            }
            int ID = this->queue.front();
            this->queue.pop();
            this->vector.erase(std::remove(this->vector.begin(), this->vector.end(), ID), this->vector.end());
            return ID;
        } catch (...) {
            std::cout << "ERROR: queue is empty, cannot pop from front." << std::endl;
            return -1; // Flagged bad value
        }
    }
    bool isEmpty() {
        return this->queue.empty();
    }
};


#endif //PROJECT_AGENTQUEUEINTERFACE_H
