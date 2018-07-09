/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TCP.h
 * Author: xujiaji2
 *
 * Created on March 30, 2018, 11:08 AM
 */

#ifndef TCP_H
#define TCP_H

#include "Ant.h"

#define ITERATION_NUM 100  // total number of iterations
#define ANT_NUM 100  // number of ant for every iteration
#define Q 1e5  // constant tuning pheromone left for each ant
#define ROU 0.8  // constant tuning the amount of pheromone left after each iteration

class TCP {
private:
    Ant ants[ANT_NUM];
    Ant best_ant; // the ant in this iteration that has minimum travel time
    unsigned delivery_number, depot_number;
    double best_time;
public:
    void Init();
    void Update_pheromone_info();
    void Search();
    Ant& get_best_ant();
};

#endif /* TCP_H */
