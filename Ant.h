/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Ant.h
 * Author: xujiaji2
 *
 * Created on March 30, 2018, 11:08 AM
 */

#ifndef ANT_H
#define ANT_H

#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "m3.h"
#include "m4.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <cmath>

#define MAX_SIZE 500  // assume number of delivery not exceeding this number
#define ALPHA 0.8  // constant tuning strength of pheromone
#define BETA 1  // constant tuning visivlity of the path

extern double pheromone_info[MAX_SIZE][MAX_SIZE];
extern double travel_time_info[MAX_SIZE][MAX_SIZE];
extern double geo_distance_info[MAX_SIZE][MAX_SIZE];
extern std::vector<DeliveryInfo> deliveries_global;
extern std::vector<unsigned> depots_global;
extern double turn_penalty_global;

class Ant {
private:
    unsigned delivery_number, depot_number;  // total number
    std::vector<IntersectionIndex> path;  // The path this ant currently gone, store IntersectionIndex!
    std::vector<unsigned> path_id;  // store the id arranged by 2*delivery+depot
    IntersectionIndex start_depot, end_depot;  // This is index from input
    std::vector<bool> visited_pick;  // visited pick-up point, true for visited
    std::vector<bool> visited_drop;  // visited drop-off point
    double total_time;  // current time this ant travels
    IntersectionIndex current_intersection;
    unsigned current_intersection_index;  // range from 0 to 2*delivery_number+depot_number, align with the order of pheromone_info vector
    unsigned move_count;  // current moves of this ant

public:
    void Init();
    void Move();
    IntersectionIndex choose_next_intersection();
    void Back_to_depot();
    void Travel();
    std::vector<IntersectionIndex> & get_path();
    std::vector<unsigned> & get_path_id();
    double get_total_time();
};

#endif /* ANT_H */
