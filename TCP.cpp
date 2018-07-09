/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "TCP.h"

void TCP::Init() {
    delivery_number = deliveries_global.size();
    depot_number = depots_global.size();
    best_time = 1e10;
    unsigned temp_count = 0;
    // Initialize the intersection travel time info
    for (unsigned i = 0; i < 2*delivery_number+depot_number; i++) {
        for (unsigned j = 0; j <= i; j++) {
            // Do not initiate the value from depot to depot
            if (i<2*delivery_number || j<2*delivery_number) {
                // Store all geo distances between delivery points, so that do not need this calculation when moving ants
                geo_distance_info[i][j] = find_distance_between_two_points(
                        getIntersectionPosition((i<2*delivery_number) ? ((i<delivery_number)?deliveries_global[i].pickUp:deliveries_global[i-delivery_number].dropOff) : depots_global[i-2*delivery_number]), 
                        getIntersectionPosition((j<2*delivery_number) ? ((j<delivery_number)?deliveries_global[j].pickUp:deliveries_global[j-delivery_number].dropOff) : depots_global[j-2*delivery_number]));
                // // Store all travel time data between delivery points, which is definitely too slow when there are many points
                // // Will likely to exceed time limit when more than 30 deliveries
                // travel_time_info[i][j] = -1;
                //     compute_path_travel_time(find_path_between_intersections(
                //             (i<2*delivery_number) ? ((i<delivery_number)?deliveries_global[i].pickUp:deliveries_global[i-delivery_number].dropOff) : depots_global[i-2*delivery_number],
                //             (j<2*delivery_number) ? ((j<delivery_number)?deliveries_global[j].pickUp:deliveries_global[j-delivery_number].dropOff) : depots_global[j-2*delivery_number],
                //             turn_penalty_global), turn_penalty_global);
                geo_distance_info[j][i] = geo_distance_info[i][j];
            }
        }

    // Initialize the pheromone value
    for (unsigned i = 0; i < 2*delivery_number+depot_number; i++) {
        for (unsigned j = 0; j < 2*delivery_number+depot_number; j++) {
            // Do not initiate the value from depot to depot
            if (i < 2*delivery_number || j < 2*delivery_number) {
                pheromone_info[i][j] = 1.0;  // Initial value of pheromone to be 1
            }
        }
    }
}

void TCP::Update_pheromone_info() {
    double cur_iteration_pheromone[MAX_SIZE][MAX_SIZE];
    unsigned row, col;
    // Loop through all the ants to store all updates in current loop
    for (unsigned ant_count = 0; ant_count < ANT_NUM; ant_count++) {
        // Loop through the path
        for (unsigned path_count = 0; path_count < 2*delivery_number+1; path_count++) {
            row = ants[ant_count].get_path_id()[path_count];
            col = ants[ant_count].get_path_id()[path_count+1];
            cur_iteration_pheromone[row][col] += Q/ants[ant_count].get_total_time();
            // If the start point of this move is a pick-up, then if next time the ant is at the destination point, it is still encouraged to get here
            // Therefore also enhance the pheromone
            // However if the start is a drop-off, then next time we don't know if the cargo is picked up, so not encouraged to go this way
            if (row < delivery_number) 
                cur_iteration_pheromone[col][row] += Q/ants[ant_count].get_total_time();
        }
    }
    
    // Update the phermone info
    for (unsigned i=0; i < 2*delivery_number+depot_number; i++) {
        for (unsigned j=0; j < 2*delivery_number+depot_number; j++) {
            // Do not add the value for depot to depot
            if (i<2*delivery_number || j<2*delivery_number) {
                // New pheromone value = old pheromone value * left parameter + new phermone in this iteration
                pheromone_info[i][j] = pheromone_info[i][j] * ROU + cur_iteration_pheromone[i][j];
            }
        }
    }
}

void TCP::Search() {  // Entry of the general path finding process
    for (unsigned it_count = 0; it_count < ITERATION_NUM; it_count++) {
        for (unsigned ant_count = 0; ant_count < ANT_NUM; ant_count++) {
            std::cout<<"after next loop starts\n";
            ants[ant_count].Travel();
            std::cout<<"before exiting previous loop\n";
        }
        // store the best result
        std::cout<<"ant travel for one iteration finish" <<std::endl;
        for (unsigned ant_count = 0; ant_count < ANT_NUM; ant_count++) {
            if(best_time > ants[ant_count].get_total_time()) {
                best_ant = ants[ant_count];  // shallow copy?
                best_time = best_ant.get_total_time();
				// std::cout << "New best time: " << best_time << " s" << std::endl;
            }
        }
        // std::cout << "store best ant for one iteration finish" << std::endl;
        Update_pheromone_info();
        std::cout << "Current minimum time: " << best_ant.get_total_time() << " s" << std::endl;
    }
}

Ant& TCP::get_best_ant() {
    return best_ant;
}
