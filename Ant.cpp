/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "Ant.h"

unsigned random_unsigned(unsigned low, unsigned high);
double random_double(double low, double high);

double pheromone_info[MAX_SIZE][MAX_SIZE];
double travel_time_info[MAX_SIZE][MAX_SIZE];
double geo_distance_info[MAX_SIZE][MAX_SIZE];

void Ant::Init() {
    delivery_number = deliveries_global.size();
    depot_number = depots_global.size();
    visited_pick.assign(delivery_number, false);
    visited_drop.assign(delivery_number, false);
    total_time = 0;
    move_count = 0;
    path.clear();
    path_id.clear();
    start_depot = random_unsigned(0, depot_number-1);
    // Ensure the start depot can connect to some delivery point
    bool can_move_first = false;
    while (!can_move_first) {
        for(unsigned i=0; i<delivery_number; i++){
            if((find_path_between_intersections(depots_global[start_depot], deliveries_global[i].pickUp, turn_penalty_global)).size()!=0){
                can_move_first = true;
                break;
            }
        }
        if(!can_move_first) start_depot = random_unsigned(0, depot_number-1);
    }
    current_intersection = depots_global[start_depot]; // for calculating travel time between intersections
    current_intersection_index = 2*delivery_number + start_depot; // multiply by 2 because each delivery has two intersections info, used for getting pheromone_info
    path.push_back(current_intersection);
    path_id.push_back(current_intersection_index);
}

IntersectionIndex Ant::choose_next_intersection() {
    IntersectionIndex next_intersection = NULL;  // set it to null temporarily

    // Calculate the probablity to each of the intersections not visited
    // If an intersection has been visited or a drop-off intersection doesn't have the cargo picked, then its probability is zero
    double total_probability = 0;
    std::vector<double> each_probability;
    each_probability.resize(2*delivery_number);
    // Go through pick up intersections first, then drop-off intersections
    for (unsigned pick_count = 0; pick_count < delivery_number; pick_count++) {
        if(visited_pick[pick_count] == false) {
            // // Note: phermone_info access uses the delivery intersection index sequenced by input
            // // but finding the time path still need access to the intersection id
            // // Calculate the travel time of current path and update it into the data matrix
            // if (travel_time_info[current_intersection_index][pick_count] == -1) {
            //     travel_time_info[current_intersection_index][pick_count] =
            //         compute_path_travel_time(find_path_between_intersections(
            //             current_intersection,
            //             deliveries_global[pick_count].pickUp,
            //             turn_penalty_global), turn_penalty_global);
            // }

            // The above method is abandoned because using pure geo distance is much faster than calculating the travel time
            each_probability[pick_count] = pow(pheromone_info[current_intersection_index][pick_count],ALPHA)
                                            *pow(1e4/geo_distance_info[current_intersection_index][pick_count],BETA);  // compute_path_travel_time(find_path_between_intersections(current_intersection, deliveries_global[pick_count].pickUp, turn_penalty_global), turn_penalty_global),BETA);
            total_probability += each_probability[pick_count];
        }
        else each_probability[pick_count] = 0;
    }
    for (unsigned drop_count = 0; drop_count < delivery_number; drop_count++) {
        if(visited_drop[drop_count] == false && visited_pick[drop_count] == true) {  // drop placed not visited, but pickUp place visited
            // if (travel_time_info[current_intersection_index][drop_count+delivery_number] == -1) {
            //     travel_time_info[current_intersection_index][drop_count+delivery_number] =
            //         compute_path_travel_time(find_path_between_intersections(
            //             current_intersection,
            //             deliveries_global[drop_count].dropOff,
            //             turn_penalty_global), turn_penalty_global);
            // }
            
            each_probability[delivery_number+drop_count] = pow(pheromone_info[current_intersection_index][drop_count+delivery_number],ALPHA)
                                            *pow(1e4/geo_distance_info[current_intersection_index][drop_count+delivery_number],BETA);  // compute_path_travel_time(find_path_between_intersections(current_intersection, deliveries_global[drop_count].dropOff, turn_penalty_global), turn_penalty_global),BETA);
            total_probability += each_probability[delivery_number+drop_count];
        }
        else each_probability[delivery_number+drop_count] = 0;
    }
    
    // Do a roulette choosing, the probability of each intersection being chosen is based on each probability above
    double random_probability_position = 0.0;
    if(total_probability > 0) {
        random_probability_position = random_double(0, total_probability);
        for (unsigned current_id = 0; current_id < 2*delivery_number; current_id++) {
            random_probability_position -= each_probability[current_id];
            if (random_probability_position < 0){  // <0 means the random position falls in the interval of this intersection probability
                if (current_id < delivery_number) {  // this is a pick-up point
                    next_intersection = deliveries_global[current_id].pickUp;
                    visited_pick[current_id] = true;  // set this intersection as visited;
                }
                else {  // this is a drop-off point
                    next_intersection = deliveries_global[current_id-delivery_number].dropOff;
                    visited_drop[current_id-delivery_number] = true;  // set this intersection as visited;
                }
                current_intersection_index = current_id;
                break;
            }
        }
    }

    // If all the pheromone info are very small (this shouldn't happen)
    if (next_intersection == NULL)
        std::cout << "No pheromone anywhere or probability deduction wrong" << std::endl;
    return next_intersection;
}

void Ant::Move() {  // process of an ant moving to the next intersection
    IntersectionIndex next_intersection = choose_next_intersection(); // generate the next intersection the ant will go
    path.push_back(next_intersection);  // add this intersection to the path this ant goes
    path_id.push_back(current_intersection_index);
    // // update the total travel time
    // if (travel_time_info[path_id[move_count]][path_id[move_count+1]] == -1) {
    //     std::cout << "recalculation, something wrong"<<std::endl;
    //     travel_time_info[path_id[move_count]][path_id[move_count+1]] =
    //         compute_path_travel_time(find_path_between_intersections(
    //             path[move_count],path[move_count+1],
    //             turn_penalty_global), turn_penalty_global);
    // }
    total_time += compute_path_travel_time(find_path_between_intersections(path[move_count], path[move_count+1], turn_penalty_global), turn_penalty_global); 
    current_intersection = next_intersection;
    move_count++;
}

void Ant::Back_to_depot() {
    if (path.size() != 2*delivery_number+1) {
        std::cout << "must be something wrong with path creating" << std::endl;
        return;
    }
    IntersectionIndex closest_depot;
    // double temp_min_time = 1e10;
    double temp_min_distance = 1e10;
    // std::vector<unsigned> temp_shortest_path;
    for (unsigned depot_count =0; depot_count < depot_number; depot_count++) {
        // No way back to depot from this intersection
        if((find_path_between_intersections(current_intersection, depots_global[depot_count], turn_penalty_global)).size() == 0) continue;
        // // The method to calculate travel time, abandoned same as above
        // if (travel_time_info[path_id[move_count]][2*delivery_number+depot_count]==-1) {
        //         travel_time_info[path_id[move_count]][2*delivery_number+depot_count] =
        //             compute_path_travel_time(find_path_between_intersections(
        //                 current_intersection, depots_global[depot_count],
        //                     turn_penalty_global), turn_penalty_global);
        //     }
        // double temp_time = travel_time_info[path_id[move_count]][2*delivery_number+depot_count]; //compute_path_travel_time(temp_path, turn_penalty_global);
        // if (temp_time < temp_min_time) {
        //     temp_min_time = temp_time;
        //     closest_depot = depots_global[depot_count];
        //     end_depot = depot_count;
        //     current_intersection_index = 2*delivery_number+depot_count;
        // }
        double temp_distance = geo_distance_info[path_id[move_count]][2*delivery_number+depot_count];
        if (temp_distance < temp_min_distance) {
            temp_min_distance = temp_distance;
            closest_depot = depots_global[depot_count];
            end_depot = depot_count;
            current_intersection_index = 2*delivery_number+depot_count;
        }
    }
    path.push_back(closest_depot);
    path_id.push_back(current_intersection_index);
    current_intersection = closest_depot;
    if (temp_min_distance == 1e10) std::cout << "probably something wrong back to depot" <<std::endl;
    total_time += compute_path_travel_time(find_path_between_intersections(path[move_count], closest_depot, turn_penalty_global), turn_penalty_global);
    move_count++;
}

void Ant::Travel() {  // Entry of an ant traveling for a delivery
    Init();
    while(move_count < 2*delivery_number) {  // not until all deliveries are made
        Move();
        // std::cout << move_count;
    }
    Back_to_depot();
}

unsigned random_unsigned(unsigned low, unsigned high) {  // generate a random unsigned number within the limit
    return low + rand() % (high - low + 1);
}

double random_double(double low, double high) {  // generate a random double, 0.001 precision, within the limit
    unsigned difference_100 = (unsigned)(high*1000) - (unsigned)(low*1000);
    double rand_difference = double(rand()%(difference_100 + 1)) / 1000.0;
    return low + rand_difference;
}

std::vector<IntersectionIndex> & Ant::get_path() {
    return path;
}

std::vector<unsigned> & Ant::get_path_id() {
    return path_id;
}

double Ant::get_total_time() {
    return total_time;
}
