/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "TCP.h"
#include <vector>
#include <time.h>

#define MAX_SIZE 500  // assume number of delivery not exceeding this number

double best_time;
double turn_penalty_global;
std::vector<unsigned> best_route_found;
std::vector<DeliveryInfo> deliveries_global;
std::vector<unsigned> depots_global;
unsigned global_count = 0;

std::vector<unsigned> travel_route_ant_colony(const std::vector<DeliveryInfo>& deliveries, 
                                        const std::vector<unsigned>& depots, 
                                        const float turn_penalty) {
    srand(time(NULL));
    deliveries_global = deliveries;
    depots_global = depots;
    if (deliveries_global.size()+depots_global.size()>500) std::cout << "Out of size" << std::endl;
    turn_penalty_global = turn_penalty;
    std::cout<<"delivery: " << deliveries_global.size() << " depot: "<< depots_global.size() << std::endl;

    TCP tcp;
    tcp.Init(); // Initialize
    // std::cout<<"done initialize"<<std::endl;
    tcp.Search(); // Do the WHOLE search

    // After all iterations, the path of best_ant is the best path
    best_route_found.clear();
    std::vector<unsigned> empty_vector;
    empty_vector.resize(0);
    std::vector<IntersectionIndex> best_path_intersection_form = tcp.get_best_ant().get_path();

    // loop through all the path to get the route in the form of street segments
    for (unsigned i = 0; i < 2*deliveries.size()+1; i++) {
        std::vector<unsigned> path_between_intersection = find_path_between_intersections(best_path_intersection_form[i], best_path_intersection_form[i+1], turn_penalty);
        best_route_found.insert(best_route_found.end(), path_between_intersection.begin(), path_between_intersection.end());
    }

    return best_route_found;
}
