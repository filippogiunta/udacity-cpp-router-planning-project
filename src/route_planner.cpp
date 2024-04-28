#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    // NOTE: start_node and end_nodes are attributes of the RoutePlanner class

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);

    std::cout << "The start node is: " << start_node->x << ", " << start_node->y << std::endl;
    std::cout << "The end node is: " << end_node->x << ", " << end_node->y << std::endl;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);

}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate the current_node.neighbors vector with all neighbors by invoking the method FindNeighbors().
    // NOTE: We can use "->" because current_node is passed by reference (using a pointer)
    current_node->FindNeighbors();
    
    for (auto item : current_node->neighbors){
        item -> parent = current_node;
        item -> h_value = CalculateHValue(item);

        // The g value of the neighbor is the sum of the g value of the current node and the distance from the current node to the neighbor.
        item -> g_value = current_node->g_value + current_node->distance(*item);

        // Add the item to the open_list and set the node's visited attribute to true.
        open_list.push_back(item);
        item -> visited = true;
    }
}

bool RoutePlanner::CompareNodes(const RouteModel::Node *a, const RouteModel::Node *b) {
    // Given two nodes, a and b, this function returns true if the sum of the h value and g value of node a is grater than the sum of the h value and g value of node b.
    
    float sum_a = a->h_value + a->g_value;
    float sum_b = b->h_value + b->g_value;
    
    return sum_a > sum_b;
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // To sort the list according to the sum of the h value and g value, we can use the std::sort function 
    // and a function as the third argument to define the way how we would like to sort the list.
    // In this case we would like to sort the list in descending order according to the sum of the h value and g value.
    // We chose to order the list in descending order because we want to remove the node with the lowest sum from the list. We can do it easily using open_list.pop_back().
    // As third argument we can pass the function CompareNodes.
    std::sort(open_list.begin(), open_list.end(), CompareNodes);

    // Create a pointer to the node in the list with the lowest sum. We ordered the list in descending order, so the last element of the list is the one with the lowest sum.
    auto node_with_lowest_sum = open_list.back();

    // Remove that node from the open_list. If we do not do this operation the node will be expanded again in the next iteration,
    // leading to an infinite loop in the A* algorithm.
    open_list.pop_back();

    // Return the pointer.
    return node_with_lowest_sum ;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Iterate from the end node to the start node by following the parent nodes
    while (current_node != start_node) {
        // Add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*(current_node->parent));
        // Add the current node to the path_found vector
        path_found.push_back(*current_node);
        // Move to the parent node
        current_node = current_node->parent;
    }

    // Add the start node to the path_found vector
    path_found.push_back(*start_node);

    // Reverse the path to get the correct order (we started from end and went to start. We want now to reverse the path to get the start to end path)
    std::reverse(path_found.begin(), path_found.end());


    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.


void RoutePlanner::AStarSearch() {

    // To avoid undesired behavior when declaring a pointer is good practice to init it to nullptr.
    RouteModel::Node *current_node = nullptr;

    // We start from the start_node, therefore it can be marked as visited and added to the open_list.
    start_node->visited = true;
    open_list.push_back(start_node);
    
    std::cout << "Starting A* search from node (" << start_node->x << ", " << start_node->y << ").\n";

    // As long as there are nodes in the open_list, we will continue to search for the end node.
    while (open_list.size() > 0) {
        // Get the next node to expand from open_list. 
        current_node = NextNode();
        std::cout << "Expanding node (" << current_node->x << ", " << current_node->y << ").\n";

        // If the current node is the end node, we can stop the search and construct the final path.
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            std::cout << "End node reached. Constructing path.\n";
            return;
        } else {
            // If the current node is not the end node, we need to add all of the neighbors of the current node to the open_list.
            AddNeighbors(current_node);
            std::cout << "Added neighbors of node (" << current_node->x << ", " << current_node->y << ") to the open list.\n";

        }
    }
}
