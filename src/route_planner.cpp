#include "route_planner.h"
#include <algorithm>

// RoutePlanner takes a reference to the model of the road and the start and end points of the path to be found
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // DONE 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node   = &m_Model.FindClosestNode(end_x, end_y);

    /* Test only - delete */
    // std::cout << start_node << " " << end_node << "\n";
    // std::cout << &m_Model.FindClosestNode(start_x, start_y) << "\n";
    // std::cout << &m_Model.FindClosestNode(end_x, end_y) << "\n";

}
  

// DONE 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calcualtes distance between given node and end_node
    float h_value = node -> distance(*end_node);

    return h_value;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node -> FindNeighbors();

    for (auto neighbor : current_node -> neighbors) {
        if (!(neighbor -> visited)) {
            neighbor -> parent = current_node;
            neighbor -> g_value = (current_node -> g_value) + (current_node -> distance(*neighbor));
            neighbor -> h_value = CalculateHValue(neighbor);
        
            open_list.push_back(neighbor);

            neighbor -> visited = true;
        }
    }
    
}

bool Compare (RouteModel::Node *lval, RouteModel::Node *rval) {
    float f1 = (lval -> g_value) + (lval -> h_value);
    float f2 = (rval -> g_value) + (rval -> h_value);

    if (f1 < f2) {
        return false;
    }
    else
    {
        return true;
    }
    
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);

    // Get a pointer to the last element of the open_list
    auto *closest_node = open_list.back();
    open_list.pop_back();

    return closest_node;
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

    int count = 0;

    // Copy the current node
    RouteModel::Node *copy_node = current_node;

    path_found.push_back(*copy_node);
    // TODO: Implement your solution here.
    // Iterate through nodes as long as they have parent nodes
    while (copy_node != start_node) {
        // Increase the distance of the path
        distance += copy_node -> distance(*(copy_node -> parent));

        // Add the parent node to the path
        path_found.push_back(*(copy_node -> parent));

        // Take the next node
        copy_node = copy_node -> parent;
    }

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
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;

    current_node -> visited = true;

    while (true) {
        // Add the neighbors of the current node to the open list
        AddNeighbors(current_node);

        // Get the node with the lowest f-value
        current_node = NextNode();

        // std::cout << current_node -> x << "\n";
        // std::cout << end_node -> x << "\n";
        
        // std::cout << current_node -> y << "\n";
        // std::cout << end_node -> y << "\n";



        // In case we do not find any solution, exit
        if ((current_node -> x == end_node -> x) && 
            (current_node -> y == end_node -> y)) {

            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
    }

    

}