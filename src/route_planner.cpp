#include "route_planner.h"
#include <algorithm>
#include <iostream>

using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// - Use the distance to the end_node for the h value.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node* neighbor: current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
            neighbor->h_value = CalculateHValue(neighbor);
            open_list.emplace_back(neighbor);
            neighbor->visited = true;
        }
    }
}

// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    SortOpenList();
    RouteModel::Node * current = open_list.back();
    open_list.pop_back();
    return current; 
}

/**
 * Sort the vector of RouteModel::Node in descending order.
 */
void RoutePlanner::SortOpenList() {
  sort(open_list.begin(), open_list.end(), Compare);
}

/**
 * Compare the F values of two nodes.
 */
bool RoutePlanner::Compare(RouteModel::Node* a, RouteModel::Node* b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2; 
}

// - This method takes the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.emplace_back(*current_node);
    RouteModel::Node * parent = current_node->parent;
    while (current_node != start_node) {
        distance += current_node->distance(*parent);
        current_node = parent;
        parent = current_node->parent;
        path_found.emplace_back(*current_node);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    std::reverse(path_found.begin(),path_found.end());
    return path_found;

}

// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list.emplace_back(start_node);
  	current_node = start_node;
    start_node->visited = true;

    AddNeighbors(current_node);

    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    };
}