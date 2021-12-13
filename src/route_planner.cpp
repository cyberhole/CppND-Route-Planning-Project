#include "route_planner.h"
#include <algorithm>
#include <iostream>
#include <vector>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x = (start_x <= 100 && start_x >= 0) ? start_x*0.01 : 0;
    start_y = (start_y <= 100 && start_y >= 0) ? start_y*0.01 : 0;
    end_x = (end_x <= 100 && end_x >= 0) ? end_x*0.01 : 0;
    end_y = (end_y <= 100 && end_y >= 0) ? end_y*0.01 : 0;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x,start_y);
    this->end_node = &m_Model.FindClosestNode(end_x,end_y);
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

    current_node->FindNeighbors();

    std::vector<RouteModel::Node*> neighbors = current_node->neighbors;

    float h;
    for (int i=0; i<neighbors.size(); i++) {
        h = CalculateHValue(neighbors[i]);
        neighbors[i]->h_value = h;
        neighbors[i]->g_value = current_node->g_value + neighbors[i]->distance(*current_node);
        neighbors[i]->parent = current_node;
        neighbors[i]->visited = true;
        open_list.push_back(neighbors[i]);                      
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(RouteModel::Node *a, RouteModel::Node *b) {
  float h1 = a->h_value;
  float h2 = b->h_value;
  float g1 = a->g_value;
  float g2 = b->g_value; 

  float total1 = h1 + g1;
  float total2 = h2 + g2;
  return total1 > total2; 
}


RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *temp;

    float sum1;
    float sum2;

    std::sort(open_list.begin(), open_list.end(), Compare);

    RouteModel::Node *closestNode;
    closestNode = open_list.back();
    open_list.pop_back();

    return closestNode;
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
    this->distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    RouteModel::Node *nextNode; //node to save the parent node 
    
    // adds the end node to the vector of the path
    path_found.push_back(*current_node);

    while(current_node->parent != nullptr) {
        nextNode = current_node->parent; // get the parent node
        distance += current_node->distance(*nextNode); // calculate the distance between then
        path_found.push_back(*nextNode);  //pushes the parent node into the vector of nodes for the path 
        current_node = nextNode; //changes to next node
    }
    std::reverse(path_found.begin(), path_found.end()); // reverse vector
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
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);
        
    RouteModel::Node *current_node = nullptr;
    this->AddNeighbors(this->start_node);
    current_node = this->NextNode();

    //while (current_node->x!=end_node->x && current_node->y!=end_node->y){
    while (current_node != this->end_node) {
        this->AddNeighbors(current_node);
        current_node = this->NextNode();
    }

    //current node must be final node
    this->m_Model.path = this->ConstructFinalPath(current_node);
}