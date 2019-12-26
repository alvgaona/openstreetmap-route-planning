#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return this->end_node->distance(*node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for(auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value = current_node->distance(*neighbor);
    open_list.emplace_back(neighbor);
    neighbor->visited = true;
  }
}

bool RoutePlanner::Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
  float f1 = a->g_value + a->h_value; // f1 = g1 + h1
  float f2 = b->g_value + b->h_value; // f2 = g2 + h2
  return f1 > f2;
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(), Compare);
  RouteModel::Node* node = open_list.back();
  open_list.pop_back();
  return node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != nullptr) {
      path_found.push_back(*current_node);
      if(current_node->parent) {
        distance += current_node->distance(*current_node->parent);
      }
      current_node = current_node->parent;
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

}