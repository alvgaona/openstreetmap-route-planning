#include "route_planner.h"

#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

bool RoutePlanner::IsStartNode(const RouteModel::Node *node) {
  return start_node->x == node->x && start_node->y == node->y;
}

bool RoutePlanner::IsEndNode(const RouteModel::Node *node) {
  return end_node->x == node->x && end_node->y == node->y;
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return this->end_node->distance(*node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
    open_list.emplace_back(neighbor);
    neighbor->visited = true;
  }
}

bool RoutePlanner::CompareNodesByFValue(const RouteModel::Node *a,
                                        const RouteModel::Node *b) {
  float f1 = a->g_value + a->h_value;  // f1 = g1 + h1
  float f2 = b->g_value + b->h_value;  // f2 = g2 + h2
  return f1 > f2;
}

/* Sort the open_list according to the sum of the h value and g value.
 * Removes the node from the open_list with the lowest f value and returns
 * the pointer to that node.
 */
RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(), CompareNodesByFValue);
  RouteModel::Node *node = open_list.back();
  open_list.pop_back();
  return node;
}

/* This method takes the current (final) node as an argument and iteratively
 * follow the chain of parents of nodes until the starting node is found. For
 * each node in the chain, add the distance from the node to its parent to the
 * distance variable. The returned vector is in the correct order: the start
 * node is the first element of the vector, the end node is the last element.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (!IsStartNode(current_node)) {
    path_found.push_back(*current_node);
    distance += current_node->distance(*current_node->parent);
    current_node = current_node->parent;
  }
  path_found.push_back(*current_node);
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale();  // Multiply the distance by the scale of
                                      // the map to get meters.
  return path_found;
}

/* The A* Search algorithm.
 * Uses the AddNeighbors method to add all of the neighbors of the current node
 * to the open_list. Starts by adding the start_node into the open_list.
 * Advances node by node by calling NextNode() which gets the lowest f-valued
 * neighbor. When the search has reached the end_node, constructs final path
 * that was found.
 */
void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = start_node;
  current_node->visited = true;
  open_list.push_back(current_node);

  while (!open_list.empty()) {
    current_node = NextNode();
    if (IsEndNode(current_node)) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);
  }
}
