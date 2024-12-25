#include "graph.hpp"

void Graph::add_node(const double longitude, const double latitude) {
    nodes.emplace(std::make_pair(latitude, longitude), std::make_unique<Node>(longitude, latitude));
}

void Graph::connect_nodes(double start_longitude, double start_latitude, double dest_longitude, double dest_latitude, double weight, const bool orientation) {
    auto start_node_it = nodes.find({ start_latitude, start_longitude });
    auto dest_node_it = nodes.find({ dest_latitude, dest_longitude });

    if (start_node_it == nodes.end() || dest_node_it == nodes.end()) {
        return;
    }

    Node* start_node = start_node_it->second.get();
    Node* dest_node = dest_node_it->second.get();
    start_node->add_edge(dest_node, weight);
    if (!orientation)
        dest_node->add_edge(start_node, weight);
}

std::vector<Node*> Graph::bfs(Node* start_node, Node* dest_node) {
    std::deque<Node*> queue{ start_node }; // Queue for BFS
    std::unordered_set<Node*> visited{ start_node }; // Set to track visited nodes to avoid processing the same node multiple times

    std::unordered_map<Node*, Node*> parent; // Map to store the parent of each node for path reconstruction
    parent[start_node] = nullptr;

    // Perform BFS until there are no more nodes in the queue
    while (!queue.empty()) {
        Node* current_node = queue.front();
        queue.pop_front();

        // Check if we have reached the destination node
        if (current_node == dest_node) {
            return construct_path(current_node, parent);
        }

        // Explore all adjacent nodes (edges) of the current node
        for (const auto& edge : current_node->edges) {
            if (visited.insert(edge.first).second) {
                parent[edge.first] = current_node;
                queue.push_back(edge.first);
            }
        }
    }

    return {};
}

std::vector<Node*> Graph::dfs(Node* start_node, Node* dest_node) {
    std::deque<Node*> stack{ start_node }; // Stack for DFS
    std::unordered_set<Node*> visited{ start_node }; // Set to track visited nodes to avoid processing the same node multiple times

    std::unordered_map<Node*, Node*> parent; // Map to store the parent of each node for path reconstruction
    parent[start_node] = nullptr;

    // Perform DFS until there are no more nodes in the stack
    while (!stack.empty()) {
        Node* current_node = stack.back();
        stack.pop_back();

        // Check if we have reached the destination node
        if (current_node == dest_node) {
            return construct_path(current_node, parent);
        }

        // Explore all adjacent nodes (edges) of the current node
        for (const auto& edge : current_node->edges) {
            if (visited.insert(edge.first).second) {
                parent[edge.first] = current_node;
                stack.push_back(edge.first);
            }
        }
    }
    return {};
}

std::vector<Node*> Graph::dijkstra(Node* start_node, Node* dest_node) {
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<>> priority_queue; // Priority queue for dijkstra

    std::unordered_map<Node*, double> path_cost;
    path_cost.reserve(nodes.size());
    for (const auto& node : nodes) {
        path_cost[node.second.get()] = INFINITY;
    }

    std::unordered_map<Node*, Node*> parent; // Map to store the parent of each node
    parent[start_node] = nullptr;

    path_cost[start_node] = 0;
    priority_queue.emplace(0, start_node);

    // Perform dijkstra until there are no more nodes in the priority_queue
    while (!priority_queue.empty()) {
        auto current = priority_queue.top();
        priority_queue.pop();


        if (current.weight <= path_cost[current.node]) {
            // Check if we have reached the destination node
            if (current.node == dest_node) {
                return construct_path(current.node, parent);
            };

            for (const auto& edge : current.node->edges) {
                double weights_sum = current.weight + edge.second;

                if (weights_sum < path_cost[edge.first]) {
                    path_cost[edge.first] = weights_sum;
                    priority_queue.emplace(weights_sum, edge.first);
                    parent[edge.first] = current.node;
                }
            }
        }
    }

    return {};
}

std::vector<Node*> Graph::a_star(Node* start_node, Node* dest_node) {
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<>> priority_queue; // Priority queue for A*

    std::unordered_map<Node*, double> path_cost;
    std::unordered_map<Node*, double> total_estimated_cost;
    path_cost.reserve(nodes.size());
    total_estimated_cost.reserve(nodes.size());
    for (const auto& node_p : nodes) {
        Node* node = node_p.second.get();
        path_cost[node] = INFINITY;
        total_estimated_cost[node] = INFINITY;
    }

    std::unordered_map<Node*, Node*> parent;
    parent[start_node] = nullptr;

    path_cost[start_node] = 0;
    total_estimated_cost[start_node] = a_star_heuristic(start_node, dest_node);
    priority_queue.emplace(total_estimated_cost[start_node], start_node);

    // Perform A* until there are no more nodes in the priority_queue
    while (!priority_queue.empty()) {
        Node* current_node = priority_queue.top().node;
        priority_queue.pop();

        // Check if we have reached the destination node
        if (current_node == dest_node) {
            return construct_path(current_node, parent);
        };

        for (const auto& edge : current_node->edges) {
            double weights_sum = path_cost[current_node] + edge.second;

            if (weights_sum < path_cost[edge.first]) {
                parent[edge.first] = current_node;
                path_cost[edge.first] = weights_sum;
                total_estimated_cost[edge.first] = weights_sum + a_star_heuristic(edge.first, dest_node);
                priority_queue.emplace(total_estimated_cost[edge.first], edge.first);
            }
        }
    }

    return {};
}

Node* Graph::get_nearest_node(const double latitude, const double longitude) {
    double min_distance_squared = std::numeric_limits<double>::max();
    Node* closest_node = nullptr;

    for (const auto& it : nodes) {
        Node* node = it.second.get();
        double delta_lat = node->latitude - latitude;
        double delta_lon = node->longitude - longitude;
        double distance_squared = delta_lat * delta_lat + delta_lon * delta_lon;

        if (distance_squared == 0)
            return node;

        if (distance_squared < min_distance_squared) {
            closest_node = node;
            min_distance_squared = distance_squared;
        }
    }

    return closest_node;
}

double Graph::a_star_heuristic(const Node* node_1, const Node* node_2) {
    const double lat_diff = node_1->latitude - node_2->latitude;
    const double lon_diff = node_1->longitude - node_2->longitude;
    return lat_diff * lat_diff + lon_diff * lon_diff;
}

std::vector<Node*> Graph::construct_path(Node* current_node, const std::unordered_map<Node*, Node*>& parent) {
    std::vector<Node*> path;
    // Backtrack to construct the path
    for (Node* node = current_node; node != nullptr; node = parent.at(node)) {
        path.push_back(node);
    }
    // Reverse to get the path from start to destination
    std::reverse(path.begin(), path.end());

    return path;
}