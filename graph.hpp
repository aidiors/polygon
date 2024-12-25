#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <queue>

struct HashPair {
    template <typename T1, typename T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        size_t hash1 = std::hash<T1>{}(p.first);
        size_t hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

struct Node {
    double longitude;
    double latitude;
    std::vector<std::pair<Node*, double>> edges;

    Node(double longitude, double latitude)
        : longitude(longitude), latitude(latitude) {
    }

    void add_edge(Node* neighbor, double weight) {
        edges.emplace_back(neighbor, weight);
    }

};

struct QueueNode {
    double weight;
    Node* node;

    QueueNode(double weight, Node* node) : weight(weight), node(node) {}

    bool operator>(const QueueNode& other) const {
        return weight > other.weight;
    }
};

class Graph {
public:
    void add_node(const double longitude, const double latitude);
    void connect_nodes(double longitude1, double latitude1, double longitude2, double latitude2, double weight, const bool orientation);
    std::vector<Node*> bfs(Node* start_node, Node* dest_node);
    std::vector<Node*> dfs(Node* start_node, Node* dest_node);
    std::vector<Node*> dijkstra(Node* start_node, Node* dest_node);
    std::vector<Node*> a_star(Node* start_node, Node* dest_node);
    Node* get_nearest_node(const double latitude, const double longitude);
private:
    std::unordered_map<std::pair<double, double>, std::unique_ptr<Node>, HashPair> nodes;
    double a_star_heuristic(const Node* a, const Node* b);
    std::vector<Node*> construct_path(Node* current_node, const std::unordered_map<Node*, Node*>& parent);
};