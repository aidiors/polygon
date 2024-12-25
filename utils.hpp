#pragma once

#include <chrono>
#include "graph.hpp"

bool parse_node_coordinates(const std::string& str, double& longitude, double& latitude) {
    return sscanf_s(str.c_str(), "%lf,%lf", &longitude, &latitude) == 2;
}

bool parse_edges(const std::string& str, double& longitude, double& latitude, double& weight) {
    return sscanf_s(str.c_str(), "%lf,%lf,%lf", &longitude, &latitude, &weight) == 3;
}

void build_graph_from_file(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);

        std::string node_info;
        if (!std::getline(ss, node_info, ':')) {
            throw std::runtime_error("Invalid line format: " + line);
        }
        double start_node_longitude;
        double start_node_latitude;
        if (parse_node_coordinates(node_info, start_node_longitude, start_node_latitude)) {
            graph.add_node(start_node_longitude, start_node_latitude);
        }
        else
            throw std::runtime_error("Invalid node format in line: " + line);

        std::string neighbors_info;
        while (std::getline(ss, neighbors_info, ';')) {
            double dest_node_longitude;
            double dest_node_latitude;
            double edge_weight;
            if (parse_edges(neighbors_info, dest_node_longitude, dest_node_latitude, edge_weight)) {
                graph.add_node(dest_node_longitude, dest_node_latitude);
                graph.connect_nodes(start_node_longitude, start_node_latitude, dest_node_longitude, dest_node_latitude, edge_weight, false);
            }
            else
                throw std::runtime_error("Invalid neighbor format in line: " + line);
        }
    }
}

void print_path(const std::vector<Node*>& path) {
    if (path.empty()) {
        std::cout << "No path found.\n";
        return;
    }
    for (Node* node : path) {
        std::cout << "[" << node->longitude << "," << node->latitude << "] -> ";
    }
    std::cout << "END\n";
}

void print_path_info(const std::string& algorithm_name, const std::vector<Node*>& path, const std::chrono::duration<double>& duration, const bool print) {
    std::cout << algorithm_name << ": \n";
    if (print)
        print_path(path);
    std::cout << "  " << algorithm_name << " Path Length: " << path.size() << "\n";
    std::cout << "  " << "Time " << algorithm_name << ": " << duration.count() << " seconds\n";
}
