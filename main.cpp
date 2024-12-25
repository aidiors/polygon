#include "graph.hpp"
#include "utils.hpp"

const std::string GRAPH_FILE = "spb_graph.txt";
const double START_LONGITUDE = 30.372067;
const double START_LATITUDE = 59.928577;
const double DEST_LONGITUDE = 30.310001;
const double DEST_LATITUDE = 59.956423;

const std::string ALGO_BFS = "BFS";
const std::string ALGO_DFS = "DFS";
const std::string ALGO_DIJKSTRA = "Dijkstra";
const std::string ALGO_A_STAR = "A*";

std::vector<Node*> find_path_using_algorithm(Graph& graph, Node* start_node, Node* dest_node, const std::string& algorithm_name, const bool print) {
    const auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Node*> path;

    if (algorithm_name == ALGO_BFS) {
        path = graph.bfs(start_node, dest_node);
    }
    else if (algorithm_name == ALGO_DFS) {
        path = graph.dfs(start_node, dest_node);
    }
    else if (algorithm_name == ALGO_DIJKSTRA) {
        path = graph.dijkstra(start_node, dest_node);
    }
    else if (algorithm_name == ALGO_A_STAR) {
        path = graph.a_star(start_node, dest_node);
    }
    else {
        std::cerr << "Unknown algorithm: " << algorithm_name << "\n";
        return {};
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    print_path_info(algorithm_name, path, end_time - start_time, print);

    return path;
}

int main() {
    Graph graph;
    build_graph_from_file(GRAPH_FILE, graph);

    Node* start_node = graph.get_nearest_node(START_LATITUDE, START_LONGITUDE);
    Node* dest_node = graph.get_nearest_node(DEST_LATITUDE, DEST_LONGITUDE);

    find_path_using_algorithm(graph, start_node, dest_node, ALGO_BFS, false);
    find_path_using_algorithm(graph, start_node, dest_node, ALGO_DFS, false);
    find_path_using_algorithm(graph, start_node, dest_node, ALGO_DIJKSTRA, false);
    find_path_using_algorithm(graph, start_node, dest_node, ALGO_A_STAR, false);

    return 0;
}