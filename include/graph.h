#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <set>
#include <limits>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Edge structure representing a connection between two nodes
 */
struct Edge {
    size_t to;
    double cost;
    
    Edge(size_t to_, double cost_) : to(to_), cost(cost_) {}
};

/**
 * @brief Graph class for representing traversability routes between waypoints
 */
class Graph {
public:
    /**
     * @brief Constructor for Graph
     * 
     * @param num_nodes Number of nodes (waypoints) in the graph
     */
    explicit Graph(size_t num_nodes);
    
    /**
     * @brief Add an edge between two nodes (bidirectional)
     * 
     * @param from Source node index
     * @param to Destination node index
     * @param cost Cost/distance of the edge
     */
    void addEdge(size_t from, size_t to, double cost);
    
    /**
     * @brief Get all edges from a specific node
     * 
     * @param node Node index
     * @return Vector of edges from the node
     */
    const std::vector<Edge>& getEdges(size_t node) const;
    
    /**
     * @brief Get the number of nodes in the graph
     * 
     * @return Number of nodes
     */
    size_t getNumNodes() const;
    
    /**
     * @brief Check if there's a direct edge between two nodes
     * 
     * @param from Source node
     * @param to Destination node
     * @return true if edge exists, false otherwise
     */
    bool hasEdge(size_t from, size_t to) const;
    
    /**
     * @brief Get the cost of the edge between two nodes
     * 
     * @param from Source node
     * @param to Destination node
     * @return Cost of the edge, or infinity if no edge exists
     */
    double getEdgeCost(size_t from, size_t to) const;
    
    /**
     * @brief Find shortest path between two nodes using Dijkstra's algorithm
     * 
     * @param start Start node index
     * @param end End node index
     * @return Vector of node indices representing the shortest path
     */
    std::vector<size_t> findShortestPath(size_t start, size_t end) const;
    
    /**
     * @brief Calculate total cost of a given path
     * 
     * @param path Vector of node indices representing the path
     * @return Total cost of the path
     */
    double calculatePathCost(const std::vector<size_t>& path) const;
    
    /**
     * @brief Get all possible paths between two nodes (up to a maximum depth)
     * 
     * @param start Start node index
     * @param end End node index
     * @param max_depth Maximum search depth to prevent infinite loops
     * @return Vector of paths, each path is a vector of node indices
     */
    std::vector<std::vector<size_t>> getAllPaths(size_t start, size_t end, size_t max_depth = 10) const;
    
    /**
     * @brief Print graph statistics and connectivity information
     * 
     * @param logger ROS logger for output
     */
    void printGraphInfo(const rclcpp::Logger& logger) const;
    
    /**
     * @brief Get connectivity matrix (for visualisation or analysis)
     * 
     * @return 2D vector representing the adjacency matrix with costs
     */
    std::vector<std::vector<double>> getAdjacencyMatrix() const;

private:
    size_t num_nodes_;
    std::vector<std::vector<Edge>> adjacency_list_;
    
    /**
     * @brief Recursive helper function for finding all paths
     */
    void findAllPathsRecursive(size_t current, size_t end, std::vector<size_t>& current_path, 
                              std::set<size_t>& visited, std::vector<std::vector<size_t>>& all_paths, 
                              size_t max_depth) const;
};

#endif // GRAPH_H