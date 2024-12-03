#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>
#include <set>
#include <stdexcept>
#include <algorithm>  // For std::remove_if

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  std::unordered_map<VertexT, std::vector<std::pair<VertexT, WeightT>>> adjacency_list;
  size_t edge_count = 0;  // Track the number of edges

 public:
  /// Default constructor
  graph() = default;

  /// @brief Add a vertex to the graph. O(1).
  bool addVertex(const VertexT& v) {
    if (adjacency_list.find(v) == adjacency_list.end()) {
      adjacency_list[v] = {};
      return true;  // Vertex added
    }
    return false;  // Vertex already exists
  }

  /// @brief Add a directed edge from `from` to `to` with weight `weight`.
  /// If the edge already exists, overwrite the weight.
  bool addEdge(const VertexT& from, const VertexT& to, const WeightT& weight) {
    // Ensure both vertices exist
    if (!containsVertex(from) || !containsVertex(to)) {
      return false;  // Edge cannot be added if either vertex is missing
    }

    // Check if the edge already exists and update its weight
    for (auto& edge : adjacency_list[from]) {
      if (edge.first == to) {
        edge.second = weight;  // Update weight if edge exists
        return true;  // Edge updated
      }
    }

    // Add a new edge
    adjacency_list[from].emplace_back(to, weight);
    edge_count++;
    return true;
  }

  /// @brief Remove a directed edge from `from` to `to`.
  void removeEdge(const VertexT& from, const VertexT& to) {
    if (adjacency_list.find(from) != adjacency_list.end()) {
      auto& edges = adjacency_list[from];
      auto initial_size = edges.size();
      edges.erase(std::remove_if(edges.begin(), edges.end(),
                                 [&to](const std::pair<VertexT, WeightT>& edge) {
                                   return edge.first == to;
                                 }),
                  edges.end());
      if (edges.size() < initial_size) {
        edge_count--;  // Only decrement if an edge was removed
      }
    }
  }

  /// @brief Get all neighbors of a vertex.
  std::vector<std::pair<VertexT, WeightT>> getNeighbors(const VertexT& v) const {
    auto it = adjacency_list.find(v);
    if (it == adjacency_list.end()) {
      return {};  // Return empty if vertex does not exist
    }
    return it->second;
  }

  /// @brief Get all neighbors of a vertex as a set of vertices.
  std::set<VertexT> neighbors(const VertexT& v) const {
    std::set<VertexT> neighborSet;
    auto it = adjacency_list.find(v);
    if (it == adjacency_list.end()) {
      return neighborSet;  // Return empty set if vertex does not exist
    }
    for (const auto& neighbor : it->second) {
      neighborSet.insert(neighbor.first);  // Only insert the vertex IDs
    }
    return neighborSet;
  }

  /// @brief Check if the graph contains a vertex.
  bool containsVertex(const VertexT& v) const {
    return adjacency_list.find(v) != adjacency_list.end();
  }

  /// @brief Get the number of vertices in the graph.
  size_t numVertices() const {
    return adjacency_list.size();
  }

  /// @brief Get the number of edges in the graph.
  size_t numEdges() const {
    return edge_count;
  }

  /// @brief Get all vertices in the graph.
  std::vector<VertexT> getVertices() const {
    std::vector<VertexT> vertices;
    for (const auto& [vertex, edges] : adjacency_list) {
      vertices.push_back(vertex);
    }
    return vertices;
  }

  /// @brief Get the weight of an edge between `from` and `to`.
  /// @param from Starting vertex
  /// @param to Ending vertex
  /// @param weight The weight of the edge, if it exists
  /// @return `true` if the edge exists, otherwise `false`
  bool getWeight(const VertexT& from, const VertexT& to, WeightT& weight) const {
    auto it = adjacency_list.find(from);
    if (it != adjacency_list.end()) {
      for (const auto& edge : it->second) {
        if (edge.first == to) {
          weight = edge.second;
          return true;  // Edge found
        }
      }
    }
    return false;  // No edge found
  }

  /// @brief Print the graph (for debugging).
  void print(std::ostream& out = std::cout) const {
    for (const auto& [vertex, edges] : adjacency_list) {
      out << vertex << ": ";
      for (const auto& edge : edges) {
        out << "{" << edge.first << ", " << edge.second << "} ";
      }
      out << "\n";
    }
  }
};
