#include "alpha_model.h"
#include <chrono>
#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <ratio>
#include <unordered_map>
#include <utility>
#include <vector>
#include <set>

/*
void writePointsToCSV(const std::vector<Point> &points,
                      const std::string &filename) {
  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  for (const Point &point : points) {
    file << point.x << "," << point.y << std::endl;
  }

  file.close();
}

void writeTriangles(const std::vector<std::size_t> &triangles,
                    const std::string &filename) {
  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  for (int i = 0; i < triangles.size(); i += 3) {
    file << triangles[i] << "," << triangles[i + 1] << "," << triangles[i + 2]
         << std::endl;
  }

  file.close();
}

struct NumberOfTrips {
  int city_a;
  int city_b;
  double number_of_trips;
};

void write_number_of_trips(const std::vector<NumberOfTrips> &p_not,
                    const std::string &filename) {
  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  for (int i = 0; i < p_not.size(); i++) {
    file << p_not[i].city_a << "," << p_not[i].city_b << "," << p_not[i].number_of_trips << std::endl;
  }

  file.close();
}

void write_vertices_to_csv(const std::vector<RoadGraph::Vertex> &points,
                           const std::string &filename) {
  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  for (const Vertex &point : points) {
    file << point.x << "," << point.y << "," << point.mass << std::endl;
  }

  file.close();
}

void write_road_network(const std::vector<std::pair<int, int>> &edges,
                    const std::string &filename) {
  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  for (int i = 0; i < edges.size(); i++) {
    file << edges[i].first << "," << edges[i].second << std::endl;
  }

  file.close();
}


void write_tris(const std::vector<std::pair<int, int>> &edges,
                    const std::string &filename) {
  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  for (int i = 0; i < edges.size(); i++) {
    file << edges[i].first << "," << edges[i].second << std::endl;
  }

  file.close();
}
*/

void write_vertices(const std::vector<RoadGraph::Vertex> &p_vertices, const std::string &p_filename) {
  std::ofstream file(p_filename);
  if (!file) {
    std::cerr << "Error opening file: " << p_filename << std::endl;
    return;
  }

  for (int i = 0; i < p_vertices.size(); i++) {
    file << p_vertices[i].point.x << "," << p_vertices[i].point.y << "," << p_vertices[i].mass << std::endl;
  }

  file.close();
}

void write_edges(const std::vector<RoadGraph::Edge> &p_edges, const std::string &p_filename) {
  std::ofstream file(p_filename);
  if (!file) {
    std::cerr << "Error opening file: " << p_filename << std::endl;
    return;
  }

  for (int i = 0; i < p_edges.size(); i++) {
    file << p_edges[i].v1 << "," << p_edges[i].v2 << std::endl;
  }

  file.close();
}

int main() {
  std::vector<RoadGraph::Vertex> cities;
  cities.push_back({.point = RoadGraph::Point {.x= 0.5, .y = 0.5}, .mass = 0.26});
  cities.push_back({.point = RoadGraph::Point {.x= 0.86, .y = 0.85}, .mass = 0.50});
  cities.push_back({.point = RoadGraph::Point {.x= 0.16, .y = 0.9}, .mass = 0.45});
  cities.push_back({.point = RoadGraph::Point {.x= 0.1, .y = 0.25}, .mass = 0.7});
  cities.push_back({.point = RoadGraph::Point {.x= 0.6, .y = 0.7}, .mass = 0.4});
  cities.push_back({.point = RoadGraph::Point {.x= 0.23, .y = 0.2}, .mass = 0.1});
  cities.push_back({.point = RoadGraph::Point {.x= 0.15, .y = 0.13}, .mass = 0.5});
  cities.push_back({.point = RoadGraph::Point {.x= 0.88, .y = 0.3}, .mass = 0.25});
  cities.push_back({.point = RoadGraph::Point {.x= 0.1, .y = 0.5}, .mass = 0.35});
  cities.push_back({.point = RoadGraph::Point {.x= 0.9, .y = 0.43}, .mass = 5.0});
  AlphaModelRoadGenerator road_generator = AlphaModelRoadGenerator({}, cities);
  AlphaModelRoadGenerator::RoadGenerationOutput output;
  road_generator.generate_roads({}, output);
  write_vertices(output.vertices, "vertices.csv");
  write_edges(output.road_edges, "edges.csv");
}