#ifndef PRM_STAR_H_
#define PRM_STAR_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <queue>

#include <ros/ros.h>

#include "HelperClasses.h"

using std::vector, std::pair;

class PRMStar {
    public:
        PRMStar() {}

        PRMStar(BoundingBox &_bounds, float _bot_rad, int _num_points) :
            agent_rad(_bot_rad), bounds(_bounds), num_points(_num_points+2) 
        {
            assert(_num_points != 0 && "_num_points must be > 0");

            for (int i = 0; i < num_points; i++) {
                nodes.push_back(Node(i, Vec2(), num_points));
            }

            iter_start = 1;
            iter_end = num_points-1;

            parents = vector<int>(num_points);

            start_idx = 0;
            goal_idx = num_points-1;

            cmp_dist = comp_dists{};
            cmp_node = comp_nodes{};

            last_obstacle_checked = 0;
        }

        ~PRMStar() {}

        void generateNodes() {
            if (num_points == 0) return;

            float d = 2.0f; // 2 dimensional space
            Vec2 diffs = bounds.max - bounds.min;
            float space_area = diffs.x*diffs.y; // area of total space
            for (const auto& obst : obstacles) {
                space_area -= obst.radiusSq * M_PI;
            }
            float lebesgue = M_PI; // volume of unit ball in 2d
            gamma_star_prm = 
                std::pow(2 * (1 + 1 / d), 1 / d) *
                std::pow(space_area / lebesgue, 1 / d);

            gamma_prm = gamma_star_prm*1.5;
            r = gamma_prm * std::pow(std::log(num_points) / num_points, 1 / d);
            r_sq = r * r;

            k_prm = 2*EULER;
            k = std::ceil(k_prm * std::log(num_points));

            last_obstacle_checked = obstacles.size();

            for (int i = iter_start; i < iter_end; i++) {
                Node &n = nodes[i];

                bool in_obstacle = false;
                do {    
                    in_obstacle = false;
                    n.coords.x = randRange(bounds.min.x, bounds.max.x);
                    n.coords.y = randRange(bounds.min.y, bounds.max.y);

                    int len = obstacles.size();
                    for (int j = 0; j < len; j++) {
                        Circle &c = obstacles[j];
                        Vec2 n_to_c = n.coords - c.center;
                        float dist = n_to_c.lenSq();

                        if (dist <= (c.radiusSq+(agent_rad*agent_rad))) 
                            in_obstacle = true;
                    }
                } while (in_obstacle);
            }

            updateGraphs();
        }

        void updateGraphs() {
            for (int i = iter_start; i < iter_end; i++) 
                connectPoints(i, i+1, num_points-1);
        }

        void updateStartPos(float x, float y) {
            nodes[start_idx].coords.x = x;
            nodes[start_idx].coords.y = y;
        }

        void connectStart() {
            connectPoints(start_idx, 0, num_points);
        }

        void updateGoalPos(float x, float y) {
            nodes[goal_idx].coords.x = x;
            nodes[goal_idx].coords.y = y;
        }

        void connectGoal() {
            connectPoints(goal_idx, 0, num_points);
        }

        void connectStartAndGoal() {
            connectPoints(goal_idx, 0, num_points);
            connectPoints(start_idx, 0, num_points-1);
        }

        void updateNodeConnections() {
            int len = nodes.size();
            for (int i = 0; i < len; i++) {
                Node &node = nodes[i];
                for (int j = i+1; j < len; j++) {
                    if (!node.graph[j]) continue;
                    Node &end = nodes[j];
                    Vec2 start_to_end = end.coords - node.coords;
                    float dist = start_to_end.len();

                    start_to_end.x /= dist;
                    start_to_end.y /= dist;

                    bool available = !intersectsAnyCircle(
                        obstacles, node.coords, dist, start_to_end
                    );

                    if (!available && node.graph[end.idx]) {
                        int path_len = path_indices.size();
                        for (int k = 0; k < path_len-1; k++) {
                            bool connection = 
                                (path_indices[k] == node.idx && 
                                 path_indices[k+1] == end.idx) ||
                                (path_indices[k] == end.idx &&
                                 path_indices[k+1] == node.idx);
                            if (connection) {
                                regenerate_path = true;
                                break;
                            }
                        }
                    }

                    node.graph[end.idx] = available;
                    end.graph[node.idx] = available;
                }
            }

            last_obstacle_checked = obstacles.size();
        }

        void updatePath() {
            path.clear();
            path_indices.clear();
            fringe.clear();

            for (int i = 0; i < num_points; i++) {
                bool is_start = i == start_idx;
                parents[i] = -1;
                nodes[i].g = is_start ? 0.0 : INFINITY;
                nodes[i].cost = is_start ? 
                    heuristic(start_idx, goal_idx) : 
                    INFINITY;
            }
            
            fringe.push_back(&nodes[start_idx]);

            while (!fringe.empty()) {
                std::make_heap(fringe.begin(), fringe.end(), cmp_node);
                std::pop_heap(fringe.begin(), fringe.end(), cmp_node);
                Node *current_node = fringe.back();
                fringe.pop_back();

                // break if we have found the goal
                if (current_node->idx == goal_idx) 
                    break;

                for (int i = 0; i < num_points; i++) {
                    if (!current_node->graph[i]) continue;
                    Node *neighbor_node = &nodes[i];

                    Vec2 curr_to_neighbor = 
                        current_node->coords - neighbor_node->coords;
                    float g = current_node->g + curr_to_neighbor.lenSq();

                    if (g < neighbor_node->g) {
                        parents[neighbor_node->idx] = current_node->idx;
                        neighbor_node->g = g;
                        neighbor_node->cost = 
                            g + heuristic(neighbor_node->idx, goal_idx);

                        bool in_fringe = false;
                        for (const auto &n : fringe) {
                            if (n->idx == neighbor_node->idx) {
                                in_fringe = true;
                                break;
                            }
                        }

                        if (!in_fringe) 
                            fringe.push_back(neighbor_node);
                    }
                }
            }  

            regenerate_path = false;

            if (fringe.size() == 0) {
                return;
            }

            int prev_node = parents[goal_idx];
            path.insert(path.begin(), nodes[goal_idx].coords);
            path_indices.insert(path_indices.begin(), goal_idx);
            // don't put agent node into the path
            while (prev_node > 0) {
                path.insert(path.begin(), nodes[prev_node].coords);
                path_indices.insert(path_indices.begin(), prev_node);
                prev_node = parents[prev_node];
            }
        }

        float heuristic(int id1, int id2) {
            Vec2 p = nodes[id1].coords - nodes[id2].coords;
            return p.lenSq();
        }

        void connectPoints(int index, int start, int end) {
            Node &node = nodes[index];
            node.k_nearest.clear();
            for (int j = start; j < end; j++) {
                if (j == index) continue;
                Node &end = nodes[j];
                Vec2 start_to_end = end.coords - node.coords;
                float dist_sq = start_to_end.lenSq();

                // if further than r or not in k nearest
                bool not_k_nearest = (
                    node.k_nearest.size() == k && dist_sq > node.max_dist_sq
                );
                if (not_k_nearest || dist_sq > r_sq) 
                    continue;

                float dist = std::sqrt(dist_sq);
                start_to_end.x /= dist;
                start_to_end.y /= dist;

                // line: y = mx + b
                // circ: (x - h)^2 + (y - k)^2 = r^2
                // find determinant and solve for roots if >= 0
                bool available = !intersectsAnyCircle(
                    obstacles, node.coords, dist, start_to_end
                );

                // this is nlogn i think, not great, could be sped up with KDTree?
                if (available) {
                    if (node.k_nearest.size() < k) {
                        node.k_nearest.push_back(pair<int, float>(j, dist_sq));
                        node.max_dist_sq = std::max(dist_sq, node.max_dist_sq);
                    } else {
                        // make heap to get the highest to the front
                        std::make_heap(
                            node.k_nearest.begin(), node.k_nearest.end(), cmp_dist
                        );
                        // pop heap to get second highest to front and first to back
                        std::pop_heap(
                            node.k_nearest.begin(), node.k_nearest.end(), cmp_dist
                        );
                        // compare distance of current neighbor to previous highest
                        if (dist_sq < node.k_nearest[k-1].second) {
                            // if its closer than the furthest neighbor, update
                            node.k_nearest[k-1].second = dist_sq;
                            int other_index = node.k_nearest[0].first;
                            node.k_nearest[k-1].first = j;

                            // get rid of the other connection
                            node.graph[other_index] = false;
                            nodes[other_index].graph[node.idx] = false;

                            // set new max dist
                            node.max_dist_sq = std::max(
                                dist_sq, node.k_nearest[0].second
                            );
                        } else {
                            available = false;
                        }
                    }
                }
                
                node.graph[end.idx] = available;
                end.graph[node.idx] = available;
            }
        }

        BoundingBox bounds;
        float agent_rad, r, r_sq, gamma_prm, gamma_star_prm, k_prm;
        int num_points, start_idx, goal_idx, 
            iter_start, iter_end, k, last_obstacle_checked;

        bool regenerate_path = false;

        vector<Node> nodes;
        vector<Vec2> path;
        vector<int> path_indices;
        vector<int> parents;
        vector<Circle> obstacles;
        vector<Node*> fringe;

        comp_dists cmp_dist;
        comp_nodes cmp_node;
};

#endif 
