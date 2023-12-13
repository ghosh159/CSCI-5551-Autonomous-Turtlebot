#ifndef RRG_H_
#define RRG_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

#include "HelperClasses.h"

using std::vector;

class RRG {
    public:
    RRG() {}

    RRG(BoundingBox &_bounds, float _bot_rad, int num_iters) : 
        iters(num_iters), bounds(_bounds)
    {
        assert(num_iters != 0 && "_num_points must be > 0");
        eta = 5.0f;

        // cmp_dist = comp_dists{};
        // cmp_node = comp_nodes{};

        num_points = iters+2;
        start_idx = 0;
        goal_idx = iters+1;

        parents = vector<int>(num_points);

        last_obstacle_checked = 0;
    }

    ~RRG() {} 

    void generateNodes(Vec2 &pos, Vec2 &goal) {
        if (isInAnyCircle(pos)) return;
        nodes.clear();
        last_obstacle_checked = obstacles.size();

        float d = 2.0f; // 2 dimensional space
        Vec2 diffs = bounds.max - bounds.min;
        float space_area = diffs.x*diffs.y; // area of total space
        for (const auto& obst : obstacles) {
            space_area -= obst.radiusSq * M_PI;
        }
        float lebesgue = M_PI; // volume of unit ball in 2d
        float gamma_star_rrg = 
            std::pow(2 * (1 + 1 / d), 1 / d) *
            std::pow(space_area / lebesgue, 1 / d);

        gamma_rrg = gamma_star_rrg*1.5;
        k_rrg = 2*EULER;

        // insert position as first node, don't assume number of other nodes
        nodes.push_back(Node(0, pos, iters+2));
        
        for (int i = 0; i < iters; i++) {
            Vec2 rand_point;
            bool in_obstacle = false;
            // generate 
            int nearest;
            do {    
                in_obstacle = false;
                rand_point.x = randRange(bounds.min.x, bounds.max.x);
                rand_point.y = randRange(bounds.min.y, bounds.max.y);

                nearest = getNearestNode(rand_point);
                
                // if rand_point wasn't blocked by obstacles
                auto [dist, ray] = steerPoint(
                    nodes[nearest].coords, rand_point, eta
                );
                in_obstacle = intersectsAnyCircle(
                    obstacles, rand_point, dist, ray
                );
            } while (in_obstacle);

            int idx = nodes.size();
            nodes.push_back(Node(idx, rand_point, iters));

            // nearest and current are now connected
            nodes[nearest].graph[idx] = true;
            nodes[idx].graph[nearest] = true;

            // we have found a new point
            float cardV = nodes.size();
            int k = std::ceil(k_rrg*std::log(cardV));
            float r = std::min(
                gamma_rrg * std::sqrt(std::log(cardV) / cardV), eta
            );

            connectPoints(idx, nearest, k, r);
        }

        int idx = nodes.size();
        nodes.push_back(Node(idx, goal, iters));
        connectPoints(idx, -1, num_points, INFINITY);
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
                    obstacles, node.coords, dist, 
                    start_to_end, last_obstacle_checked
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

    void connectPoints(int index, int exclude, int k, float r) {
        Node &node = nodes[index];
        node.k_nearest.clear();

        int len = nodes.size();
        for (int j = 0; j < len; j++) {
            if (j == index || j == exclude) continue;
            Node &end = nodes[j];
            Vec2 start_to_end = end.coords - node.coords;
            float dist = start_to_end.len();

            // if further than r or not in k nearest
            bool not_k_nearest = (
                node.k_nearest.size() == k && dist > node.max_dist
            );
            if (not_k_nearest || dist > r) 
                continue;

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
                    node.k_nearest.push_back(pair<int, float>(j, dist));
                    node.max_dist = std::max(dist, node.max_dist);
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
                    if (dist < node.k_nearest[k-1].second) {
                        // if its closer than the furthest neighbor, update
                        node.k_nearest[k-1].second = dist;
                        int other_index = node.k_nearest[0].first;
                        node.k_nearest[k-1].first = j;

                        // get rid of the other connection
                        node.graph[other_index] = false;
                        nodes[other_index].graph[node.idx] = false;

                        // set new max dist
                        node.max_dist_sq = std::max(
                            dist, node.k_nearest[0].second
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

    void updateStartPos(float x, float y) {
        nodes[0].coords.x = x;
        nodes[0].coords.y = y;
    }

    void updatePath() {
        path.clear();
        fringe.clear();
        assert(num_points == nodes.size());

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

    int getNearestNode(Vec2 &point) {
        int closest = 0;
        float min_dist = INFINITY;
        for (const auto &node : nodes) {
            float dist = (node.coords - point).lenSq();
            if (dist < min_dist) {
                min_dist = dist;
                closest = node.idx;
            }
        }

        return closest;
    }

    bool isInAnyCircle(Vec2 &node_pos) {
        int len = obstacles.size();
        for (int k = 0; k < len; k++) {
            float dist = (node_pos - obstacles[k].center).lenSq();
            if (dist <= obstacles[k].radiusSq) 
                return true;
        }

        return false;
    }

    int iters, num_points, start_idx, goal_idx, last_obstacle_checked;
    float eta, gamma_rrg, k_rrg;
    bool regenerate_path = false;
    BoundingBox bounds;

    vector<Node> nodes;
    vector<Circle> obstacles;
    vector<Vec2> path;
    vector<int> path_indices, parents;
    vector<Node*> fringe;

    comp_dists cmp_dist;
    comp_nodes cmp_node;
};

#endif
