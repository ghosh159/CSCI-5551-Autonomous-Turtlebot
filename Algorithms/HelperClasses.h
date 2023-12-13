#ifndef HELPER_H_
#define HELPER_H_

#include <iostream>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

using namespace std::chrono;
using std::vector, std::pair;

// helpful constants
float MY_2PI = 2*M_PI;
float ONE_OVER_RAND_MAX = 1.0f / RAND_MAX;
double EULER = std::exp(1.0);
float TO_DEGS = 180.0 / M_PI;
float TO_NODE_THRESH = 0.05;
float LINEAR_SPEED = 1;
float ANGULAR_SPEED = 1;

namespace Colors {
    cv::Scalar RED(255, 0, 0);
    cv::Scalar GREEN(0, 255, 0);
    cv::Scalar BLUE(0, 0, 255);
    cv::Scalar YELLOW(0, 255, 255);
    cv::Scalar WHITE(255, 255, 255);
}

typedef struct Vec2 {
    Vec2() : x(0.0), y(0.0) {}
    Vec2(float _x, float _y) : x(_x), y(_y) {}

    Vec2 operator-(const Vec2 &op) const {
        return {this->x - op.x, this->y - op.y};
    }

    void operator*=(const float t) {
        x *= t; y *= t;
    }

    void operator+=(const Vec2 &ovec) {
        x += ovec.x; y += ovec.y;
    }

    float lenSq() {
        return this->x*this->x + this->y*this->y;
    }

    float len() {
        return std::sqrt(this->lenSq());
    }

    static float dot(const Vec2 &v1, const Vec2 &v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    float normalize() {
        float len = this->len();
        x /= len;
        y /= len;

        return len;
    }

    float x, y;
} Vec2;

typedef struct BoundingBox {
    BoundingBox() {}
    BoundingBox(Vec2 mind, Vec2 maxd) {
        this->min = mind;
        this->max = maxd;

        this->tl = Vec2(mind.x, maxd.y);
        this->br = Vec2(maxd.x, mind.y);
    }
    Vec2 min, max, tl, br;
} BoundingBox;

typedef struct Circle {
    Circle(Vec2 c, float r) : center(c), radius(r), radiusSq(r*r) {}
    Vec2 center;
    float radius, radiusSq;
} Circle;

class Node {
    public:
    Node(int i, Vec2 p, int op) : 
        idx(i), coords(p), graph(op, false), k_nearest() {
        this->g = this->cost = 0.0;
        this->max_dist_sq = this->max_dist = INFINITY;
    };
    ~Node() {}

    int idx;
    Vec2 coords;
    vector<bool> graph;
    vector<pair<int, float>> k_nearest;

    float g, cost, max_dist_sq, max_dist;
};

struct comp_nodes {
    // should min-heapfiy
    bool operator()(Node *a, Node *b) {
        return a->cost > b->cost;
    }
};

struct comp_dists {
    // should max-heapify
    bool operator()(pair<int, float> &a, pair<int, float> &b) {
        return a.second < b.second;
    }
};

float randRange(float min, float max) {
    return (float)std::rand() * ONE_OVER_RAND_MAX * (max - min) + min;
}

// returns distance between the new point and x
std::tuple<float, Vec2> steerPoint(const Vec2 &x, Vec2 &y, float eta) {
    // find the z that minimizes ||z - y|| while keeping ||z - x|| <= n
    Vec2 y2x = x - y;
    // avoid unnecessary sqrt
    float dist = y2x.len();
    y2x.x /= dist; y2x.y /= dist;
    // z is exactly y because distance is less than eta
    if (dist <= eta) return {dist, y2x};

    // move z towards x until z - x dist == n
    // normalize
    

    // extend the vector to the proper point
    float diff = dist - eta;

    // get offset point
    y.x += y2x.x * diff;
    y.y += y2x.y * diff;

    return {eta, y2x};
}

float time_function(std::string msg, std::function<void()> func, bool verbose=false) {
    auto start = high_resolution_clock::now();
    func();
    auto end = high_resolution_clock::now();
    duration<float> dur = end - start;
    if (verbose) 
        printf("%s %fms\n", msg.c_str(), dur.count()*1000);
    return dur.count()*1000;
}

bool intersectsCircle(Circle &cir, Vec2 &node_pos, float dist, Vec2 &ray) {
    Vec2 cp = cir.center - node_pos;

    float c_dist_sq = cp.lenSq();

    float a = 1;
    // dot the ray with the line to circle center
    float b = -2 * Vec2::dot(ray, cp);
    float c = c_dist_sq - cir.radiusSq;
    float d = b*b - 4*a*c;

    if (d >= 0) {
        float one_over_2a = 1 / (2*a);
        float sqrt_d = std::sqrt(d);

        float t1 = (-b - sqrt_d) * one_over_2a;
        float t2 = (-b + sqrt_d) * one_over_2a;

        bool two_int = (t1 > 0 && t1 < dist);
        bool inside = (t1 < 0 && t2 > 0);
        
        return two_int || inside;
    }

    return false;
}

bool intersectsAnyCircle(
    vector<Circle> &obstacles, Vec2 &node_pos, 
    float dist, Vec2 &ray, int start=0
) {
    int len = obstacles.size();
    for (int k = start; k < len; k++) 
        if (intersectsCircle(obstacles[k], node_pos, dist, ray)) 
            return true;                    

    return false;
}

template <typename T>
// constrains x is the min and y is the max
void generateRandomObstacles(
    T& rm, int num, float min_rad=0.0, float max_rad=1.0, vector<Vec2> constrains=vector<Vec2>()
) {
    for (int i = 0; i < num; i++) {
        Vec2 pos;
        bool placed = false;
        do {
            float rx = randRange(rm.bounds.min.x, rm.bounds.max.x);
            float ry = randRange(rm.bounds.min.y, rm.bounds.max.y);
            bool can_place = true;
            for (auto &constrain : constrains) {
                if (rx < constrain.y && rx > constrain.x &&
                    ry < constrain.y && ry > constrain.x) can_place = false;
            }
            if (can_place) {
                pos.x = rx;
                pos.y = ry;
                placed = true;
            }
        } while (!placed);
        rm.obstacles.push_back(Circle(pos, randRange(min_rad, max_rad)));
    }
}

// drawing constants and functions
BoundingBox c_world_bounds;
float min = -8.0, max = 8.0;

float out_dim = 1000.0;
float out_meter_dim = 16.0;
float outer_padding = 0.0; // meters
float to_pixels = out_dim / (out_meter_dim + outer_padding*2);

float CIRCLE_SIZE = 0.01;
float AGENT_SIZE = 0.05;

float convert(float val) {
    float normalized = ((val - min) / (max - min));
    float out_sized = normalized * out_meter_dim;
    float padded = out_sized + outer_padding;
    float pixelized = padded * to_pixels;

    return pixelized;
}

void embed_circle(cv::Mat &mat, Vec2 &center, float radius, cv::Scalar color) {
    cv::circle(
        mat, cv::Point(convert(center.x), convert(center.y)), 
        radius * to_pixels, color, cv::FILLED
    );
}

void embed_line(cv::Mat &mat, Vec2 &p1, Vec2 &p2, cv::Scalar color, int thickness=1) {
    cv::line(
        mat, 
        cv::Point(convert(p1.x), convert(p1.y)),
        cv::Point(convert(p2.x), convert(p2.y)),
        color, thickness
    );
}

template <typename T>
void drawRoadmap(cv::Mat &img, Vec2 &agent_pos, T &rm) {
    embed_line(img, rm.bounds.min, rm.bounds.tl, Colors::WHITE);
    embed_line(img, rm.bounds.min, rm.bounds.br, Colors::WHITE);
    embed_line(img, rm.bounds.max, rm.bounds.tl, Colors::WHITE);
    embed_line(img, rm.bounds.max, rm.bounds.br, Colors::WHITE);

    for (auto &circle : rm.obstacles) {
        embed_circle(img, circle.center, circle.radius, Colors::RED);
    }
    
    if (rm.nodes.size() != 0) {
        for (int i = 1; i < rm.num_points; i++) {
            Node &n = rm.nodes[i];
            embed_circle(img, n.coords, CIRCLE_SIZE, Colors::GREEN);
            for (int j = i+1; j < rm.num_points; j++) {
                if (!n.graph[j]) continue;
                embed_line(img, n.coords, rm.nodes[j].coords, Colors::GREEN, 1);
                embed_circle(img, rm.nodes[j].coords, CIRCLE_SIZE, Colors::GREEN);
            }
        }
    }
    

    int len = rm.path.size()-1;
    if (len != 0) {
        for (int i = 0; i < len; i++) {
            if (i == 0) {
                embed_line(img, agent_pos, rm.path[0], Colors::RED, 2);
            }
            embed_circle(img, rm.path[i], CIRCLE_SIZE, Colors::BLUE);
            embed_line(img, rm.path[i], rm.path[i+1], Colors::BLUE, 2);
            if (i == len-1) {
                embed_circle(img, rm.path[i+1], CIRCLE_SIZE, Colors::BLUE);
                break;
            }
        }
    } else if (len == 0) {
        embed_circle(img, rm.path[0], CIRCLE_SIZE, Colors::BLUE);
        embed_line(img, agent_pos, rm.path[0], Colors::RED, 2);
    }

    embed_circle(img, agent_pos, AGENT_SIZE, Colors::YELLOW);
}

#endif
