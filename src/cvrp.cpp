#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cmath>
#include <omp.h>
#include <random>
#include <iomanip>

bool use_truncated_math = false;
bool use_integer_math = false;

using namespace std;
using namespace std::chrono;

struct Node {
    int id; double x, y; long long demand, ready, due, service;
};

struct VRPData {
    vector<Node> nodes;
    long long capacity;
    int max_vehicles;
    vector<vector<double>> matrix;
    vector<vector<int>> knn; // Stores indices of 40 nearest neighbors
    bool matrix_ready = false;
    
        inline double dist(int i, int j) const {
        if (matrix_ready) return matrix[i][j];
        double dx = nodes[i].x - nodes[j].x;
        double dy = nodes[i].y - nodes[j].y;
        double d = sqrt(dx * dx + dy * dy);
        d = round(d);
        return d;
    }
    
    void build_matrix() {
        int n = nodes.size();
        matrix.assign(n, vector<double>(n, 0.0));
        knn.assign(n, vector<int>());
        for (int i = 0; i < n; ++i) {
            vector<pair<double, int>> dists;
            for (int j = 0; j < n; ++j) {
                if (i != j) {
                                        double dx = nodes[i].x - nodes[j].x;
                    double dy = nodes[i].y - nodes[j].y;
                    double d = sqrt(dx * dx + dy * dy);
                    d = round(d);
                    matrix[i][j] = d;
                    
                    // Spatio-Temporal Distance
                    double time_warp = 0;
                    if (nodes[j].ready > nodes[i].due + nodes[i].service) {
                        time_warp = (double)(nodes[j].ready - (nodes[i].due + nodes[i].service));
                    }
                    double st_dist = d + time_warp * 2;
                    dists.push_back({st_dist, j});
                }
            }
            sort(dists.begin(), dists.end());
            int k = min(60, (int)dists.size()); // Increased to 60 for HGS Randomized exploration
            for (int j = 0; j < k; ++j) {
                knn[i].push_back(dists[j].second);
            }
        }
        matrix_ready = true;
    }
};

VRPData load_vrp(string filename) {
    VRPData data;
    ifstream file(filename);
    if (!file.is_open()) return data;
    string line; bool reading_coords = false; 
    data.capacity = 200; data.max_vehicles = 100;
    
    bool solomon_format = false;
    
    while (getline(file, line)) {
        if (line.find("EOF") != string::npos) break;
        
        // DIMACS Format
        if (line.find("CAPACITY") != string::npos && line.find(":") != string::npos) {
            size_t pos = line.find(":");
            data.capacity = stoll(line.substr(pos + 1));
        }
        if (line.find("NODE_COORD_SECTION") != string::npos) { reading_coords = true; continue; }
        
        // Solomon Format
        if (line.find("NUMBER     CAPACITY") != string::npos) {
            solomon_format = true;
            getline(file, line);
            stringstream ss(line);
            int num; ss >> num >> data.capacity;
            data.max_vehicles = num;
        }
        if (line.find("CUST NO.") != string::npos) {
            solomon_format = true;
            reading_coords = true;
            continue;
        }
        
        if (reading_coords) {
            int id; double x, y; double d = 0, r = 0, du = 999999, s = 0; 
            stringstream ss(line);
            if (ss >> id >> x >> y) {
                if (ss >> d >> r >> du >> s) {}
                // FORCE CVRP MODE: Ignore Time Windows and Service Times
                data.nodes.push_back({id, x, y, (long long)d, 0, 9999999, 0});
            }
        }
    }

    if (data.nodes.size() > 0) {
        int max_id = 0;
        for (auto& n : data.nodes) max_id = max(max_id, n.id);
        vector<Node> padded_nodes(max_id + 1);
        for (auto& n : data.nodes) padded_nodes[n.id] = n;
        data.nodes = padded_nodes;
    }
    
    return data;
}

// PI Variables
double global_alpha_cap = 100.0; // High initial penalty

bool update_route_times(const vector<int>& route, const VRPData& data) {
    if (route.empty()) return true;
    double t = 0;
    int current = 0;
    for (int node_idx : route) {
        double d = data.dist(current, node_idx);
        double arr = t + d;
        if (arr > (double)data.nodes[node_idx].due) return false;
        t = max(arr, (double)data.nodes[node_idx].ready) + (double)data.nodes[node_idx].service;
        current = node_idx;
    }
    double arr_final = t + data.dist(current, 0);
    if (arr_final > (double)data.nodes[0].due) return false;
    return true;
}

struct RouteContext {
    vector<double> arrival;
    vector<double> wait;
    vector<double> max_delay;
};

void build_route_context(const vector<int>& route, const VRPData& data, RouteContext& ctx) {
    int L = route.size();
    ctx.arrival.assign(L, 0.0);
    ctx.wait.assign(L, 0.0);
    ctx.max_delay.assign(L, 0.0);
    if (L == 0) return;
    
    double t = 0;
    int current = 0;
    for (int i = 0; i < L; ++i) {
        int node = route[i];
        double arr = t + data.dist(current, node);
        ctx.arrival[i] = arr;
        ctx.wait[i] = max(0.0, (double)data.nodes[node].ready - arr);
        t = max(arr, (double)data.nodes[node].ready) + (double)data.nodes[node].service;
        current = node;
    }
    
    double depot_slack = (double)data.nodes[0].due - (t + data.dist(current, 0));
    double next_delay = depot_slack;
    
    for (int i = L - 1; i >= 0; --i) {
        int node = route[i];
        double node_slack = (double)data.nodes[node].due - ctx.arrival[i];
        double allowed = min(node_slack, next_delay + ctx.wait[i]);
        ctx.max_delay[i] = allowed;
        next_delay = allowed;
    }
}

bool can_insert_O1(int u, const vector<int>& r, const RouteContext& ctx, int pos, const VRPData& data) {
    double t_dep_prev = 0;
    if (pos > 0) {
        int p = pos - 1;
        t_dep_prev = ctx.arrival[p] + ctx.wait[p] + (double)data.nodes[r[p]].service;
    }
    
    double arr_u = t_dep_prev + data.dist((pos == 0) ? 0 : r[pos-1], u);
    if (arr_u > (double)data.nodes[u].due) return false;
    
    double t_dep_u = max(arr_u, (double)data.nodes[u].ready) + (double)data.nodes[u].service;
    
    if (pos == (int)r.size()) {
        double arr_depot = t_dep_u + data.dist(u, 0);
        return arr_depot <= (double)data.nodes[0].due;
    } else {
        double new_arr_next = t_dep_u + data.dist(u, r[pos]);
        double delay = new_arr_next - ctx.arrival[pos];
        return delay <= ctx.max_delay[pos];
    }
}

bool can_replace_O1(int u, const vector<int>& r, const RouteContext& ctx, int i, const VRPData& data) {
    double t_dep_prev = 0;
    if (i > 0) {
        int p = i - 1;
        t_dep_prev = ctx.arrival[p] + ctx.wait[p] + (double)data.nodes[r[p]].service;
    }
    
    double arr_u = t_dep_prev + data.dist((i == 0) ? 0 : r[i-1], u);
    if (arr_u > (double)data.nodes[u].due) return false;
    
    double t_dep_u = max(arr_u, (double)data.nodes[u].ready) + (double)data.nodes[u].service;
    
    if (i == (int)r.size() - 1) {
        double arr_depot = t_dep_u + data.dist(u, 0);
        return arr_depot <= (double)data.nodes[0].due;
    } else {
        double new_arr_next = t_dep_u + data.dist(u, r[i+1]);
        double delay = new_arr_next - ctx.arrival[i+1];
        return delay <= ctx.max_delay[i+1];
    }
}

bool is_feasible(const vector<vector<int>>& routes, const VRPData& data) {
    for (const auto& r : routes) {
        long long load = 0;
        for (int n : r) load += data.nodes[n].demand;
        if (load > data.capacity) return false;
    }
    return true;
}

double evaluate_route_temporal(const vector<int>& route, const VRPData& data) {
    double t = 0;
    int current = 0;
    for (int node_idx : route) {
        double d = data.dist(current, node_idx);
        double arr = t + d;
        if (arr > (double)data.nodes[node_idx].due) return -1.0;
        t = max(arr, (double)data.nodes[node_idx].ready) + (double)data.nodes[node_idx].service;
        current = node_idx;
    }
    double arr_final = t + data.dist(current, 0);
    if (arr_final > (double)data.nodes[0].due) return -1.0;
    return arr_final;
}

double evaluate_route(const vector<int>& route, const VRPData& data) {
    double cost = 0;
    int current = 0;
    for (int node_idx : route) {
        cost += data.dist(current, node_idx);
        current = node_idx;
    }
    cost += data.dist(current, 0);
    return cost;
}

double evaluate_all(const vector<vector<int>>& routes, const VRPData& data, double alpha_cap) {
    double total = 0;
    for (const auto& r : routes) {
        total += evaluate_route(r, data);
        long long load = 0; for(int n: r) load += data.nodes[n].demand;
        if(load > data.capacity) {
            total += alpha_cap * (load - data.capacity);
        }
    }
    return total;
}

double fitness_eval(const vector<vector<int>>& routes, const VRPData& data, double alpha_cap = global_alpha_cap) {
    return evaluate_all(routes, data, alpha_cap);
}

vector<vector<int>> solomon_i1(const VRPData& data) {
    vector<vector<int>> routes;
    vector<bool> unvisited(data.nodes.size(), true);
    unvisited[0] = false; 
    int remaining = data.nodes.size() - 1;
    vector<int> candidates;
    for (size_t i = 1; i < data.nodes.size(); i++) candidates.push_back(i);
    
    sort(candidates.begin(), candidates.end(), [&](int a, int b) { return data.dist(0, a) > data.dist(0, b); });
    
    while (remaining > 0 && routes.size() < (size_t)data.max_vehicles) {
        vector<int> current_route; long long current_load = 0;
        while (remaining > 0) {
            int best_u = -1, best_pos = -1; double best_cost = 1e15;
            for (int u : candidates) {
                if (!unvisited[u] || current_load + data.nodes[u].demand > data.capacity) continue;
                for (size_t pos = 0; pos <= current_route.size(); ++pos) {
                    vector<int> cand_route = current_route; cand_route.insert(cand_route.begin() + pos, u);
                    if (update_route_times(cand_route, data)) {
                        int node_i = (pos == 0) ? 0 : current_route[pos - 1];
                        int node_j = (pos == current_route.size()) ? 0 : current_route[pos];
                        double dist_cost = data.dist(node_i, u) + data.dist(u, node_j) - data.dist(node_i, node_j);
                        long long cost = dist_cost;
                        
                        if (data.capacity >= 500) {
                            double old_arr_time = evaluate_route_temporal(current_route, data);
                            double arr_time = evaluate_route_temporal(cand_route, data);
                            double time_shift = arr_time - (old_arr_time == -1.0 ? 0.0 : old_arr_time);
                            
                            cost = dist_cost + (time_shift / 10.0);
                        }
                        
                        if (cost < best_cost) { best_cost = cost; best_u = u; best_pos = pos; }
                    }
                }
            }
            if (best_u != -1) {
                unvisited[best_u] = false; current_route.insert(current_route.begin() + best_pos, best_u);
                current_load += data.nodes[best_u].demand; remaining--;
            } else break;
        }
        if (!current_route.empty()) routes.push_back(current_route);
        else break;
    }
    for(size_t i=1; i<data.nodes.size(); i++) if(unvisited[i]) routes.push_back({(int)i});
    return routes;
}

thread_local vector<int> scratch_route;
thread_local vector<int> scratch_route2;

void optimize_routes(vector<vector<int>>& routes, const VRPData& data) {
    bool improved = true;
    int opt_counter = 0;
    while (improved) {
        if (++opt_counter > 2000) {
            // std::cout << "DEBUG: optimize_routes hit 2000 iterations limit!" << std::endl;
            break;
        }
        improved = false;
        
        vector<int> node_to_route(data.nodes.size(), -1);
        vector<RouteContext> contexts(routes.size());
        vector<long long> route_loads(routes.size(), 0);
        
        for(size_t r=0; r<routes.size(); ++r) {
            for(int n : routes[r]) {
                node_to_route[n] = r;
                route_loads[r] += data.nodes[n].demand;
            }
            build_route_context(routes[r], data, contexts[r]);
        }
        
        // 0. Route Elimination
        vector<pair<int, int>> route_sizes;
        for (size_t i = 0; i < routes.size(); ++i) route_sizes.push_back({routes[i].size(), i});
        sort(route_sizes.begin(), route_sizes.end());
        
        for (auto [sz, i] : route_sizes) {
            if (sz == 0 || sz > 12) continue;
            bool success = true;
            vector<vector<int>> temp_routes = routes;
            for (int u : routes[i]) {
                int best_r = -1, best_p = -1; double best_c = 1e15;
                
                vector<int> check_r2;
                vector<bool> seen_r2(temp_routes.size(), false);
                for (int v : data.knn[u]) {
                    int r2 = node_to_route[v];
                    if (r2 != -1 && r2 != i && !seen_r2[r2]) {
                        seen_r2[r2] = true;
                        check_r2.push_back(r2);
                    }
                }
                
                for (size_t r_idx : check_r2) {
                    long long load = 0; for(int n: temp_routes[r_idx]) load += data.nodes[n].demand;
                    if (load + data.nodes[u].demand > data.capacity) continue;
                    for (size_t pos = 0; pos <= temp_routes[r_idx].size(); ++pos) {
                        vector<int> cand = temp_routes[r_idx]; cand.insert(cand.begin() + pos, u);
                        if (update_route_times(cand, data)) {
                            int p_n = (pos == 0) ? 0 : temp_routes[r_idx][pos-1];
                            int n_n = (pos == temp_routes[r_idx].size()) ? 0 : temp_routes[r_idx][pos];
                            long long cost = data.dist(p_n, u) + data.dist(u, n_n) - data.dist(p_n, n_n);
                            if (cost < best_c) { best_c = cost; best_r = r_idx; best_p = pos; }
                        }
                    }
                }
                if (best_r != -1) {
                    temp_routes[best_r].insert(temp_routes[best_r].begin() + best_p, u);
                    // Update node_to_route for the next node in the route being eliminated
                    node_to_route[u] = best_r;
                }
                else { success = false; break; }
            }
            if (success) {
                temp_routes[i].clear();
                routes.clear();
                for (auto& r : temp_routes) if (!r.empty()) routes.push_back(r);
                improved = true; break;
            }
        }
        if (improved) continue;
        
        // 1. Inter-route Relocate
        for (size_t r1_idx = 0; r1_idx < routes.size(); ++r1_idx) {
            if (routes[r1_idx].size() <= 1) continue;
            for (size_t i = 0; i < routes[r1_idx].size(); ++i) {
                int u = routes[r1_idx][i];
                int best_r2 = -1, best_j = -1; double best_savings = 0;
                int n_prev = (i == 0) ? 0 : routes[r1_idx][i-1];
                int n_next = (i == routes[r1_idx].size() - 1) ? 0 : routes[r1_idx][i+1];
                double rem_cost = data.dist(n_prev, u) + data.dist(u, n_next) - data.dist(n_prev, n_next);
                
                vector<int> check_r2;
                vector<bool> seen_r2(routes.size(), false);
                for (int v : data.knn[u]) {
                    int r2 = node_to_route[v];
                    if (r2 != -1 && r2 != r1_idx && !seen_r2[r2]) {
                        seen_r2[r2] = true;
                        check_r2.push_back(r2);
                    }
                }
                for (size_t r2_idx : check_r2) {
                    if (route_loads[r2_idx] + data.nodes[u].demand > data.capacity) continue;
                    for (size_t j = 0; j <= routes[r2_idx].size(); ++j) {
                        int m_prev = (j == 0) ? 0 : routes[r2_idx][j-1];
                        int m_next = (j == routes[r2_idx].size()) ? 0 : routes[r2_idx][j];
                        double ins_cost = data.dist(m_prev, u) + data.dist(u, m_next) - data.dist(m_prev, m_next);
                        double savings = rem_cost - ins_cost;
                        if (savings > best_savings) {
                            if (can_insert_O1(u, routes[r2_idx], contexts[r2_idx], j, data)) {
                                best_savings = savings; best_r2 = r2_idx; best_j = j;
                            }
                        }
                    }
                }
                if (best_savings > 0) {
                    routes[best_r2].insert(routes[best_r2].begin() + best_j, u);
                    routes[r1_idx].erase(routes[r1_idx].begin() + i);
                    improved = true; break;
                }
            }
            if (improved) break;
        }
        if (improved) continue;
        
        // 2. Inter-route Exchange
        for (size_t r1_idx = 0; r1_idx < routes.size(); ++r1_idx) {
            for (size_t i = 0; i < routes[r1_idx].size(); ++i) {
                int u = routes[r1_idx][i];
                int best_r2 = -1, best_j = -1; double best_savings = 0;
                int n_prev = (i == 0) ? 0 : routes[r1_idx][i-1];
                int n_next = (i == routes[r1_idx].size() - 1) ? 0 : routes[r1_idx][i+1];
                double u_rem_cost = data.dist(n_prev, u) + data.dist(u, n_next) - data.dist(n_prev, n_next);
                
                vector<int> check_r2;
                vector<bool> seen_r2(routes.size(), false);
                for (int v : data.knn[u]) {
                    int r2 = node_to_route[v];
                    if (r2 != -1 && r2 > r1_idx && !seen_r2[r2]) {
                        seen_r2[r2] = true;
                        check_r2.push_back(r2);
                    }
                }
                for (size_t r2_idx : check_r2) {
                    for (size_t j = 0; j < routes[r2_idx].size(); ++j) {
                        int v = routes[r2_idx][j];
                        if (route_loads[r1_idx] - data.nodes[u].demand + data.nodes[v].demand > data.capacity) continue;
                        if (route_loads[r2_idx] - data.nodes[v].demand + data.nodes[u].demand > data.capacity) continue;
                        
                        int m_prev = (j == 0) ? 0 : routes[r2_idx][j-1];
                        int m_next = (j == routes[r2_idx].size() - 1) ? 0 : routes[r2_idx][j+1];
                        double v_rem_cost = data.dist(m_prev, v) + data.dist(v, m_next) - data.dist(m_prev, m_next);
                        
                        double v_ins_cost = data.dist(n_prev, v) + data.dist(v, n_next) - data.dist(n_prev, n_next);
                        double u_ins_cost = data.dist(m_prev, u) + data.dist(u, m_next) - data.dist(m_prev, m_next);
                        
                        double savings = (u_rem_cost + v_rem_cost) - (v_ins_cost + u_ins_cost);
                        if (savings > best_savings) {
                            if (can_replace_O1(v, routes[r1_idx], contexts[r1_idx], i, data) &&
                                can_replace_O1(u, routes[r2_idx], contexts[r2_idx], j, data)) {
                                best_savings = savings; best_r2 = r2_idx; best_j = j;
                            }
                        }
                    }
                }
                if (best_savings > 0) {
                    int v = routes[best_r2][best_j];
                    routes[best_r2][best_j] = u;
                    routes[r1_idx][i] = v;
                    improved = true; break;
                }
            }
            if (improved) break;
        }
        if (improved) continue;
        // 2.6 True Cross Exchange (Segment Swap L1xL2 up to 3x3)
        // 2.6 True Cross Exchange (Segment Swap L1xL2 up to 3x3)
        for (size_t r1_idx = 0; r1_idx < routes.size(); ++r1_idx) {
            if (routes[r1_idx].empty()) continue;
            
            vector<int> check_r2;
            vector<bool> seen_r2(routes.size(), false);
            for (int u : routes[r1_idx]) {
                for (int v : data.knn[u]) {
                    int r2 = node_to_route[v];
                    if (r2 != -1 && r2 != r1_idx && !seen_r2[r2]) {
                        seen_r2[r2] = true;
                        check_r2.push_back(r2);
                    }
                }
            }
            
            for (size_t r2_idx : check_r2) {
                if (routes[r2_idx].empty()) continue;
                for (int L1 = 1; L1 <= 3; ++L1) {
                    if (L1 > (int)routes[r1_idx].size()) break;
                    for (int L2 = 1; L2 <= 3; ++L2) {
                        if (L2 > (int)routes[r2_idx].size()) break;
                        if (L1 == 1 && L2 == 1) continue; // Handled by standard exchange
                        for (size_t i = 0; i <= routes[r1_idx].size() - L1; ++i) {
                            for (size_t j = 0; j <= routes[r2_idx].size() - L2; ++j) {
                                int u_start = routes[r1_idx][i]; int u_end = routes[r1_idx][i + L1 - 1];
                                int p1 = (i == 0) ? 0 : routes[r1_idx][i - 1];
                                int n1 = (i + L1 == routes[r1_idx].size()) ? 0 : routes[r1_idx][i + L1];
                                
                                int v_start = routes[r2_idx][j]; int v_end = routes[r2_idx][j + L2 - 1];
                                int p2 = (j == 0) ? 0 : routes[r2_idx][j - 1];
                                int n2 = (j + L2 == routes[r2_idx].size()) ? 0 : routes[r2_idx][j + L2];
                                
                                double old_edges = data.dist(p1, u_start) + data.dist(u_end, n1) + data.dist(p2, v_start) + data.dist(v_end, n2);
                                double new_edges = data.dist(p1, v_start) + data.dist(v_end, n1) + data.dist(p2, u_start) + data.dist(u_end, n2);
                                
                                if (new_edges >= old_edges - 1e-6) continue;
                                
                                scratch_route = routes[r1_idx];
                                scratch_route.erase(scratch_route.begin() + i, scratch_route.begin() + i + L1);
                                scratch_route.insert(scratch_route.begin() + i, routes[r2_idx].begin() + j, routes[r2_idx].begin() + j + L2);
                                long long load1 = 0; for(int n: scratch_route) load1 += data.nodes[n].demand;
                                if(load1 > data.capacity) continue;
                                
                                scratch_route2 = routes[r2_idx];
                                scratch_route2.erase(scratch_route2.begin() + j, scratch_route2.begin() + j + L2);
                                scratch_route2.insert(scratch_route2.begin() + j, routes[r1_idx].begin() + i, routes[r1_idx].begin() + i + L1);
                                long long load2 = 0; for(int n: scratch_route2) load2 += data.nodes[n].demand;
                                if(load2 > data.capacity) continue;
                                
                                if (update_route_times(scratch_route, data) && update_route_times(scratch_route2, data)) {
                                    routes[r1_idx] = scratch_route; routes[r2_idx] = scratch_route2;
                                    improved = true; break;
                                }
                            }
                            if (improved) break;
                        }
                        if (improved) break;
                    }
                    if (improved) break;
                }
                if (improved) break;
            }
            if (improved) break;
        }
        if (improved) continue;
        
        // 2.7 Inter-route 2-OPT* (Tail Swap)
        for (size_t r1_idx = 0; r1_idx < routes.size(); ++r1_idx) {
            if (routes[r1_idx].empty()) continue;
            
            vector<int> check_r2;
            vector<bool> seen_r2(routes.size(), false);
            for (int u : routes[r1_idx]) {
                for (int v : data.knn[u]) {
                    int r2 = node_to_route[v];
                    if (r2 != -1 && r2 != r1_idx && !seen_r2[r2]) {
                        seen_r2[r2] = true;
                        check_r2.push_back(r2);
                    }
                }
            }
            
            for (size_t r2_idx : check_r2) {
                if (routes[r2_idx].empty()) continue;
                for (size_t i = 0; i < routes[r1_idx].size(); ++i) {
                    for (size_t j = 0; j < routes[r2_idx].size(); ++j) {
                        scratch_route.assign(routes[r1_idx].begin(), routes[r1_idx].begin() + i + 1);
                        scratch_route.insert(scratch_route.end(), routes[r2_idx].begin() + j + 1, routes[r2_idx].end());
                        
                        long long load1 = 0; for(int n: scratch_route) load1 += data.nodes[n].demand;
                        if(load1 > data.capacity) continue;
                        
                        scratch_route2.assign(routes[r2_idx].begin(), routes[r2_idx].begin() + j + 1);
                        scratch_route2.insert(scratch_route2.end(), routes[r1_idx].begin() + i + 1, routes[r1_idx].end());
                        
                        long long load2 = 0; for(int n: scratch_route2) load2 += data.nodes[n].demand;
                        if(load2 > data.capacity) continue;
                        
                        double old_cost = evaluate_route(routes[r1_idx], data) + evaluate_route(routes[r2_idx], data);
                        double new_cost = evaluate_route(scratch_route, data) + evaluate_route(scratch_route2, data);
                        
                        if (new_cost < old_cost) {
                            if (update_route_times(scratch_route, data) && update_route_times(scratch_route2, data)) {
                                routes[r1_idx] = scratch_route;
                                routes[r2_idx] = scratch_route2;
                                improved = true; break;
                            }
                        }
                    }
                    if (improved) break;
                }
                if (improved) break;
            }
            if (improved) break;
        }
        if (improved) continue;
        
        // 3. Intra-route 2-opt
        for (size_t r_idx = 0; r_idx < routes.size(); ++r_idx) {
            auto& r = routes[r_idx];
            if (r.size() < 3) continue;
            bool r_imp = true;
            while (r_imp) {
                r_imp = false;
                for (size_t i = 0; i < r.size() - 1; ++i) {
                    for (size_t k = i + 2; k < r.size(); ++k) {
                        int n1 = (i == 0) ? 0 : r[i-1];
                        int n2 = r[i];
                        int n3 = r[k];
                        int n4 = (k == r.size() - 1) ? 0 : r[k+1];
                        double cur_cost = data.dist(n1, n2) + data.dist(n3, n4);
                        double new_cost = data.dist(n1, n3) + data.dist(n2, n4);
                        if (cur_cost > new_cost + 1e-6) {
                            vector<int> cand = r;
                            reverse(cand.begin() + i, cand.begin() + k + 1);
                            if (update_route_times(cand, data)) {
                                r = cand; improved = true; r_imp = true; break;
                            }
                        }
                    }
                    if (r_imp) break;
                }
            }
        }
        
        // 4. Intra-route Or-opt: relocate segments of 2 or 3 consecutive nodes
        for (size_t r_idx = 0; r_idx < routes.size(); ++r_idx) {
            auto& r = routes[r_idx];
            if (r.size() < 4) continue;
            for (int seg_len = 3; seg_len >= 2; --seg_len) {
                for (size_t i = 0; i + seg_len - 1 < r.size(); ++i) {
                    // Remove segment [i, i+seg_len-1]
                    int prev_seg = (i == 0) ? 0 : r[i-1];
                    int first_seg = r[i];
                    int last_seg = r[i + seg_len - 1];
                    int next_seg = (i + seg_len >= r.size()) ? 0 : r[i + seg_len];
                    double rem_cost = data.dist(prev_seg, first_seg) + data.dist(last_seg, next_seg);
                    double bridge = data.dist(prev_seg, next_seg);
                    
                    for (size_t j = 0; j < r.size() - seg_len + 1; ++j) {
                        if (j >= i && j <= i + seg_len) continue;
                        int ins_prev = (j == 0 && i != 0) ? 0 : (j <= i ? r[j == 0 ? 0 : j-1] : r[j + seg_len - 1 < r.size() ? j + seg_len - 1 : r.size()-1]);
                        // Simpler: just try the move and evaluate
                        vector<int> cand;
                        for (size_t k = 0; k < r.size(); ++k) {
                            if (k == i) { k += seg_len - 1; continue; } // skip segment
                            cand.push_back(r[k]);
                            if (cand.size() == j + 1 - (j > i ? seg_len : 0) || (j == 0 && k == 0 && i != 0)) {
                                // This approach is getting complex. Use direct move.
                            }
                        }
                        // Direct approach: build candidate by removing then inserting
                        cand.clear();
                        vector<int> seg;
                        for (size_t k = 0; k < r.size(); ++k) {
                            if (k >= i && k < i + seg_len) seg.push_back(r[k]);
                            else cand.push_back(r[k]);
                        }
                        size_t ins_pos = (j > i) ? j - seg_len : j;
                        if (ins_pos > cand.size()) ins_pos = cand.size();
                        cand.insert(cand.begin() + ins_pos, seg.begin(), seg.end());
                        
                        double old_c = evaluate_route(r, data);
                        double new_c = evaluate_route(cand, data);
                        if (new_c < old_c - 1e-6 && update_route_times(cand, data)) {
                            r = cand; improved = true; break;
                        }
                    }
                    if (improved) break;
                }
                if (improved) break;
            }
            if (improved) break;
        }
    }
}

void load_external_matrix(string filename, VRPData& data) {
    ifstream file(filename);
    if (!file.is_open()) return;
    string line;
    getline(file, line); 
    while (getline(file, line)) {
        stringstream ss(line);
        string token;
        int from = -1, to = -1;
        double dist = -1;
        try {
            getline(ss, token, ','); from = stoi(token);
            getline(ss, token, ','); to = stoi(token);
            getline(ss, token, ','); dist = stod(token);
            if (from >= 0 && from < (int)data.matrix.size() && to >= 0 && to < (int)data.matrix.size()) {
                data.matrix[from][to] = dist;
            }
        } catch (...) {}
    }
}

int main(int argc, char* argv[]) {
    if (const char* env_p = std::getenv("USE_TRUNCATED_MATH")) {
        if (std::string(env_p) == "1") {
        }
    }
    if (const char* env_p2 = std::getenv("USE_INTEGER_MATH")) {
        if (std::string(env_p2) == "1") {
            use_integer_math = true;
            cout << "[INFO] Engine running in INTEGER physics mode (round(d))" << endl;
        }
    }
    if (const char* env_p = std::getenv("USE_TRUNCATED_MATH")) {
        if (std::string(env_p) == "1") {
            use_truncated_math = true;
            cout << "[INFO] Engine running in TRUNCATED physics mode (* 10.0 -> / 10.0)" << endl;
        }
    }

    string filename = (argc > 1) ? argv[1] : "api_job_test.vrp";
    double time_limit = (argc > 2) ? stod(argv[2]) : 300.0;
    VRPData data = load_vrp(filename);
    if (data.nodes.size() < 2) return 1;
    data.build_matrix();
    if (argc > 3) {
        string matrix_file = argv[3];
        load_external_matrix(matrix_file, data);
        cout << "[INFO] Loaded External Distance Matrix: " << matrix_file << endl;
    }

    vector<vector<int>> best_routes;
    double best_total_cost = 1e15;
    int best_vehicles = 1e9;
    
    vector<vector<int>> routes = solomon_i1(data);
    optimize_routes(routes, data);
    double fit = evaluate_all(routes, data, global_alpha_cap);
    int v_count = 0; for (const auto& r : routes) if (!r.empty()) v_count++;
    
    best_vehicles = v_count;
    best_total_cost = fit;
    best_routes = routes;
    
    ofstream log_file("cvrp_convergence.csv");
    log_file << "Time,Cost\n";
    log_file.close();
    
    auto start_time = high_resolution_clock::now();
    
    #pragma omp parallel
    {
        mt19937 rng(42 + omp_get_thread_num());
        vector<vector<int>> current_routes = best_routes;
        double current_fit = best_total_cost;
        int iter_count = 0;
        
        while (true) {
            double elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start_time).count() / 1000.0;
            double progress = elapsed / time_limit;
            if (elapsed > time_limit) break;
            
            if (omp_get_thread_num() == 0 && (iter_count % 100 == 0)) {
                ofstream log_file_app("cvrp_convergence.csv", ios::app);
                log_file_app << elapsed << "," << best_total_cost << "\n";
                log_file_app.close();
            }
            iter_count++;
            
            global_alpha_cap = 1.0 + (999.0 * progress);
            
            vector<vector<int>> perturbed = current_routes;
            vector<int> removed_nodes;
            
            int ruin_type = rng() % 3; // 0=Shaw, 1=Random, 2=Worst-cost
            
            if (ruin_type == 0) {
                // Shaw Ruin: remove nodes closest to a random seed node
                int seed = 1 + (rng() % (data.nodes.size() - 1));
                vector<pair<double, pair<int, int>>> distances;
                for(size_t r=0; r<perturbed.size(); ++r) {
                    for(size_t n=0; n<perturbed[r].size(); ++n) {
                        int u = perturbed[r][n];
                        distances.push_back({data.dist(seed, u), {(int)r, (int)n}});
                    }
                }
                sort(distances.begin(), distances.end());
                int num_eject = 40 + (rng() % 80); // Standard ruin size
                vector<pair<int, int>> to_remove;
                for(int i=0; i<min((int)distances.size(), num_eject); ++i) {
                    to_remove.push_back(distances[i].second);
                }
                sort(to_remove.rbegin(), to_remove.rend());
                for(auto p : to_remove) {
                    removed_nodes.push_back(perturbed[p.first][p.second]);
                    perturbed[p.first].erase(perturbed[p.first].begin() + p.second);
                }
            } else if (ruin_type == 1) {
                // Random Ruin: remove entire random routes + random individual nodes
                int num_routes_to_ruin = 2 + (rng() % 4);
                for(int k=0; k<num_routes_to_ruin; k++) {
                    if(perturbed.empty()) break;
                    int r_idx = rng() % perturbed.size();
                    for(int u : perturbed[r_idx]) removed_nodes.push_back(u);
                    perturbed.erase(perturbed.begin() + r_idx);
                }
                int num_eject = 15 + (rng() % 40);
                for(int k=0; k<num_eject; k++) {
                    if(perturbed.empty()) break;
                    int r_idx = rng() % perturbed.size();
                    if(perturbed[r_idx].empty()) continue;
                    int n_idx = rng() % perturbed[r_idx].size();
                    removed_nodes.push_back(perturbed[r_idx][n_idx]);
                    perturbed[r_idx].erase(perturbed[r_idx].begin() + n_idx);
                }
            } else {
                // Worst-cost Ruin: remove the most expensive (highest detour cost) nodes
                vector<pair<double, pair<int, int>>> costs;
                for(size_t r=0; r<perturbed.size(); ++r) {
                    for(size_t n=0; n<perturbed[r].size(); ++n) {
                        int u = perturbed[r][n];
                        int prev = (n == 0) ? 0 : perturbed[r][n-1];
                        int next = (n == perturbed[r].size()-1) ? 0 : perturbed[r][n+1];
                        double detour = data.dist(prev, u) + data.dist(u, next) - data.dist(prev, next);
                        costs.push_back({detour, {(int)r, (int)n}});
                    }
                }
                sort(costs.rbegin(), costs.rend()); // Most expensive first
                int num_eject = 40 + (rng() % 80);
                vector<pair<int, int>> to_remove;
                for(int i=0; i<min((int)costs.size(), num_eject); ++i) {
                    to_remove.push_back(costs[i].second);
                }
                sort(to_remove.rbegin(), to_remove.rend());
                for(auto p : to_remove) {
                    removed_nodes.push_back(perturbed[p.first][p.second]);
                    perturbed[p.first].erase(perturbed[p.first].begin() + p.second);
                }
            }
            
            while (!removed_nodes.empty()) {
                double max_regret = -1;
                int best_u_idx = -1;
                int best_r = -1, best_p = -1;
                
                for (size_t i = 0; i < removed_nodes.size(); ++i) {
                    int u = removed_nodes[i];
                    double best_c = 1e15, second_best_c = 1e15;
                    int cur_best_r = -1, cur_best_p = -1;
                    
                    for (size_t r_idx = 0; r_idx < perturbed.size(); ++r_idx) {
                        long long load = 0; for(int n: perturbed[r_idx]) load += data.nodes[n].demand;
                        if (load + data.nodes[u].demand > data.capacity) continue;
                        for (size_t pos = 0; pos <= perturbed[r_idx].size(); ++pos) {
                            vector<int> cand = perturbed[r_idx]; cand.insert(cand.begin() + pos, u);
                            if (update_route_times(cand, data)) {
                                int p_n = (pos == 0) ? 0 : perturbed[r_idx][pos-1];
                                int n_n = (pos == perturbed[r_idx].size()) ? 0 : perturbed[r_idx][pos];
                                long long cost = data.dist(p_n, u) + data.dist(u, n_n) - data.dist(p_n, n_n);
                                if (cost < best_c) {
                                    second_best_c = best_c;
                                    best_c = cost;
                                    cur_best_r = r_idx; cur_best_p = pos;
                                } else if (cost < second_best_c) {
                                    second_best_c = cost;
                                }
                            }
                        }
                    }
                    if (cur_best_r != -1) {
                        double regret = (second_best_c == 1e15) ? 0 : (second_best_c - best_c);
                        if (regret > max_regret) {
                            max_regret = regret;
                            best_u_idx = i;
                            best_r = cur_best_r;
                            best_p = cur_best_p;
                        }
                    }
                }
                
                if (best_u_idx != -1) {
                    int u = removed_nodes[best_u_idx];
                    perturbed[best_r].insert(perturbed[best_r].begin() + best_p, u);
                    removed_nodes.erase(removed_nodes.begin() + best_u_idx);
                } else {
                    if (data.capacity >= 500) {
                        perturbed.push_back({removed_nodes[0]});
                        removed_nodes.erase(removed_nodes.begin());
                    } else {
                        for (int u : removed_nodes) perturbed.push_back({u});
                        removed_nodes.clear();
                    }
                }
            }
            
            optimize_routes(perturbed, data);
            double new_fit = fitness_eval(perturbed, data);
            
            if (new_fit < current_fit) {
                current_routes = perturbed;
                current_fit = new_fit;
                
                int pert_veh = 0; for(const auto& r: perturbed) if(!r.empty()) pert_veh++;
                
                #pragma omp critical
                {
                    int best_veh = 0; for(const auto& r: best_routes) if(!r.empty()) best_veh++;
                    if (pert_veh < best_veh || (pert_veh == best_veh && new_fit < best_total_cost)) {
                        if (is_feasible(perturbed, data)) {
                            best_total_cost = new_fit;
                            best_routes = current_routes;
                        }
                    }
                }
            } else {
                double diff = new_fit - current_fit;
                double temp = 15.0 * exp(-4.0 * progress);
                if (exp(-diff / temp) > (double)(rng() % 1000) / 1000.0) {
                    current_routes = perturbed;
                    current_fit = new_fit;
                }
            }
            
            // HGS Genetic Crossover
            iter_count++;
            if (iter_count % 100 == 0) {
                vector<vector<int>> parent_b;
                #pragma omp critical
                {
                    parent_b = best_routes; // Inherit global best
                }
                
                // Keep 50% of parent B routes
                vector<vector<int>> child;
                vector<bool> assigned(data.nodes.size() + 1, false);
                
                // Initialize route IDs
                vector<int> route_ids(parent_b.size());
                iota(route_ids.begin(), route_ids.end(), 0);
                shuffle(route_ids.begin(), route_ids.end(), rng);
                
                int num_inherit = max(1, (int)(parent_b.size()) / 2);
                for(int i=0; i<num_inherit; i++) {
                    child.push_back(parent_b[route_ids[i]]);
                    for(int u : parent_b[route_ids[i]]) assigned[u] = true;
                }
                
                // Identify unassigned nodes
                vector<int> unassigned;
                for(size_t i=1; i<data.nodes.size(); i++) {
                    if(!assigned[data.nodes[i].id]) unassigned.push_back(data.nodes[i].id);
                }
                
                // Aggressive Route Elimination: dissolve the smallest route into unassigned
                if (child.size() > 1) {
                    int smallest_r = 0;
                    for (size_t r = 1; r < child.size(); r++) {
                        if (child[r].size() < child[smallest_r].size() && child[r].size() > 0) {
                            smallest_r = r;
                        }
                    }
                    if (child[smallest_r].size() <= 8) { // Dissolve if it's a small fragmented route
                        for (int u : child[smallest_r]) {
                            unassigned.push_back(u);
                        }
                        child.erase(child.begin() + smallest_r);
                    }
                }
                
                // Shuffle unassigned for randomization
                shuffle(unassigned.begin(), unassigned.end(), rng);
                
                // Greedily insert unassigned nodes into the child
                while(!unassigned.empty()) {
                    int u = unassigned.back();
                    unassigned.pop_back();
                    
                    double best_c = 1e15;
                    int cur_best_r = -1, cur_best_p = -1;
                    
                    for (size_t r_idx = 0; r_idx < child.size(); ++r_idx) {
                        long long load = 0; for(int n: child[r_idx]) load += data.nodes[n].demand;
                        if (load + data.nodes[u].demand > data.capacity) continue;
                        for (size_t pos = 0; pos <= child[r_idx].size(); ++pos) {
                            vector<int> cand = child[r_idx]; cand.insert(cand.begin() + pos, u);
                            if (update_route_times(cand, data)) {
                                int p_n = (pos == 0) ? 0 : child[r_idx][pos-1];
                                int n_n = (pos == child[r_idx].size()) ? 0 : child[r_idx][pos];
                                long long cost = data.dist(p_n, u) + data.dist(u, n_n) - data.dist(p_n, n_n);
                                if (cost < best_c) {
                                    best_c = cost;
                                    cur_best_r = r_idx; cur_best_p = pos;
                                }
                            }
                        }
                    }
                    
                    if (cur_best_r != -1) {
                        child[cur_best_r].insert(child[cur_best_r].begin() + cur_best_p, u);
                    } else {
                        child.push_back({u});
                    }
                }
                
                // Educate offspring
                optimize_routes(child, data);
                double child_fit = fitness_eval(child, data);
                
                // Accept child
                current_routes = child;
                current_fit = child_fit;
                
                #pragma omp critical
                {
                    if (child_fit < best_total_cost) {
                        if (is_feasible(child, data)) {
                            best_total_cost = child_fit;
                            best_routes = child;
                        }
                    }
                }
            }
        }
    }

    // Output formatted routes for appnvidia.py to parse
    for (size_t i = 0; i < best_routes.size(); ++i) {
        cout << "ROUTE " << i << ": ";
        for (int node : best_routes[i]) cout << node << " ";
        cout << endl;
    }
    cout << fixed << setprecision(2) << "FINAL_COST: " << best_total_cost << endl;

    return 0;
}

