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

using namespace std;
using namespace std::chrono;

#include <cstdlib>
double PHYSICS_MULTIPLIER = 10000.0;

struct Node {
    int id; double x, y; long long demand, ready, due, service;
    int pickup_id;
    int delivery_id;
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
        return sqrt(dx * dx + dy * dy);
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
    string line;
    data.capacity = 200; data.max_vehicles = 100;
    
    bool solomon_format = false;
    bool li_lim_format = false;
    bool reading_coords = false;
    
    // Peek at first line to detect Li & Lim format (first line: "vehicles capacity speed")
    if (getline(file, line)) {
        stringstream peek(line);
        int v, c, s;
        if ((peek >> v >> c >> s) && peek.eof()) {
            // Li & Lim format: first line is "num_vehicles capacity speed"
            li_lim_format = true;
            data.max_vehicles = v;
            data.capacity = c;
            reading_coords = true; // All remaining lines are node data
        } else {
            // Not Li & Lim, process normally
            file.seekg(0);
        }
    }
    
    while (getline(file, line)) {
        if (line.empty() || line.find("EOF") != string::npos) continue;
        
        if (!li_lim_format) {
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
        }
        
        if (reading_coords && !line.empty() && (isdigit(line[0]) || line[0] == ' ' || line[0] == '\t')) {
            int id, p_id = 0, d_id = 0;
            double x, y;
            long long d = 0, r = 0, du = 0, s = 0;
            stringstream ss(line);
            if (ss >> id >> x >> y) {
                if (ss >> d >> r >> du >> s) {
                    if (ss >> p_id >> d_id) {} // Parse pickup/delivery partners
                }
                data.nodes.push_back({id, x, y, (long long)d, (long long)r, (long long)du, (long long)s, p_id, d_id});
            }
        }
    }
    return data;
}


// PI Variables
double global_alpha_cap = 100.0;
#pragma omp threadprivate(global_alpha_cap)

bool update_route_times(const vector<int>& route, const VRPData& data) {
    if (route.empty()) return true;
    double t = 0;
    long long current_load = 0;
    int current = 0;
    vector<bool> picked_up(data.nodes.size() + 1, false);
    
    for (int node_idx : route) {
        if (data.nodes[node_idx].pickup_id != 0) {
            if (!picked_up[data.nodes[node_idx].pickup_id]) return false;
        }
        if (data.nodes[node_idx].delivery_id != 0) {
            picked_up[node_idx] = true;
        }
        
        current_load += data.nodes[node_idx].demand;
        if (current_load > data.capacity || current_load < 0) return false;
        
        double d = data.dist(current, node_idx);
        double arr = t + d;
        if (arr > (double)data.nodes[node_idx].due) return false;
        t = max(arr, (double)data.nodes[node_idx].ready) + (double)data.nodes[node_idx].service;
        current = node_idx;
    }
    double arr_final = t + data.dist(current, 0);
    if (arr_final > (double)data.nodes[0].due) return false;
    
    for (int node_idx : route) {
        if (data.nodes[node_idx].delivery_id != 0) {
            bool delivered = false;
            for (int scan : route) if (scan == data.nodes[node_idx].delivery_id) delivered = true;
            if (!delivered) return false;
        }
    }
    return true;
}

double evaluate_route(const vector<int>& route, const VRPData& data) {
    double cost = 0;
    int current = 0;
    for (int node_idx : route) {
        cost += data.dist(current, node_idx);
        current = node_idx;
    }
    return cost + data.dist(current, 0);
}

double evaluate_all(const vector<vector<int>>& routes, const VRPData& data) {
    double total = 0;
    for (const auto& r : routes) {
        if (!r.empty()) total += evaluate_route(r, data);
    }
    return total;
}

double fitness_eval(const vector<vector<int>>& routes, const VRPData& data) {
    double total = evaluate_all(routes, data);
    int vehicles = 0;
    for (const auto& r : routes) if (!r.empty()) vehicles++;
    return total + (vehicles * PHYSICS_MULTIPLIER);
}

void optimize_routes_pdp(vector<vector<int>>& routes, const VRPData& data) {
    bool improved = true;
    while (improved) {
        improved = false;
        
        // 1. Intra-route paired shift
        for (size_t r_idx = 0; r_idx < routes.size(); ++r_idx) {
            auto& r = routes[r_idx];
            if (r.size() < 4) continue;
            vector<pair<int, int>> pairs;
            for (int u : r) {
                if (data.nodes[u].delivery_id != 0) pairs.push_back({u, data.nodes[u].delivery_id});
            }
            
            for (auto p : pairs) {
                int u = p.first;
                int v = p.second;
                vector<int> cand;
                for (int n : r) if (n != u && n != v) cand.push_back(n);
                
                double best_c = evaluate_route(r, data);
                vector<int> best_cand = r;
                bool found = false;
                
                for (size_t i = 0; i <= cand.size(); ++i) {
                    for (size_t j = i; j <= cand.size() + 1; ++j) {
                        vector<int> test = cand;
                        test.insert(test.begin() + i, u);
                        test.insert(test.begin() + j, v);
                        if (update_route_times(test, data)) {
                            double c = evaluate_route(test, data);
                            if (c < best_c) {
                                best_c = c;
                                best_cand = test;
                                found = true;
                            }
                        }
                    }
                }
                if (found) { r = best_cand; improved = true; }
            }
        }
        if (improved) continue;
        
        // 2. Inter-route paired relocate
        for (size_t r1 = 0; r1 < routes.size(); ++r1) {
            for (size_t r2 = 0; r2 < routes.size(); ++r2) {
                if (r1 == r2 || routes[r1].empty()) continue;
                
                vector<pair<int, int>> pairs;
                for (int u : routes[r1]) {
                    if (data.nodes[u].delivery_id != 0) pairs.push_back({u, data.nodes[u].delivery_id});
                }
                
                for (auto p : pairs) {
                    int u = p.first;
                    int v = p.second;
                    
                    vector<int> c1;
                    for (int n : routes[r1]) if (n != u && n != v) c1.push_back(n);
                    if (!update_route_times(c1, data)) continue;
                    
                    double base_cost = evaluate_route(routes[r1], data) + evaluate_route(routes[r2], data);
                    double best_c = base_cost;
                    vector<int> best_c2;
                    bool found = false;
                    
                    for (size_t i = 0; i <= routes[r2].size(); ++i) {
                        for (size_t j = i; j <= routes[r2].size() + 1; ++j) {
                            vector<int> c2 = routes[r2];
                            c2.insert(c2.begin() + i, u);
                            c2.insert(c2.begin() + j, v);
                            if (update_route_times(c2, data)) {
                                double cost = evaluate_route(c1, data) + evaluate_route(c2, data);
                                if (cost < best_c) {
                                    best_c = cost;
                                    best_c2 = c2;
                                    found = true;
                                }
                            }
                        }
                    }
                    if (found) {
                        routes[r1] = c1; routes[r2] = best_c2;
                        improved = true; break;
                    }
                }
                if (improved) break;
            }
            if (improved) break;
        }
    }
}

vector<vector<int>> pdp_initial_insertion(const VRPData& data) {
    vector<vector<int>> routes;
    vector<bool> unvisited(data.nodes.size(), true);
    unvisited[0] = false;
    
    vector<pair<int, int>> pairs;
    for (size_t i = 1; i < data.nodes.size(); ++i) {
        if (data.nodes[i].delivery_id != 0 && unvisited[i]) {
            pairs.push_back({(int)i, data.nodes[i].delivery_id});
            unvisited[i] = false; unvisited[data.nodes[i].delivery_id] = false;
        }
    }
    
    for (auto p : pairs) {
        int u = p.first, v = p.second;
        bool inserted = false;
        
        for (size_t r_idx = 0; r_idx < routes.size(); ++r_idx) {
            for (size_t i = 0; i <= routes[r_idx].size(); ++i) {
                for (size_t j = i; j <= routes[r_idx].size() + 1; ++j) {
                    vector<int> cand = routes[r_idx];
                    cand.insert(cand.begin() + i, u);
                    cand.insert(cand.begin() + j, v);
                    if (update_route_times(cand, data)) {
                        routes[r_idx] = cand; inserted = true; break;
                    }
                }
                if (inserted) break;
            }
            if (inserted) break;
        }
        if (!inserted) {
            vector<int> new_r = {u, v};
            if (update_route_times(new_r, data)) routes.push_back(new_r);
        }
    }
    optimize_routes_pdp(routes, data);
    return routes;
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
    bool use_iter_limit = false;
    int max_iters = 10000;
    if (const char* env_iter = std::getenv("USE_ITER_LIMIT")) {
        if (std::string(env_iter) == "1") {
            use_iter_limit = true;
            if (const char* env_max = std::getenv("MAX_ITERS")) max_iters = stoi(env_max);
            cout << "[INFO] Engine running in ITERATION LIMIT mode (" << max_iters << " iters)" << endl;
        }
    }
    
    if (const char* env_p = std::getenv("USE_TRUNCATED_MATH")) {
        if (std::string(env_p) == "1") {
            PHYSICS_MULTIPLIER = 10.0;
            cout << "[INFO] Engine running in TRUNCATED physics mode (* 10.0)" << endl;
        }
    }
    string filename = (argc > 1) ? argv[1] : "lc101_mock.txt";
    double time_limit = (argc > 2) ? stod(argv[2]) : 300.0;
    VRPData data = load_vrp(filename);
    if (data.nodes.size() < 2) return 1;
    data.build_matrix();
    if (argc > 3) {
        string matrix_file = argv[3];
        load_external_matrix(matrix_file, data);
        cout << "[INFO] Loaded External Distance Matrix: " << matrix_file << endl;
    }

    vector<vector<int>> best_routes = pdp_initial_insertion(data);
    double best_total_cost = fitness_eval(best_routes, data);
    
    ofstream log_file("pdp_convergence.csv");
    log_file << "Time,Cost\n";
    log_file.close();

    auto start_time = high_resolution_clock::now();
    int max_threads = omp_get_max_threads();
    vector<double> thread_best_costs(max_threads, 1e18);
    vector<vector<vector<int>>> thread_best_routes(max_threads);
    
    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        mt19937 rng(42 + tid);
        global_alpha_cap = 100.0;
        vector<vector<int>> current_routes = best_routes;
        double current_fit = best_total_cost;
        vector<vector<int>> thread_best_r = best_routes;
        double thread_best_c = best_total_cost;
        int iter_count = 0;
        
        while (true) {
            double elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start_time).count() / 1000.0;
            double progress = 0.0;
            if (use_iter_limit) {
                progress = (double)iter_count / max_iters;
                if (iter_count >= max_iters) break;
            } else {
                progress = elapsed / time_limit;
                if (elapsed > time_limit) break;
            }
            
            if (tid == 0 && (iter_count % 100 == 0)) {
                ofstream log_file_app("pdp_convergence.csv", ios::app);
                log_file_app << elapsed << "," << best_total_cost << "\n";
                log_file_app.close();
            }
            iter_count++;
            
            global_alpha_cap = 1.0 + (999.0 * progress);
            
            vector<vector<int>> perturbed = current_routes;
            vector<pair<int, int>> removed_pairs;
            
            int ruin_type = rng() % 2; // 0 = Random, 1 = Shaw
            int num_remove = 2 + (rng() % 5);
            
            if (ruin_type == 0) {
                // Random Ruin
                for (int k = 0; k < num_remove; k++) {
                    vector<int> valid_r;
                    for (size_t i = 0; i < perturbed.size(); i++) if (!perturbed[i].empty()) valid_r.push_back(i);
                    if (valid_r.empty()) break;
                    
                    int r_idx = valid_r[rng() % valid_r.size()];
                    int p_idx = rng() % perturbed[r_idx].size();
                    int u = perturbed[r_idx][p_idx];
                    
                    int partner = 0;
                    if (data.nodes[u].pickup_id != 0) partner = data.nodes[u].pickup_id;
                    if (data.nodes[u].delivery_id != 0) partner = data.nodes[u].delivery_id;
                    
                    int pickup = (data.nodes[u].delivery_id != 0) ? u : partner;
                    int delivery = (data.nodes[u].pickup_id != 0) ? u : partner;
                    
                    if (pickup != 0 && delivery != 0) {
                        removed_pairs.push_back({pickup, delivery});
                        vector<int> new_r;
                        for (int n : perturbed[r_idx]) if (n != pickup && n != delivery) new_r.push_back(n);
                        perturbed[r_idx] = new_r;
                    }
                }
            } else {
                // Shaw Ruin (Spatio-Temporal)
                vector<int> valid_r;
                for (size_t i = 0; i < perturbed.size(); i++) if (!perturbed[i].empty()) valid_r.push_back(i);
                if (!valid_r.empty()) {
                    int r_idx = valid_r[rng() % valid_r.size()];
                    int p_idx = rng() % perturbed[r_idx].size();
                    int seed_u = perturbed[r_idx][p_idx];
                    
                    int partner = 0;
                    if (data.nodes[seed_u].pickup_id != 0) partner = data.nodes[seed_u].pickup_id;
                    if (data.nodes[seed_u].delivery_id != 0) partner = data.nodes[seed_u].delivery_id;
                    int seed_p = (data.nodes[seed_u].delivery_id != 0) ? seed_u : partner;
                    int seed_d = (data.nodes[seed_u].pickup_id != 0) ? seed_u : partner;
                    
                    if (seed_p != 0 && seed_d != 0) {
                        removed_pairs.push_back({seed_p, seed_d});
                        vector<int> new_r;
                        for (int n : perturbed[r_idx]) if (n != seed_p && n != seed_d) new_r.push_back(n);
                        perturbed[r_idx] = new_r;
                        
                        // Rip out physically close pairs
                        for (int k = 1; k < num_remove; k++) {
                            int target_p = 0, target_d = 0;
                            int best_dist = 1e9;
                            int target_r = -1;
                            
                            for (size_t i = 0; i < perturbed.size(); i++) {
                                for (int n : perturbed[i]) {
                                    if (data.nodes[n].delivery_id != 0) { // is a pickup
                                        int dist = data.dist(seed_p, n);
                                        if (dist < best_dist) {
                                            best_dist = dist;
                                            target_p = n; target_d = data.nodes[n].delivery_id;
                                            target_r = i;
                                        }
                                    }
                                }
                            }
                            if (target_p != 0 && target_d != 0) {
                                removed_pairs.push_back({target_p, target_d});
                                vector<int> nr;
                                for (int n : perturbed[target_r]) if (n != target_p && n != target_d) nr.push_back(n);
                                perturbed[target_r] = nr;
                            }
                        }
                    }
                }
            }
            
            // REPAIR: Regret-2 Insertion
            bool success = true;
            while (!removed_pairs.empty()) {
                long long max_regret = -1;
                int best_pair_idx = -1;
                int global_best_r = -1;
                int global_best_i = -1;
                int global_best_j = -1;
                
                for (size_t p_idx = 0; p_idx < removed_pairs.size(); p_idx++) {
                    int u = removed_pairs[p_idx].first;
                    int v = removed_pairs[p_idx].second;
                    
                    double best_c = 1e15;
                    double second_best_c = 1e15;
                    int best_r = -1, best_i = -1, best_j = -1;
                    
                    for (size_t r_idx = 0; r_idx < perturbed.size(); r_idx++) {
                        for (size_t i = 0; i <= perturbed[r_idx].size(); ++i) {
                            for (size_t j = i; j <= perturbed[r_idx].size() + 1; ++j) {
                                vector<int> cand = perturbed[r_idx];
                                cand.insert(cand.begin() + i, u);
                                cand.insert(cand.begin() + j, v);
                                
                                if (update_route_times(cand, data)) {
                                    double cost = evaluate_route(cand, data) - evaluate_route(perturbed[r_idx], data);
                                    if (cost < best_c) {
                                        second_best_c = best_c;
                                        best_c = cost;
                                        best_r = r_idx; best_i = i; best_j = j;
                                    } else if (cost < second_best_c) {
                                        second_best_c = cost;
                                    }
                                }
                            }
                        }
                    }
                    
                    double regret = (second_best_c == 1e15) ? 0 : (second_best_c - best_c);
                    if (regret > max_regret) {
                        max_regret = regret;
                        best_pair_idx = p_idx;
                        global_best_r = best_r; global_best_i = best_i; global_best_j = best_j;
                    }
                }
                
                if (best_pair_idx != -1) {
                    int u = removed_pairs[best_pair_idx].first;
                    int v = removed_pairs[best_pair_idx].second;
                    removed_pairs.erase(removed_pairs.begin() + best_pair_idx);
                    
                    if (global_best_r != -1) {
                        perturbed[global_best_r].insert(perturbed[global_best_r].begin() + global_best_i, u);
                        perturbed[global_best_r].insert(perturbed[global_best_r].begin() + global_best_j, v);
                    } else {
                        vector<int> new_r = {u, v};
                        if (update_route_times(new_r, data)) {
                            perturbed.push_back(new_r);
                        } else {
                            success = false; break;
                        }
                    }
                } else {
                    success = false; break;
                }
            }
            
            if (!success) continue;
            
            // Apply Deep Exploration (Local Search)
            optimize_routes_pdp(perturbed, data);
            
            double new_fit = fitness_eval(perturbed, data);
            int pert_veh = 0; for (const auto& r : perturbed) if (!r.empty()) pert_veh++;
            int cur_veh = 0; for (const auto& r : current_routes) if (!r.empty()) cur_veh++;
            
            if (pert_veh < cur_veh || (pert_veh == cur_veh && new_fit < current_fit)) {
                current_routes = perturbed;
                current_fit = new_fit;
                
                int best_veh = 0; for (const auto& r : thread_best_r) if (!r.empty()) best_veh++;
                if (pert_veh < best_veh || (pert_veh == best_veh && new_fit < thread_best_c)) {
                    thread_best_c = new_fit;
                    thread_best_r = current_routes;
                }
            } else {
                double diff = new_fit - current_fit;
                double temp = 15.0 * exp(-4.0 * progress);
                if (exp(-diff / temp) > (double)(rng() % 1000) / 1000.0) {
                    current_routes = perturbed;
                    current_fit = new_fit;
                }
            }
            iter_count++;
        }
        thread_best_costs[tid] = thread_best_c;
        thread_best_routes[tid] = thread_best_r;
    }
    
    for (int i = 0; i < max_threads; ++i) {
        if (thread_best_costs[i] < best_total_cost) {
            best_total_cost = thread_best_costs[i];
            best_routes = thread_best_routes[i];
        }
    }

    int print_idx = 0;
    for (size_t i = 0; i < best_routes.size(); ++i) {
        if (best_routes[i].empty()) continue;
        cout << "ROUTE " << print_idx++ << ": ";
        for (int node : best_routes[i]) cout << node << " ";
        cout << endl;
    }
    double real_cost = best_total_cost - (print_idx * PHYSICS_MULTIPLIER);
    cout << fixed << setprecision(2) << "FINAL_COST: " << real_cost << endl;

    return 0;
}
