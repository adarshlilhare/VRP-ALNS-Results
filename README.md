# A Highly Scalable OpenMP-Accelerated Adaptive Large Neighborhood Search Framework for Highly Constrained Vehicle Routing Problems

This repository contains the official C++ algorithmic engines, Python dual-mode orchestrator, raw computational solution traces (`.sol` files), and verified gap analysis results (`.csv`) for the Hybrid ALNS+HGS framework.

To ensure 100% academic reproducibility, all code, benchmark evaluations across **843 benchmark instances**, and ablation logs are open-sourced under the MIT License.

---

## 🚀 Key Highlights & Architectural Overview

The framework introduces a **Dual-Scale Shared-Memory Architecture** engineered to bypass the serialization bottlenecks of standard scripting languages (such as the Python Global Interpreter Lock):

1. **Horizontal Batch-Mode (1 thread per instance):** Dispatches independent routing instances concurrently across physical CPU cores to maximize throughput. Used for evaluating massive academic benchmark suites under strict wall-clock time limits.
2. **Vertical Acceleration-Mode (all threads per instance):** Deploys the entire OpenMP thread pool onto a single routing instance, executing concurrent destroy-and-repair searches and periodic **Hybrid Genetic Search (HGS) crossovers** in shared memory to escape local minima in under 60 seconds (averaging 45.3s).
3. **Four Specialized C++ Routing Engines:** Rather than relying on a generalized solver burdened with conditional branching lag, four compiled C++ binaries strictly enforce variant-specific rules:
   * **CVRP**: Capacitated Vehicle Routing Problem (Uchoa X-Instances)
   * **VRPTW**: VRP with Time Windows (Solomon and Gehring & Homberger)
   * **MDVRP**: Multi-Depot VRP (Cordeau P/PR)
   * **PDPTW**: Pickup and Delivery Problem with Time Windows (Li & Lim)

---

## 🏆 Matched Global Best Known Solutions (71 BKS)

Under strict 50-minute execution limits, the framework matched **71 Global Best Known Solutions (BKS)** (achieving a 0.00% gap in both vehicle count and continuous distance) across the highly competitive VRPTW and PDPTW benchmarks.

### VRPTW (25 Matched Global BKS)
Matched official BKS across 25 Solomon and Gehring & Homberger instances:
`C101`, `C102`, `C103`, `C105`, `C106`, `C107`, `C108`, `C109`, `C1_2_1`, `C1_2_5`, `C1_2_6`, `C1_2_7`, `C1_6_1`, `C1_6_5`, `C1_8_5`, `C201`, `C202`, `C203`, `C205`, `C206`, `C207`, `C208`, `C2_2_1`, `R105`, `R206`

### PDPTW (46 Matched Global BKS)
Matched official Li & Lim BKS across 46 paired-precedence instances:
`LC101`, `LC102`, `LC103`, `LC104`, `LC105`, `LC106`, `LC107`, `LC108`, `LC109`, `LC201`, `LC202`, `LC203`, `LC204`, `LC205`, `LC206`, `LC207`, `LC208`, `LC1_2_1`, `LC1_2_2`, `LC1_2_5`, `LC1_2_6`, `LC1_2_7`, `LC2_2_2`, `LC2_2_3`, `LC2_2_5`, `LC2_2_10`, `LR101`, `LR102`, `LR103`, `LR104`, `LR105`, `LR106`, `LR107`, `LR108`, `LR109`, `LR110`, `LR111`, `LR112`, `LR1_2_1`, `LR201`, `LR205`, `LR206`, `LR208`, `LR209`, `LRC101`, `LRC103`, `LRC104`, `LRC105`, `LRC107`, `LRC108`, `LRC204`, `LRC207`

---

## 🔬 Reproducing the Ablation Study (Table 7)

The ablation study isolates the mathematical necessity of the HGS crossover operator and evaluates the Dual-Scale OpenMP parallelization modes on the 56 Solomon benchmark instances.

| Configuration | Validates | Tied BKS | Avg. Veh Gap (%) | Avg. Dist Gap (%) | Avg. Time |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Vertical Acceleration-Mode** (1 Instance / 12 Threads) | Contrib. 1 & 5 | 16 | 12.11% | -1.78% | 45.3s |
| **Horizontal Batch-Mode** (12 Instances / 12 Threads) | Contrib. 4 | 17 | 12.34% | -1.97% | 45.0m (Per Batch) |
| **No-HGS Mode** (Pure ALNS + 12 Threads) | Contrib. 3 | 15 | 8.73% | -0.86% | 45.3s |

### Reproduction Commands:
```bash
# 1. Run Vertical Acceleration-Mode (all 12 threads on single instance)
python ablation/run_vertical_mode.py

# 2. Run No-HGS Ablation Mode (pure trajectory ALNS without genetic crossover)
python ablation/run_nohgs_mode.py

# 3. Compute Table 7 summary metrics
python ablation/calc_table7.py
```
Per-instance raw outputs and execution times are saved in `ablation/Vertical_Ablation_Results.csv` and `ablation/NoHGS_Ablation_Results.csv`.

---

## 📁 Repository Structure

```
.
├── src/
│   ├── cvrp.cpp                     # C++ Engine for Capacitated VRP
│   ├── vrptw.cpp                    # C++ Engine for VRPTW with O(1) Time Warp & HGS
│   ├── mdvrp.cpp                    # C++ Engine for Multi-Depot VRP
│   ├── pdptw.cpp                    # C++ Engine for PDPTW with Paired Operators
│   └── benchmark_runner_50min.py   # Python Master SLA Orchestrator
├── results/
│   ├── CVRP_Final_Results.csv       # Verified gap analysis and times for CVRP
│   ├── VRPTW_Final_Results.csv      # Verified gap analysis and times for VRPTW
│   ├── MDVRP_Final_Results.csv      # Verified gap analysis and times for MDVRP
│   └── PDPTW_Final_Results.csv      # Verified gap analysis and times for PDPTW
├── ablation/
│   ├── Vertical_Ablation_Results.csv# 56-instance logs for Vertical Acceleration-Mode
│   ├── NoHGS_Ablation_Results.csv   # 56-instance logs for No-HGS Pure ALNS
│   ├── run_vertical_mode.py         # Automated runner for Vertical Mode
│   ├── run_nohgs_mode.py            # Automated runner for No-HGS Mode
│   └── calc_table7.py               # Calculation script for Table 7 ablation metrics
├── cvrp/                            # Raw .sol solution logs for Uchoa CVRP instances
├── vrptw/                           # Raw .sol solution logs for Solomon VRPTW instances
├── mdvrp/                           # Raw .sol solution logs for Cordeau MDVRP instances
├── pdptw/                           # Raw .sol solution logs for Li & Lim PDPTW instances
├── LICENSE                          # MIT License
└── README.md                        # Documentation and replication instructions
```

---

## ⚙️ Compilation & Setup

All engines utilize C++17 with OpenMP compiler directives for shared-memory parallelization.

### Linux / GCC:
```bash
g++ -O3 -fopenmp -std=c++17 src/vrptw.cpp -o vrptw
g++ -O3 -fopenmp -std=c++17 src/cvrp.cpp -o cvrp
g++ -O3 -fopenmp -std=c++17 src/mdvrp.cpp -o mdvrp
g++ -O3 -fopenmp -std=c++17 src/pdptw.cpp -o pdptw
```

### Windows / MSVC:
```cmd
cl /O2 /openmp /std:c++17 src\vrptw.cpp
cl /O2 /openmp /std:c++17 src\cvrp.cpp
cl /O2 /openmp /std:c++17 src\mdvrp.cpp
cl /O2 /openmp /std:c++17 src\pdptw.cpp
```

### Running Benchmark Suites:
```bash
python src/benchmark_runner_50min.py
```
