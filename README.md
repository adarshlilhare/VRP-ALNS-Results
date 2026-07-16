# VRP-ALNS-Results: Unified Framework Logs

This repository contains the raw computational output logs (`.sol` files) and execution times (`.csv`) for the unified Adaptive Large Neighborhood Search (ALNS) C++ framework presented in our IEEE TechRxiv preprint.

To ensure full academic reproducibility, we have provided the evaluation traces for all **843 instances** across four major Vehicle Routing Problem (VRP) variants:
* **CVRP**: Capacitated Vehicle Routing Problem (Uchoa X-Instances)
* **VRPTW**: VRP with Time Windows (Solomon)
* **MDVRP**: Multi-Depot VRP (Cordeau P/PR)
* **PDPTW**: Pickup and Delivery Problem with Time Windows (Li & Lim)

---

## 🏆 Optimal and Record-Breaking Solutions

While our IEEE preprint provides average computational gaps, the framework successfully converged to absolute lexicographical optimality (0.00% gap for both Vehicle Count and Distance) or mathematically broke the official academic Best Known Solutions (BKS) on several heavily constrained datasets.

Below is the explicit list of the instances where the framework achieved these bounds under the strict 50-minute termination criterion.



### VRPTW (Lexicographical 0.00% Optimal)
The framework mathematically tied the official Best Known Solutions for both Fleet Size and Distance across the following 25 instances:
* `C101`, `C102`, `C103`, `C105`, `C106`, `C107`, `C108`, `C109`, `C1_2_1`, `C1_2_5`, `C1_2_6`, `C1_2_7`, `C1_6_1`, `C1_6_5`, `C1_8_5`, `C201`, `C202`, `C203`, `C205`, `C206`, `C207`, `C208`, `C2_2_1`, `R105`, `R206`

### PDPTW (Lexicographical 0.00% Optimal)
The framework tied the official Li & Lim Best Known Solutions across the following 46 instances, ensuring precedence constraints and time-windows were flawlessly enforced:
* `LC1_2_1`, `LC1_2_2`, `LC1_2_5`, `LC1_2_6`, `LC1_2_7`, `LC2_2_10`, `LC2_2_2`, `LC2_2_3`, `LC2_2_5`, `LR1_2_1`, `lc101`, `lc102`, `lc105`, `lc106`, `lc107`, `lc108`, `lc201`, `lc202`, `lc203`, `lc205`, `lc206`, `lc207`, `lc208`, `lr101`, `lr102`, `lr103`, `lr105`, `lr106`, `lr107`, `lr108`, `lr109`, `lr111`, `lr112`, `lr201`, `lr205`, `lr206`, `lr208`, `lr209`, `lrc101`, `lrc103`, `lrc104`, `lrc105`, `lrc107`, `lrc108`, `lrc204`, `lrc207`

---

## Repository Structure

* `CVRP_Final_Results.csv`: Mathematically verified gap analysis and execution times for CVRP.
* `VRPTW_Final_Results.csv`: Mathematically verified gap analysis and execution times for VRPTW.
* `MDVRP_Final_Results.csv`: Mathematically verified gap analysis and execution times for MDVRP.
* `PDPTW_Final_Results.csv`: Mathematically verified gap analysis and execution times for PDPTW.
* `/cvrp/`: Output `.sol` files for Uchoa CVRP instances.
* `/vrptw/`: Output `.sol` files for Solomon VRPTW instances.
* `/mdvrp/`: Output `.sol` files for Cordeau MDVRP instances.
* `/pdptw/`: Output `.sol` files for Li & Lim PDPTW instances.

