import csv
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

def load_bks():
    bks_data = {}
    
    # 1. Canonical benchmark repository source: results/VRPTW_Final_Results.csv
    vrptw_results = os.path.join(REPO_ROOT, 'results', 'VRPTW_Final_Results.csv')
    if os.path.exists(vrptw_results):
        with open(vrptw_results, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = row['Instance'].replace('.txt', '').replace('.sol', '').strip()
                bks_data[name] = {
                    'vehicles': float(row['BKS_Vehicles']),
                    'distance': float(row['BKS_Distance'])
                }
        if bks_data:
            return bks_data

    # 2. Master table fallback if present
    candidate_paths = [
        os.path.join(SCRIPT_DIR, 'Academic_BKS_Master_Table.csv'),
        os.path.join(REPO_ROOT, 'Academic_BKS_Master_Table.csv'),
        r'D:\businessss\Academic_BKS_Master_Table.csv',
    ]
    for p in candidate_paths:
        if os.path.exists(p):
            with open(p, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get('Variant') == 'VRPTW':
                        name = row['Instance'].replace('.txt', '').replace('.sol', '').strip()
                        bks_data[name] = {
                            'vehicles': float(row['BKS_Vehicles']),
                            'distance': float(row['BKS_Distance'])
                        }
            if bks_data:
                return bks_data

    raise FileNotFoundError("Could not locate BKS source file (VRPTW_Final_Results.csv or Academic_BKS_Master_Table.csv)")

def process_results(filename, bks_data):
    if not os.path.isabs(filename):
        filename = os.path.join(SCRIPT_DIR, filename)

    total_veh_gap = 0
    total_dist_gap = 0
    total_time = 0
    tied_bks_count = 0
    count = 0

    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get('Status') != 'SUCCESS':
                continue
                
            name = row['Instance'].replace('.txt', '').replace('.sol', '').strip()
            if name not in bks_data:
                continue
                
            bks_v = bks_data[name]['vehicles']
            bks_d = bks_data[name]['distance']
            
            v = float(row['Vehicles'])
            d = float(row['Total_Distance'])
            t = float(row['Time_Seconds'])
            
            v_gap = ((v - bks_v) / bks_v) * 100 if bks_v > 0 else 0
            d_gap = ((d - bks_d) / bks_d) * 100 if bks_d > 0 else 0
            
            total_veh_gap += v_gap
            total_dist_gap += d_gap
            total_time += t
            count += 1
            
            if abs(v - bks_v) < 0.01 and abs(d - bks_d) < 0.01:
                tied_bks_count += 1
                
    if count == 0:
        return 0, 0, 0, 0
        
    return tied_bks_count, total_veh_gap / count, total_dist_gap / count, total_time / count

def process_horizontal(vrptw_results_path, solomon_names):
    if not os.path.exists(vrptw_results_path):
        return None
    matched = []
    with open(vrptw_results_path, 'r') as f:
        for row in csv.DictReader(f):
            name = row['Instance'].replace('.sol', '').replace('.txt', '').strip()
            if name in solomon_names:
                matched.append(row)
    if not matched:
        return None
    tied = sum(1 for r in matched if abs(float(r['Vehicle_Gap(%)'])) < 0.01 and abs(float(r['Distance_Gap(%)'])) < 0.01)
    avg_vgap = sum(float(r['Vehicle_Gap(%)']) for r in matched) / len(matched)
    avg_dgap = sum(float(r['Distance_Gap(%)']) for r in matched) / len(matched)
    avg_t = sum(float(r['Time(s)']) for r in matched) / len(matched)
    return tied, avg_vgap, avg_dgap, avg_t

bks = load_bks()
v_tied, v_vgap, v_dgap, v_time = process_results('Vertical_Ablation_Results.csv', bks)
n_tied, n_vgap, n_dgap, n_time = process_results('NoHGS_Ablation_Results.csv', bks)

solomon_names = set()
vert_path = os.path.join(SCRIPT_DIR, 'Vertical_Ablation_Results.csv')
if os.path.exists(vert_path):
    with open(vert_path, 'r') as f:
        for row in csv.DictReader(f):
            solomon_names.add(row['Instance'].replace('.txt', '').replace('.sol', '').strip())

horiz_res = process_horizontal(os.path.join(REPO_ROOT, 'results', 'VRPTW_Final_Results.csv'), solomon_names)

print("=" * 80)
print("TABLE 7: ABLATION STUDY REPRODUCTION (56 SOLOMON BENCHMARK INSTANCES)")
print("=" * 80)
print(f"{'Configuration':<38} | {'Tied BKS':<8} | {'Avg Veh Gap':<11} | {'Avg Dist Gap':<12} | {'Avg Runtime'}")
print("-" * 80)
print(f"{'Vertical Acceleration-Mode (12 Th)':<38} | {v_tied:<8} | {v_vgap:>9.2f}% | {v_dgap:>10.2f}% | {v_time:.1f} s")
if horiz_res:
    h_tied, h_vgap, h_dgap, h_time = horiz_res
    print(f"{'Horizontal Batch-Mode (12 Inst/12 Th)':<38} | {h_tied:<8} | {h_vgap:>9.2f}% | {h_dgap:>10.2f}% | {h_time/60:.1f} min (Full Batch)")
print(f"{'No-HGS Mode (Pure ALNS / 12 Th)':<38} | {n_tied:<8} | {n_vgap:>9.2f}% | {n_dgap:>10.2f}% | {n_time:.1f} s")
print("=" * 80)
