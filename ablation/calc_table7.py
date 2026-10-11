import csv
import sys

def load_bks(filename):
    bks_data = {}
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['Variant'] == 'VRPTW':
                # Map names like C101.txt to C101, or whatever they are
                name = row['Instance'].replace('.txt', '').replace('.sol', '').strip()
                bks_data[name] = {
                    'vehicles': float(row['BKS_Vehicles']),
                    'distance': float(row['BKS_Distance'])
                }
    return bks_data

def process_results(filename, bks_data):
    total_veh_gap = 0
    total_dist_gap = 0
    total_time = 0
    tied_bks_count = 0
    count = 0

    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['Status'] != 'SUCCESS':
                continue
                
            name = row['Instance'].replace('.txt', '').strip()
            if name not in bks_data:
                print(f"Warning: {name} not found in BKS data")
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

bks = load_bks('Academic_BKS_Master_Table.csv')
v_tied, v_vgap, v_dgap, v_time = process_results('Vertical_Ablation_Results.csv', bks)
n_tied, n_vgap, n_dgap, n_time = process_results('NoHGS_Ablation_Results.csv', bks)

print("--- VERTICAL ACCELERATION MODE ---")
print(f"Tied BKS: {v_tied}")
print(f"Avg Veh Gap: {v_vgap:.2f}%")
print(f"Avg Dist Gap: {v_dgap:.2f}%")
print(f"Avg Time: {v_time:.2f}s")

print("\n--- NO-HGS MODE ---")
print(f"Tied BKS: {n_tied}")
print(f"Avg Veh Gap: {n_vgap:.2f}%")
print(f"Avg Dist Gap: {n_dgap:.2f}%")
print(f"Avg Time: {n_time:.2f}s")
