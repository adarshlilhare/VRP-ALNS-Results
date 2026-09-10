import os
import math
import subprocess
import time
import concurrent.futures
import threading
import xml.etree.ElementTree as ET

TIMEOUT_SEC = 3000  # 50 minutes
MAX_CORES = 14

OUT_CVRP = "CVRP_45min_Results"
OUT_VRPTW = "VRPTW_45min_Results"
OUT_MDVRP = "MDVRP_45min_Results"
OUT_PDPTW = "PDPTW_45min_Results"
TIMES_FILE = "Compute_Times_45min.csv"

file_lock = threading.Lock()

def setup_directories():
    for d in [OUT_CVRP, OUT_VRPTW, OUT_MDVRP, OUT_PDPTW]:
        if not os.path.exists(d):
            os.makedirs(d)
    
    with open(TIMES_FILE, "w") as f:
        f.write("Instance,ProblemType,Compute_Time_Seconds\n")

def convert_xml_to_vrp(xml_path, out_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    name_node = root.find('.//name')
    name = name_node.text if name_node is not None else "Uchoa_Instance"
    cap_node = root.find('.//capacity')
    capacity = cap_node.text if cap_node is not None else "0"
    nodes = root.findall('.//node')
    requests = root.findall('.//request')
    
    demands = {'1': '0'} # Depot demand
    for req in requests:
        demands[req.get('node')] = req.find('quantity').text
        
    with open(out_path, 'w') as f:
        f.write(f"NAME : {name}\n\n")
        f.write("VEHICLE\n")
        f.write("NUMBER     CAPACITY\n")
        f.write(f"  100         {capacity}\n\n")
        f.write("CUSTOMER\n")
        f.write("CUST NO.  XCOORD.   YCOORD.    DEMAND   READY TIME  DUE DATE   SERVICE TIME\n\n")
        
        for node in nodes:
            nid = node.get('id')
            cx = node.find('cx').text
            cy = node.find('cy').text
            d = demands.get(nid, "0")
            num_id = int(nid) - 1
            f.write(f"{num_id} {cx} {cy} {d} 0 9999999 0\n")

def run_instance(filepath, engine, env, out_dir, problem_type):
    file_basename = os.path.basename(filepath)
    sol_filename = os.path.splitext(file_basename)[0] + ".sol"
    out_file = os.path.join(out_dir, sol_filename)
    
    if os.path.exists(out_file):
        print(f"[SKIPPED] {file_basename} already completed. Resuming...")
        return
    
    run_filepath = filepath
    temp_vrp = None
    if filepath.endswith(".xml"):
        temp_vrp = os.path.join(out_dir, file_basename + ".temp.vrp")
        convert_xml_to_vrp(filepath, temp_vrp)
        run_filepath = temp_vrp
        
    start = time.time()
    try:
        res = subprocess.run([engine, run_filepath, str(TIMEOUT_SEC)], capture_output=True, text=True, env=env, creationflags=0x08000000)
        elapsed = round(time.time() - start, 2)
        
        sol_content = []
        for line in res.stdout.split("\n"):
            line_str = str(line).strip()
            if line_str.startswith("ROUTE") or "FINAL_COST" in line_str:
                sol_content.append(line_str)
                
        if len(sol_content) > 0:
            with open(out_file, "w") as f:
                f.write("\n".join(sol_content) + "\n")
                
            with file_lock:
                with open(TIMES_FILE, "a") as f:
                    f.write(f"{sol_filename},{problem_type},{elapsed}\n")
                    
            print(f"[SUCCESS] {problem_type} Solved {file_basename} in {elapsed}s")
        else:
            print(f"[FAILED] {problem_type} {file_basename} returned empty or failed in {elapsed}s")
            
    except Exception as e:
        print(f"[ERROR] Failed {file_basename} - {e}")
    finally:
        if temp_vrp and os.path.exists(temp_vrp):
            os.remove(temp_vrp)

def run_mdvrp_instance(filepath, engine, env, out_dir, problem_type):
    file_basename = os.path.basename(filepath)
    sol_filename = os.path.splitext(file_basename)[0] + ".sol"
    out_file = os.path.join(out_dir, sol_filename)
    
    if os.path.exists(out_file):
        print(f"[SKIPPED] MDVRP {file_basename} already completed. Resuming...")
        return
        
    start = time.time()
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        header = lines[0].strip().split()
        num_customers = int(header[2])
        num_depots = int(header[3])
        capacity = int(lines[1].strip().split()[1])
        
        customers = []
        for i in range(1 + num_depots, 1 + num_depots + num_customers):
            parts = lines[i].strip().split()
            customers.append({'id': int(parts[0]), 'x': float(parts[1]), 'y': float(parts[2]), 'demand': int(parts[4])})
            
        depots = []
        depot_start_idx = 1 + num_depots + num_customers
        for i in range(depot_start_idx, depot_start_idx + num_depots):
            parts = lines[i].strip().split()
            depots.append({'id': int(parts[0]), 'x': float(parts[1]), 'y': float(parts[2])})
            
        clusters = {depot['id']: [] for depot in depots}
        for cust in customers:
            closest_depot = min(depots, key=lambda d: math.hypot(cust['x'] - d['x'], cust['y'] - d['y']))
            clusters[closest_depot['id']].append(cust)
        
        sol_content = []
        for depot in depots:
            cluster_custs = clusters[depot['id']]
            if not cluster_custs: continue
            
            temp_file = os.path.join(out_dir, f"{file_basename}_temp_depot_{depot['id']}.vrp")
            with open(temp_file, 'w') as f:
                f.write(f"NAME : DEPOT_{depot['id']}\n\n")
                f.write("VEHICLE\n")
                f.write("NUMBER     CAPACITY\n")
                f.write(f"  100         {capacity}\n\n")
                f.write("CUSTOMER\n")
                f.write("CUST NO.  XCOORD.   YCOORD.    DEMAND   READY TIME  DUE DATE   SERVICE TIME\n\n")
                f.write(f"0 {depot['x']} {depot['y']} 0 0 9999999 0\n")
                for cust in cluster_custs:
                    f.write(f"{cust['id']} {cust['x']} {cust['y']} {cust['demand']} 0 9999999 0\n")
                
            res = subprocess.run([engine, temp_file, str(TIMEOUT_SEC)], capture_output=True, text=True, env=env, creationflags=0x08000000)
            
            for line in res.stdout.split("\n"):
                line_str = str(line).strip()
                if line_str.startswith("ROUTE") or "FINAL_COST" in line_str:
                    sol_content.append(line_str)
                    
            if os.path.exists(temp_file):
                os.remove(temp_file)
                
        elapsed = round(time.time() - start, 2)
        
        if len(sol_content) > 0:
            with open(out_file, "w") as f:
                f.write("\n".join(sol_content) + "\n")
                
            with file_lock:
                with open(TIMES_FILE, "a") as f:
                    f.write(f"{sol_filename},{problem_type},{elapsed}\n")
            print(f"[SUCCESS] MDVRP Solved {file_basename} in {elapsed}s")
        else:
            print(f"[FAILED] MDVRP {file_basename} returned empty or failed in {elapsed}s")
            
    except Exception as e:
        print(f"[ERROR] Failed MDVRP {file_basename} - {e}")

if __name__ == '__main__':
    setup_directories()
    
    tasks = []
    
    env_cvrp = os.environ.copy()
    env_cvrp['USE_TRUNCATED_MATH'] = '1'
    env_cvrp['OMP_NUM_THREADS'] = '1'
    
    env_exact = os.environ.copy()
    env_exact['USE_TRUNCATED_MATH'] = '0'
    env_exact['OMP_NUM_THREADS'] = '1'
    
    # CVRP
    cvrp_dir = r"C:\Users\adars\Downloads\uchoa-et-al-2014"
    if os.path.exists(cvrp_dir):
        for root, dirs, files in os.walk(cvrp_dir):
            for file in files:
                if file.startswith("X-n") and (file.endswith(".vrp") or file.endswith(".xml")):
                    tasks.append((run_instance, os.path.join(root, file), "./cvrp.exe", env_cvrp, OUT_CVRP, "CVRP"))
                    
    # VRPTW
    vrptw_dir = r"D:\businessss\VRPTWController-master\Instances"
    if os.path.exists(vrptw_dir):
        for root, dirs, files in os.walk(vrptw_dir):
            for file in files:
                if file.endswith(".txt") and any(file.lower().startswith(p) for p in ["c1", "c2", "r1", "r2", "rc1", "rc2"]):
                    tasks.append((run_instance, os.path.join(root, file), "./vrptw.exe", env_exact, OUT_VRPTW, "VRPTW"))
                    
    # MDVRP
    mdvrp_dir = r"C:\Users\adars\Downloads\cordeau-al-1997-mdvrp"
    if os.path.exists(mdvrp_dir):
        for root, dirs, files in os.walk(mdvrp_dir):
            for file in files:
                if file.startswith("p") and file.endswith(".txt") and "readme" not in file.lower():
                    tasks.append((run_mdvrp_instance, os.path.join(root, file), "./mdvrp.exe", env_exact, OUT_MDVRP, "MDVRP"))
                    
    # PDPTW
    pdptw_dir = r"D:\pdp"
    if os.path.exists(pdptw_dir):
        for root, dirs, files in os.walk(pdptw_dir):
            for file in files:
                if file.endswith(".txt") and any(file.lower().startswith(p) for p in ["lc1", "lc2", "lr1", "lr2", "lrc1", "lrc2"]):
                    tasks.append((run_instance, os.path.join(root, file), "./pdptw.exe", env_exact, OUT_PDPTW, "PDPTW"))
                    
    print(f"Executing {len(tasks)} total tasks with 50-minute timeout on {MAX_CORES} cores...")
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=MAX_CORES) as executor:
        futures = []
        for task in tasks:
            func = task[0]
            args = task[1:]
            futures.append(executor.submit(func, *args))
            
        concurrent.futures.wait(futures)
        
    print("ALL 50-MINUTE BENCHMARKS COMPLETED!")
