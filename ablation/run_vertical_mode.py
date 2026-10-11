import os
import subprocess
import time

DATA_DIR = r"D:\businessss\VRPTWController-master\Instances"
ENGINE = r"D:\businessss\vrptw_full.exe"
TIMEOUT_SEC = 45 # Sub-minute time limit for Vertical Acceleration
THREADS = "12"
OUT_CSV = r"D:\businessss\Vertical_Ablation_Results.csv"
OUT_LOG_DIR = r"D:\businessss\Vertical_Ablation_Logs"

if not os.path.exists(OUT_LOG_DIR):
    os.makedirs(OUT_LOG_DIR)

instances = []
if os.path.exists(DATA_DIR):
    for root, dirs, files in os.walk(DATA_DIR):
        for file in files:
            if file.endswith(".txt") and "_" not in file and any(file.lower().startswith(p) for p in ["c1", "c2", "r1", "r2", "rc1", "rc2"]):
                instances.append(os.path.join(root, file))

print(f"Found {len(instances)} Solomon instances. Starting Vertical Acceleration Mode...")

with open(OUT_CSV, "w") as f:
    f.write("Instance,Time_Seconds,Vehicles,Total_Distance,Status\n")

env = os.environ.copy()
env["OMP_NUM_THREADS"] = THREADS
env["USE_TRUNCATED_MATH"] = "0"

for idx, inst in enumerate(instances):
    name = os.path.basename(inst)
    print(f"[{idx+1}/{len(instances)}] Running {name} on {THREADS} threads...")
    start = time.time()
    status = "SUCCESS"
    cost = "N/A"
    vehicles = 0
    sol_content = []
    
    try:
        res = subprocess.run([ENGINE, inst, str(TIMEOUT_SEC)], capture_output=True, text=True, env=env, timeout=TIMEOUT_SEC + 5)
        
        for line in res.stdout.split("\n"):
            line_str = line.strip()
            if line_str.startswith("ROUTE"):
                sol_content.append(line_str)
                vehicles += 1
            elif "FINAL_COST" in line_str:
                sol_content.append(line_str)
                cost = line_str.split(":")[1].strip()
                
        sol_filename = os.path.join(OUT_LOG_DIR, name.replace(".txt", ".sol"))
        with open(sol_filename, "w") as f:
            f.write("\n".join(sol_content) + "\n")
            
    except subprocess.TimeoutExpired:
        status = "TIMEOUT"
    except Exception as e:
        status = f"ERROR"
        
    elapsed = round(time.time() - start, 2)
    print(f"  -> Finished {name} in {elapsed}s | Cost: {cost} | Vehicles: {vehicles} [{status}]")
    
    with open(OUT_CSV, "a") as f:
        f.write(f"{name},{elapsed},{vehicles},{cost},{status}\n")

print(f"All {len(instances)} instances completed! Results saved to {OUT_CSV} and logs in {OUT_LOG_DIR}")
