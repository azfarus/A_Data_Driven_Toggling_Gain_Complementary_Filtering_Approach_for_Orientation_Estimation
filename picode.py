import subprocess
import signal
import sys
import time
import psutil
import csv
from datetime import datetime
import os
from utils.utils import count_folders

# Scripts to monitor
scripts = [
    "madg_filter_no_mqtt.py",
    "ekf_filter_no_mqtt.py",
    "aqua_filter_no_mqtt.py",
    "filter_no_mqtt.py",
]

# Script(s) to run but NOT monitor
background_scripts = [
    "turntable_encoder.py"
]

# Start processes
procs = [subprocess.Popen(["python3", script]) for script in scripts]
bg_procs = [subprocess.Popen(["python3", script]) for script in background_scripts]

# Track only monitored processes for CPU/mem
proc_info = {p: psutil.Process(p.pid) for p in procs}

# Create output folder
i = count_folders("data/", increment=True)
run_folder = f"run{i}"
os.makedirs("data/" + run_folder)

# Open CSV files
cpu_file = open(f"data/{run_folder}/quaternion_cpu_usage.csv", "w", newline="")
mem_file = open(f"data/{run_folder}/quaternion_memory_usage.csv", "w", newline="")

cpu_writer = csv.writer(cpu_file)
mem_writer = csv.writer(mem_file)

# Write headers (exclude turntable)
headers = ["timestamp"] + scripts
cpu_writer.writerow(headers)
mem_writer.writerow(headers)


def cleanup(signum, frame):
    print("Terminating child processes...")
    for p in procs + bg_procs:
        p.send_signal(signal.SIGINT)
    cpu_file.close()
    mem_file.close()
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# Warm-up CPU usage measurement
for ps_proc in proc_info.values():
    ps_proc.cpu_percent(interval=0.1)

try:
    while True:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")
        cpu_row = [timestamp]
        mem_row = [timestamp]

        for script, (p, ps_proc) in zip(scripts, proc_info.items()):
            if p.poll() is None:  # process running
                try:
                    cpu = ps_proc.cpu_percent(interval=0.1) / psutil.cpu_count(logical=True)
                    mem = ps_proc.memory_info().rss / (1024 * 1024)  # MB
                except psutil.NoSuchProcess:
                    cpu, mem = None, None
            else:
                cpu, mem = None, None

            cpu_row.append(cpu)
            mem_row.append(mem)

        cpu_writer.writerow(cpu_row)
        mem_writer.writerow(mem_row)

        # Flush immediately
        cpu_file.flush()
        mem_file.flush()

        # Sampling rate ~50 ms
        time.sleep(1.0)

except KeyboardInterrupt:
    cleanup(None, None)
