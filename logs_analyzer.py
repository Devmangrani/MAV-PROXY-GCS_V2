import os
import re

log_folder = "/home/skand/PycharmProjects/MAVproxyGCS/logs"
results = []

for filename in os.listdir(log_folder):
    if filename.endswith(".log"):
        filepath = os.path.join(log_folder, filename)
        with open(filepath, 'r') as f:
            content = f.read()

            # Extract PD parameters
            position_kp = re.search(r"Position: Kp=([\d.e-]+), Kd=([\d.]+)", content)
            velocity_kp = re.search(r"Velocity: Kp=([\d.e-]+), Kd=([\d.]+)", content)

            # Extract distances
            distances = list(map(float, re.findall(r"Distance to target: ([\d.]+)m", content)))
            avg_distance = sum(distances) / len(distances) if distances else 0

            results.append({
                "file": filename,
                "kp_position": position_kp.group(1) if position_kp else "N/A",
                "kd_position": position_kp.group(2) if position_kp else "N/A",
                "kp_velocity": velocity_kp.group(1) if velocity_kp else "N/A",
                "kd_velocity": velocity_kp.group(2) if velocity_kp else "N/A",
                "avg_distance": avg_distance
            })

# Find file with lowest average distance
best_file = min(results, key=lambda x: x["avg_distance"])
print("File with lowest average distance:", best_file["file"])
print("Parameters:", best_file)