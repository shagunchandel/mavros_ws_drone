import numpy as np
from itertools import combinations

# Thresholds
SPATIAL_THRESHOLD = 1.0  # meters
TIME_STEP = 0.1  # seconds

def trapezoidal_position(p0, p1, t0, t1, t):
    """Calculate position at time t using a trapezoidal velocity profile."""
    duration = t1 - t0
    if duration <= 0:
        return p1

    a_time = duration / 4
    d_time = duration / 4
    c_time = duration - a_time - d_time

    total_distance = np.linalg.norm(p1 - p0)
    if total_distance == 0:
        return p0

    direction = (p1 - p0) / total_distance
    a = total_distance / (a_time * (0.5 * a_time + c_time + 0.5 * d_time))  # acceleration

    t_rel = t - t0
    if t_rel <= 0:
        return p0
    elif t_rel < a_time:
        s = 0.5 * a * t_rel ** 2
    elif t_rel < a_time + c_time:
        s = 0.5 * a * a_time ** 2 + a * a_time * (t_rel - a_time)
    elif t_rel < duration:
        t_dec = t_rel - a_time - c_time
        s = 0.5 * a * a_time ** 2 + a * a_time * c_time + a * a_time * t_dec - 0.5 * a * (t_dec ** 2)
    else:
        return p1

    return p0 + direction * s

def get_position(drone_path, t):
    """Get drone position at time t using trapezoidal motion between waypoints."""
    for i in range(len(drone_path) - 1):
        p0, t0 = np.array(drone_path[i][0]), drone_path[i][1]
        p1, t1 = np.array(drone_path[i + 1][0]), drone_path[i + 1][1]
        if t0 <= t <= t1:
            return trapezoidal_position(p0, p1, t0, t1, t)
    return None

def detect_conflicts(drone_paths):
    """Detect conflicts among drones."""
    all_times = []
    for path in drone_paths:
        all_times.extend([t for _, t in path])
    t_min, t_max = min(all_times), max(all_times)

    conflicts = []
    t = t_min
    while t <= t_max:
        positions = []
        for path in drone_paths:
            pos = get_position(path, t)
            if pos is not None:
                positions.append(pos)
            else:
                positions.append(None)

        for (i, j) in combinations(range(len(drone_paths)), 2):
            pi, pj = positions[i], positions[j]
            if pi is not None and pj is not None:
                distance = np.linalg.norm(pi - pj)
                if distance < SPATIAL_THRESHOLD:
                    conflicts.append((round(t, 2), i, j, pi))
        t += TIME_STEP

    return conflicts

# Example usage
if __name__ == "__main__":
    num_drones = int(input("Enter number of drones: "))
    drone_paths = []

    for i in range(num_drones):
        print(f"Enter path for Drone {i} (format: x y z t, one per line, blank line to end):")
        path = []
        while True:
            line = input()
            if not line.strip():
                break
            x, y, z, t = map(float, line.strip().split())
            path.append(((x, y, z), t))
        drone_paths.append(path)

    conflicts = detect_conflicts(drone_paths)

    if conflicts:
        print("Conflicts detected:")
        for t, i, j, pos in conflicts:
            print(f"At time {t}s, Drone {i} and Drone {j} conflict at position {np.round(pos, 2)}")
    else:
        print("No conflicts detected.")
