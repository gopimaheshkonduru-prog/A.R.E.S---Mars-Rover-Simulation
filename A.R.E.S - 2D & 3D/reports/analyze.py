import pandas as pd
import matplotlib.pyplot as plt
import os


# =====================================================
# FILE SETTINGS
# =====================================================

INPUT_FILE = "reports/telemetry.csv"
OUTPUT_DIR = "reports"


# =====================================================
# LOAD TELEMETRY
# =====================================================

df = pd.read_csv(INPUT_FILE)

print("Telemetry loaded successfully.")

print(f"Samples: {len(df)}")

print(
    f"Duration: {df['time'].iloc[-1]:.2f} seconds"
)


# =====================================================
# CREATE OUTPUT DIRECTORY
# =====================================================

os.makedirs(
    OUTPUT_DIR,
    exist_ok=True
)


# =====================================================
# GRAPH 1 — POSITION TRAJECTORY
# =====================================================

plt.figure()

plt.plot(
    df["x"],
    df["y"]
)

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")

plt.title(
    "ARES Rover Trajectory"
)

plt.grid(True)

plt.savefig(
    os.path.join(
        OUTPUT_DIR,
        "position_trajectory.png"
    ),
    dpi=300,
    bbox_inches="tight"
)

plt.close()


# =====================================================
# GRAPH 2 — POSITION VS TIME
# =====================================================

plt.figure()

plt.plot(
    df["time"],
    df["x"],
    label="X Position"
)

plt.plot(
    df["time"],
    df["y"],
    label="Y Position"
)

plt.xlabel("Time (s)")
plt.ylabel("Position (m)")

plt.title(
    "Rover Position vs Time"
)

plt.legend()
plt.grid(True)

plt.savefig(
    os.path.join(
        OUTPUT_DIR,
        "position_vs_time.png"
    ),
    dpi=300,
    bbox_inches="tight"
)

plt.close()


# =====================================================
# GRAPH 3 — HEADING
# =====================================================

plt.figure()

plt.plot(
    df["time"],
    df["theta"]
)

plt.xlabel("Time (s)")
plt.ylabel("Heading (rad)")

plt.title(
    "Rover Heading vs Time"
)

plt.grid(True)

plt.savefig(
    os.path.join(
        OUTPUT_DIR,
        "heading_vs_time.png"
    ),
    dpi=300,
    bbox_inches="tight"
)

plt.close()


# =====================================================
# GRAPH 4 — WHEEL VELOCITIES
# =====================================================

plt.figure()

plt.plot(
    df["time"],
    df["left_wheel"],
    label="Left Wheel"
)

plt.plot(
    df["time"],
    df["right_wheel"],
    label="Right Wheel"
)

plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")

plt.title(
    "Wheel Velocities vs Time"
)

plt.legend()
plt.grid(True)

plt.savefig(
    os.path.join(
        OUTPUT_DIR,
        "wheel_velocity_vs_time.png"
    ),
    dpi=300,
    bbox_inches="tight"
)

plt.close()


# =====================================================
# GRAPH 5 — TARGET ERROR
# =====================================================

plt.figure()

plt.plot(
    df["time"],
    df["target_error"]
)

plt.xlabel("Time (s)")
plt.ylabel("Target Error (m)")

plt.title(
    "Target Tracking Error vs Time"
)

plt.grid(True)

plt.savefig(
    os.path.join(
        OUTPUT_DIR,
        "target_error_vs_time.png"
    ),
    dpi=300,
    bbox_inches="tight"
)

plt.close()


# =====================================================
# GRAPH 6 — BATTERY
# =====================================================

plt.figure()

plt.plot(
    df["time"],
    df["battery"]
)

plt.xlabel("Time (s)")
plt.ylabel("Battery (%)")

plt.title(
    "Battery Level vs Time"
)

plt.grid(True)

plt.savefig(
    os.path.join(
        OUTPUT_DIR,
        "battery_vs_time.png"
    ),
    dpi=300,
    bbox_inches="tight"
)

plt.close()


# =====================================================
# CALCULATE RESULTS
# =====================================================

total_time = df["time"].iloc[-1]

final_x = df["x"].iloc[-1]

final_y = df["y"].iloc[-1]

final_error = df["target_error"].iloc[-1]

mission_distance = df["mission_distance"].max()

initial_battery = df["battery"].iloc[0]

final_battery = df["battery"].iloc[-1]

battery_consumed = (
    initial_battery
    - final_battery
)

maximum_left_speed = (
    df["left_wheel"].abs().max()
)

maximum_right_speed = (
    df["right_wheel"].abs().max()
)

maximum_wheel_speed = max(
    maximum_left_speed,
    maximum_right_speed
)

maximum_waypoints = (
    df["waypoints"].max()
)


# =====================================================
# RESULTS SUMMARY
# =====================================================

print()

print("======================================")

print("ARES SIMULATION RESULTS")

print("======================================")

print(
    f"Total simulation time : "
    f"{total_time:.2f} s"
)

print(
    f"Final X position      : "
    f"{final_x:.2f} m"
)

print(
    f"Final Y position      : "
    f"{final_y:.2f} m"
)

print(
    f"Final target error    : "
    f"{final_error:.2f} m"
)

print(
    f"Mission distance      : "
    f"{mission_distance:.2f} m"
)

print(
    f"Initial battery       : "
    f"{initial_battery:.2f}%"
)

print(
    f"Final battery         : "
    f"{final_battery:.2f}%"
)

print(
    f"Battery consumed      : "
    f"{battery_consumed:.2f}%"
)

print(
    f"Maximum wheel speed   : "
    f"{maximum_wheel_speed:.2f} m/s"
)

print(
    f"Maximum A* waypoints  : "
    f"{maximum_waypoints}"
)

print(
    f"Telemetry samples     : "
    f"{len(df)}"
)

print("======================================")

print()

print(
    "Graphs saved successfully."
)

print(
    "Check the reports folder."
)