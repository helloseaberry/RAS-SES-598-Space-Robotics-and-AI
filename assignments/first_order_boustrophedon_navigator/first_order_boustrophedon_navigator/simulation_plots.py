import pandas as pd
import matplotlib.pyplot as plt

# Read data from simulation
data = pd.read_csv("ros2_turtlesim_data.csv")

# Cross-Track Error plot
plt.figure(figsize=(10, 4))
plt.plot(data["time"], data["/cross_track_error/data"], label="Cross-Track Error", color="cyan")
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.title("Cross-Track Error")
plt.legend()
plt.grid()
plt.savefig("cross_track_error.png")

# Trajectory Plot (X, Y)
plt.figure(figsize=(6, 6))
plt.plot(data["/turtle1/pose/x"], data["/turtle1/pose/y"], label="Trajectory", color="red")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Trajectory Plot")
plt.legend()
plt.grid()
plt.savefig("trajectory.png")

# Velocity Profiles Plot
plt.figure(figsize=(10, 4))
plt.plot(data["time"], data["/turtle1/cmd_vel/linear/x"], label="Linear Velocity", color="brown")
plt.plot(data["time"], data["/turtle1/cmd_vel/angular/z"], label="Angular Velocity", color="blue")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Linear and Angular Velocity")
plt.legend()
plt.grid()
plt.savefig("velocity.png")

plt.show()
