import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data = pd.read_csv("build/output.csv")

# 3D Trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(data["true_x"], data["true_y"], data["true_z"], label="True")
ax.plot(data["est_x"], data["est_y"], data["est_z"], label="Estimated")
ax.set_title("3D Receiver Trajectory")
ax.legend()

r = 6.371e6
ax.set_zlim(-r*0.01, r*0.01)

plt.show()

# Error vs Time
plt.figure()
plt.plot(data["time"], data["error"])
plt.title("Position Error vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.show()

# Visible Satellites
plt.figure()
plt.plot(data["time"], data["visible_sats"])
plt.title("Visible Satellites vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Satellite Count")
plt.show()