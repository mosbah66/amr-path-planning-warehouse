import matplotlib.pyplot as plt

# Hypothetical data: Replace with actual measurements after simulation
algorithms = ['A*', 'RRT*']
computation_time = [0.5, 2.3]  # seconds
path_length = [15.2, 17.8]  # meters

# Bar chart for computation time
plt.figure()
plt.bar(algorithms, computation_time, color=['blue', 'orange'])
plt.title('Computation Time Comparison')
plt.ylabel('Time (seconds)')
plt.grid(True)
plt.show()

# Bar chart for path length
plt.figure()
plt.bar(algorithms, path_length, color=['blue', 'orange'])
plt.title('Path Length Comparison')
plt.ylabel('Length (meters)')
plt.grid(True)
plt.show()

