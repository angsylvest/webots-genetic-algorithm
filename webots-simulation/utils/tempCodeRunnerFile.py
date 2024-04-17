# Example positions and parameters
positions = {"1": (1.0, 2.0), "2": (2, 2)}
leader_key = "3"
leader_info = (4.0, 4.0, 0.5, 0.5)  # posx, posy, velx, vely

# Add leader's position to the positions dictionary
positions[leader_key] = (leader_info[0], leader_info[1])

# Create VectorField instance
ex_vector = VectorField(positions, leader_key, leader_info)

# Generate dispersal vectors
dispersal_vectors, cmd = ex_vector.generate_dispersal(center=(1.5, 2.0))

print(f'dispers cmd: {cmd}')

# Plotting
plt.quiver([pos[0] for pos in positions.values()], [pos[1] for pos in positions.values()], [disp[0] for disp in dispersal_vectors], [disp[1] for disp in dispersal_vectors], color='g', label='Dispersal Vectors')
plt.scatter([pos[0] for pos in positions.values()], [pos[1] for pos in positions.values()], color='black', label='Particles')
plt.legend()
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Particle Dispersal from Center')
plt.grid()
plt.show()