import math
import random
import matplotlib.pyplot as plt

def calculate_relative_position(positions):
    n_particles = len(positions)
    relative_positions = [[[0.0, 0.0] for _ in range(n_particles)] for _ in range(n_particles)]
    
    for i in range(n_particles):
        for j in range(n_particles):
            if i != j:
                distance = math.sqrt((positions[j][0] - positions[i][0])**2 + (positions[j][1] - positions[i][1])**2)
                relative_positions[i][j][0] = (positions[j][0] - positions[i][0]) / (distance + 1e-6)
                relative_positions[i][j][1] = (positions[j][1] - positions[i][1]) / (distance + 1e-6)
    
    return relative_positions

def calculate_orientation(positions):
    n_particles = len(positions)
    orientations = [[0.0, 0.0] for _ in range(n_particles)]
    
    for i in range(n_particles):
        if i < n_particles - 1:
            orientations[i][0] = positions[i + 1][0] - positions[i][0]
            orientations[i][1] = positions[i + 1][1] - positions[i][1]
        else:
            orientations[i][0] = positions[i - 1][0] - positions[i][0]
            orientations[i][1] = positions[i - 1][1] - positions[i][1]
        
        magnitude = math.sqrt(orientations[i][0]**2 + orientations[i][1]**2)
        orientations[i][0] /= magnitude + 1e-6
        orientations[i][1] /= magnitude + 1e-6
    
    return orientations

def calculate_attraction(positions, designated_spot, spot_velocity, k1, k2, collision_radius):
    n_particles = len(positions)
    attractions = [[0.0, 0.0] for _ in range(n_particles)]
    relative_positions = calculate_relative_position(positions)
    orientations = calculate_orientation(positions)
    
    for i in range(n_particles):
        distance_to_spot = math.sqrt((positions[i][0] - designated_spot[0])**2 + (positions[i][1] - designated_spot[1])**2)
        direction_to_spot = [designated_spot[0] - positions[i][0], designated_spot[1] - positions[i][1]]
        spot_direction = spot_velocity  # Spot's vector direction
        
        # Calculate the angle between the particle's orientation and spot's vector direction
        angle_cosine = (orientations[i][0] * spot_direction[0] + orientations[i][1] * spot_direction[1]) / ((math.sqrt(orientations[i][0]**2 + orientations[i][1]**2) * math.sqrt(spot_direction[0]**2 + spot_direction[1]**2)) + 1e-6)
        angle = math.acos(min(max(angle_cosine, -1.0), 1.0))
        
        # Check if particle is moving in the same general direction as the spot's vector
        moving_same_direction = angle < math.radians(45)  # Within 45-degree range
        
        if not moving_same_direction:
            # Repel the particle if not moving in the same general direction
            repel_vector = [relative_positions[i][i][0] * k2, relative_positions[i][i][1] * k2]
            attractions[i] = repel_vector
        else:
            # Calculate attraction vector towards the spot position
            attraction_magnitude = k1 / (distance_to_spot + 1e-6)
            attraction_vector = [attraction_magnitude * direction_to_spot[0] / (distance_to_spot + 1e-6), attraction_magnitude * direction_to_spot[1] / (distance_to_spot + 1e-6)]
        
        # Avoid collisions with other particles
        for j in range(n_particles):
            if i != j:
                distance_to_other = math.sqrt((positions[i][0] - positions[j][0])**2 + (positions[i][1] - positions[j][1])**2)
                if distance_to_other < collision_radius:
                    avoidance_vector = [relative_positions[i][j][0] * collision_radius, relative_positions[i][j][1] * collision_radius]
                    attraction_vector[0] += k2 * avoidance_vector[0]
                    attraction_vector[1] += k2 * avoidance_vector[1]
        
        attractions[i] = attraction_vector
    
    return attractions

# Example setup
n_particles = 10
positions = [[random.random() * 10, random.random() * 10] for _ in range(n_particles)]  # Random positions in a 10x10 grid
designated_spot = [5, 5, 0.5, 0.5]  # [Spot position x, Spot position y, Spot velocity x, Spot velocity y]
collision_radius = 0.5  # Radius for collision avoidance

# Parameters
k1 = 0.5  # Magnitude factor for attraction towards the spot
k2 = 0.9  # Reduction factor based on proximity to other particles

# Calculate attractions
attractions = calculate_attraction(positions, designated_spot, designated_spot[2:], k1, k2, collision_radius)

# Plotting
plt.quiver([pos[0] for pos in positions], [pos[1] for pos in positions], [attr[0] for attr in attractions], [attr[1] for attr in attractions], color='r', label='Attraction Vectors')
plt.quiver(designated_spot[0], designated_spot[1], designated_spot[2], designated_spot[3], color='blue', label='Designated Spot Vector')
plt.scatter([pos[0] for pos in positions], [pos[1] for pos in positions], color='black', label='Particles')
plt.legend()
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Particle Gravitation towards Designated Spot Vector with Orientation-Based Repulsion')
plt.grid()
plt.show()
