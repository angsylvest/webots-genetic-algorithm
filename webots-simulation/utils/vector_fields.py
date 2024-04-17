import math
import random
# import matplotlib.pyplot as plt # commented out so works on hpc

# can be done independently, assuming others will behave similarly 
# will be done an individual supervisor level (need to specify view range in ind supervisor)
class VectorField():
    
    def __init__(self, positions, leader_key, leader_info, collision_radius = 5.0):
        self.positions_dict = positions # is dict ["agent-id": (posx, posy)]
        self.leader_key = leader_key # leader 
        self.posx, self.posy, self.velx, self.vely = leader_info
        self.positions = list(self.positions_dict.values())
        self.reversed_positions = {value: key for key, value in self.positions_dict.items()}
        self.collision_radius = collision_radius
        self.k1 = 0.5
        self.k2 = 0.9

    # methods specificially for flocking 
    def calculate_relative_position(self, positions):
        n_particles = len(positions)
        relative_positions = [[[0.0, 0.0] for _ in range(n_particles)] for _ in range(n_particles)]
        
        for i in range(n_particles):
            for j in range(n_particles):
                if i != j:
                    distance = math.sqrt((positions[j][0] - positions[i][0])**2 + (positions[j][1] - positions[i][1])**2)
                    relative_positions[i][j][0] = (positions[j][0] - positions[i][0]) / (distance + 1e-6)
                    relative_positions[i][j][1] = (positions[j][1] - positions[i][1]) / (distance + 1e-6)
        
        return relative_positions

    def calculate_orientation(self, positions):
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

    def calculate_attraction(self):
        # uses positions dict, designated spot, spot_vel, k1, k2, collision raidus 

        n_particles = len(self.positions)
        attractions = [[0.0, 0.0] for _ in range(n_particles)]
        positions = self.positions
        goal_positions = [[0.0, 0.0] for _ in range(n_particles)]

        # positions dict -> positions list for calculations 
        designated_spot = [self.posx, self.posy, self.velx, self.vely]
        relative_positions = self.calculate_relative_position(positions)
        orientations = self.calculate_orientation(positions)
        spot_velocity = [self.velx, self.vely]
    
        goal_command = {}

        for i in range(n_particles):
            key_val = self.reversed_positions[positions[i]]
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
                repel_vector = [relative_positions[i][i][0] * self.k2, relative_positions[i][i][1] * self.k2]
                attractions[i] = repel_vector
                continue
            else:
                # Calculate attraction vector towards the spot position
                attraction_magnitude = self.k1 / (distance_to_spot + 1e-6)
                attraction_vector = [attraction_magnitude * direction_to_spot[0] / (distance_to_spot + 1e-6), attraction_magnitude * direction_to_spot[1] / (distance_to_spot + 1e-6)]
                # Update attractions[i] with attraction_vector
                attractions[i] = attraction_vector
                
            # Avoid collisions with other particles
            for j in range(n_particles):
                if i != j:
                    distance_to_other = math.sqrt((positions[i][0] - positions[j][0])**2 + (positions[i][1] - positions[j][1])**2)
                    if distance_to_other < self.collision_radius:
                        avoidance_vector = [relative_positions[i][j][0] * self.collision_radius, relative_positions[i][j][1] * self.collision_radius]
                        attraction_vector[0] += self.k2 * avoidance_vector[0]
                        attraction_vector[1] += self.k2 * avoidance_vector[1]
            
            attractions[i] = attraction_vector
            goal_command[key_val] = [positions[i][0] + attraction_vector[0], positions[i][1] + attraction_vector[1]]
    
        return attractions, goal_command


    def generate_dispersal(self, center=(0, 0), min_distance=0.1):
        dispersal_vectors = [[0.0, 0.0] for _ in range(len(self.positions))]
        positions = self.positions
        dispers_command = {}

        for i in range(len(positions)):
            key_val = self.reversed_positions[positions[i]]
            distance_to_center = math.sqrt((positions[i][0] - center[0])**2 + (positions[i][1] - center[1])**2)

            # Calculate the magnitude based on distance to the center
            magnitude = self.k2 * (self.collision_radius - distance_to_center)

            # Calculate the direction of the dispersal vector
            direction_vector = [(positions[i][0] - center[0]), (positions[i][1] - center[1])]
            direction_magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
            direction_vector = [direction_vector[0] / (direction_magnitude + 1e-6), direction_vector[1] / (direction_magnitude + 1e-6)]

            # Apply the magnitude to the direction to get the preliminary dispersal vector
            preliminary_dispersal_vector = [magnitude * direction_vector[0], magnitude * direction_vector[1]]

            # Check for collisions with other particles and adjust the dispersal vector accordingly
            for j in range(len(positions)):
                if i != j:
                    distance_to_other = math.sqrt((positions[i][0] - positions[j][0])**2 + (positions[i][1] - positions[j][1])**2)
                    if distance_to_other < min_distance:
                        avoidance_vector = [(positions[i][0] - positions[j][0]) * self.k2, (positions[i][1] - positions[j][1]) * self.k2]
                        preliminary_dispersal_vector[0] += avoidance_vector[0]
                        preliminary_dispersal_vector[1] += avoidance_vector[1]

            dispersal_vectors[i] = preliminary_dispersal_vector
            dispers_command[key_val] = [positions[i][0] + preliminary_dispersal_vector[0], positions[i][1] + preliminary_dispersal_vector[1]]

        return dispersal_vectors, dispers_command
    


# # ex positions: 
# positions = {"1": (1.0, 2.0), "2": (2,2)}
# leader_key = "3"
# leader_info = (4.0, 4.0, 0.5, 0.5) # posx, posy, velx, vely
# # Add leader's position to the positions dictionary
# positions[leader_key] = (leader_info[0], leader_info[1])
# ex_vector = VectorField(positions, leader_key, leader_info)
# attractions, cmd = ex_vector.calculate_attraction()
# print(f'attractions: {attractions} and cmd {cmd}')



# # Plotting
# plt.quiver([pos[0] for pos in positions.values()], [pos[1] for pos in positions.values()], [attr[0] for attr in attractions], [attr[1] for attr in attractions], color='r', label='Attraction Vectors')
# plt.quiver(leader_info[0], leader_info[1], leader_info[2], leader_info[3], color='blue', label='Designated Spot Vector')
# plt.scatter([pos[0] for pos in positions.values()], [pos[1] for pos in positions.values()], color='black', label='Particles')
# plt.legend()
# plt.xlabel('X-axis')
# plt.ylabel('Y-axis')
# plt.title('Particle Gravitation towards Designated Spot Vector with Orientation-Based Repulsion')
# plt.grid()
# plt.show()


# # Example positions and parameters
# positions = {"1": (1.0, 2.0), "2": (2, 2)}
# leader_key = "3"
# leader_info = (4.0, 4.0, 0.5, 0.5)  # posx, posy, velx, vely

# # Add leader's position to the positions dictionary
# positions[leader_key] = (leader_info[0], leader_info[1])

# # Create VectorField instance
# ex_vector = VectorField(positions, leader_key, leader_info)

# # Generate dispersal vectors
# dispersal_vectors, cmd = ex_vector.generate_dispersal(center=(1.5, 2.0))

# print(f'dispers cmd: {cmd}')

# # Plotting
# plt.quiver([pos[0] for pos in positions.values()], [pos[1] for pos in positions.values()], [disp[0] for disp in dispersal_vectors], [disp[1] for disp in dispersal_vectors], color='g', label='Dispersal Vectors')
# plt.scatter([pos[0] for pos in positions.values()], [pos[1] for pos in positions.values()], color='black', label='Particles')
# plt.legend()
# plt.xlabel('X-axis')
# plt.ylabel('Y-axis')
# plt.title('Particle Dispersal from Center')
# plt.grid()
# plt.show()