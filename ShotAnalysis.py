import numpy as np
import matplotlib.pyplot as plt

# Input parameters
L1 = 3.4  # Length of link 1
L2 = 1.5  # Length of link 2
L3 = 5.5  # Length of link 3
L4 = 4.5  # Length of link 4
EE = 11.932 # Length to end effector

omega2_values = np.linspace(1, 15, 1000)  # Angular velocity of input link in rad/s
theta_offset = np.deg2rad(60.28)  # Offset angle in radians
start_point = np.deg2rad(-45.31)
end_point = np.deg2rad(92.41)

K1 = L1 / L2
K2 = L1 / L4
K3 = (L2**2 - L3**2 + L4**2 + L1**2) / (2 * L2 * L4)
K4 = L1 / L3
K5 = (L4**2 - L1**2 - L2**2 - L3**2) / (2 * L2 * L3)

def projectile_motion(x0, y0, v0, theta, g=386.09):
    # Time of flight
    t_flight = 2 * v0 * np.sin(theta) / g
    
    # Time array
    t = np.linspace(0, 1, 1000)
    
    # x and y coordinates
    x = x0 + v0 * np.cos(theta) * t
    y = y0 + v0 * np.sin(theta) * t - 0.5 * g * t**2
    
    # Find index where y drops below 3.5 inches
    idx_below_height = np.where(y < 3.5)[0]
    if len(idx_below_height) > 0:
        idx_stop = idx_below_height[0]
        x = x[:idx_stop]
        y = y[:idx_stop]
    
    return x, y

def four_bar_mechanism(theta2, omega2):

    theta2 = theta2 - theta_offset
    
    # Calculate theta1 using cosine law
    A = - K1 - K2 * np.cos(theta2) + K3 + np.cos(theta2)
    B = - 2 * np.sin(theta2)
    C = K1 - K2 * np.cos(theta2) + K3 - np.cos(theta2)
    D = np.cos(theta2) - K1 + K4 * np.cos(theta2) + K5
    E = -2 * np.sin(theta2)
    F = K1 + (K4 - 1) * np.cos(theta2) + K5

    theta3 = 2 * np.arctan((-E - np.sqrt(E ** 2 - 4 * D * F)) / (2 * D))
    theta4 = 2 * np.arctan((-B - np.sqrt(B ** 2 - 4 * A * C)) / (2 * A))

    omega3 = L2 * omega2 * np.sin(theta4 - theta2) / (L3 * np.sin(theta3 - theta4))
    omega4 = L2 * omega2 * np.sin(theta2 - theta3) / (L4 * np.sin(theta4 - theta3))

    x_position = EE * np.cos(theta4 + theta_offset)
    y_position = EE * np.sin(theta4 + theta_offset)



    
    
    return theta3, theta4, omega3, omega4, x_position, y_position


# Input link angles (varying from start point to end point)
theta2_values = np.linspace(start_point, end_point, 1000)

launch_velocity = []


for omega2 in omega2_values:

    # Initialize lists to store results
    theta3_values = []
    theta4_values = []
    omega3_values = []
    omega4_values = []
    x_position_values = []
    y_position_values = []

    # Calculate and store results for each input angle
    for theta2 in theta2_values:
        theta3, theta4, omega3, omega4, x_position, y_position = four_bar_mechanism(theta2, omega2)
        theta3_values.append(theta3)
        theta4_values.append(theta4)
        omega3_values.append(omega3)
        omega4_values.append(omega4)
        x_position_values.append(x_position)
        y_position_values.append(y_position)

    launch_velocity.append(EE * abs(np.min(omega4_values)))

x_final_pos = []

# Projectile Motion Initial Conditions

x_start = -5.25
y_start = 13.67
trajectory_start = np.deg2rad(26.1)


for init_velocity in launch_velocity:
    x, y = projectile_motion(x_start, y_start, init_velocity, trajectory_start)
    x_final_pos.append(x[-1])

print()

target_distances = [7.5, 11.0, 14.5, 19]
index_distance = []

for distance in target_distances:
    closest_number = min(x_final_pos, key=lambda x: abs(x - distance))
    idx_closest = x_final_pos.index(closest_number)
    index_distance.append(idx_closest)

i = 0
for value in index_distance:
    print(value)
    x, y = projectile_motion(x_start, y_start, launch_velocity[value], trajectory_start)
    rpm = omega2_values[value] * 9.5492968
    rounded = str(round(rpm, 2))
    title = "Input RPM: " + rounded + "  Distance: " + str(target_distances[i]) + "(in)"
    i += 1
    plt.plot(x, y, label=title)
plt.title('Theoretical Projectile Motion for Targets')
plt.xlabel('Horizontal Distance (inches)')
plt.ylabel('Vertical Distance (inches)')
plt.grid(True)
plt.legend()
plt.show()




# Initialize lists to store results
theta3_values = []
theta4_values = []
omega3_values = []
omega4_values = []
x_position_values = []
y_position_values = []

omega2 = 1

# Calculate and store results for each input angle
for theta2 in theta2_values:
    theta3, theta4, omega3, omega4, x_position, y_position = four_bar_mechanism(theta2, omega2)
    theta3_values.append(theta3)
    theta4_values.append(theta4)
    omega3_values.append(omega3)
    omega4_values.append(omega4)
    x_position_values.append(x_position)
    y_position_values.append(y_position)

# Plotting
plt.figure(figsize=(12, 8))
plt.plot(theta2_values, theta4_values)
plt.title('Output Link Angle vs Input Link Angle')
plt.xlabel('Input Link Angle (rad)')
plt.ylabel('Link 4 Angle (rad)')
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 8))
plt.plot(theta2_values, omega4_values)
plt.title('Omega Out/Omega In vs Input Link Angle')
plt.xlabel('Input Link Angle (rad)')
plt.ylabel('Link 4 Angular Velocity / Input Angular Velocity')
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 8))
plt.plot(x_position_values, y_position_values)
plt.axis('equal')
plt.title('X-Y Plot of Center Point of End Effector from Ground of Link 4')
plt.xlabel('X-Plane (inches)')
plt.ylabel('Y-Plane (inches)')
plt.tight_layout()
plt.show()



