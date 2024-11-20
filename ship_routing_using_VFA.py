import numpy as np
import folium

# Define the vector field function with wave effect
def vector_field_with_wave(x, y, goal_x, goal_y, wave_speed_x):
    """Calculate the vector pointing towards the goal with wave effect in x-direction."""
    dx = goal_x - x
    dy = goal_y - y
    magnitude = np.sqrt(dx*2 + dy*2)
    
    if magnitude == 0:
        return 0, 0
    
    # Calculate the direction towards the goal
    unit_vector = np.array([dx / magnitude, dy / magnitude])
    
    # Add wave effect in the x-direction
    wave_effect = np.array([wave_speed_x, 0])  # Wave pushes in the x-direction
    
    # Combine the wave effect with the vector pointing towards the goal
    final_vector = unit_vector + wave_effect
    
    # Normalize the final vector
    final_magnitude = np.linalg.norm(final_vector)
    if final_magnitude != 0:
        final_vector = final_vector / final_magnitude

    return final_vector[0], final_vector[1]

# Path planning with wave effect
def plan_path_with_wave_effect(start, destination, step_size, max_steps, wave_speed_x):
    path = [start]
    current = np.array(start)
    goal = np.array(destination)

    for _ in range(max_steps):
        dx, dy = vector_field_with_wave(current[0], current[1], goal[0], goal[1], wave_speed_x)
        next_point = current + step_size * np.array([dx, dy])
        path.append(next_point.tolist())

        # Stop if close to the goal
        if np.linalg.norm(next_point - goal) < step_size:
            break
        current = next_point

    return path

# Interpolation function to make the path smoother
def smooth_path(latitudes, longitudes, interpolation_factor=5):
    smoothed_latitudes = []
    smoothed_longitudes = []
    for i in range(len(latitudes) - 1):
        smoothed_latitudes.append(latitudes[i])
        smoothed_longitudes.append(longitudes[i])

        # Interpolate between consecutive points
        for j in range(1, interpolation_factor):
            lat = latitudes[i] + (latitudes[i + 1] - latitudes[i]) * j / interpolation_factor
            lon = longitudes[i] + (longitudes[i + 1] - longitudes[i]) * j / interpolation_factor
            smoothed_latitudes.append(lat)
            smoothed_longitudes.append(lon)

    smoothed_latitudes.append(latitudes[-1])
    smoothed_longitudes.append(longitudes[-1])
    return smoothed_latitudes, smoothed_longitudes

# Function to visualize the route using Folium
def visualize_route_on_map(route_latitudes, route_longitudes, start_lat, start_lon, end_lat, end_lon):
    # Center of the map
    center_lat, center_lon = np.mean(route_latitudes), np.mean(route_longitudes)
    ship_map = folium.Map(location=[center_lat, center_lon], zoom_start=6)

    # Plot the waypoints (route points)
    for lat, lon in zip(route_latitudes, route_longitudes):
        folium.CircleMarker(location=[lat, lon], radius=3, color='blue', fill=True).add_to(ship_map)

    # Plot the start and destination points
    folium.Marker([start_lat, start_lon], popup="Start", icon=folium.Icon(color='green')).add_to(ship_map)
    folium.Marker([end_lat, end_lon], popup="Destination", icon=folium.Icon(color='red')).add_to(ship_map)

    # Draw the route
    folium.PolyLine(list(zip(route_latitudes, route_longitudes)), color='purple', weight=2.5, opacity=1).add_to(ship_map)

    # Save the map
    map_path = 'vector_field_with_wave_route.html'
    ship_map.save(map_path)
    print(f"Optimal ship route with wave effect plotted and saved to: {map_path}")

# User inputs for start and destination points
print("Enter the coordinates for the start and destination points:")
start_lat = float(input("Start Latitude: "))
start_lon = float(input("Start Longitude: "))
end_lat = float(input("Destination Latitude: "))
end_lon = float(input("Destination Longitude: "))

# Start and destination points
start = [start_lat, start_lon]
destination = [end_lat, end_lon]

# Path planning parameters
step_size = 0.05  # Distance to move in each step (degrees)
max_steps = 500  # Maximum number of steps
wave_speed_x = 0.1  # Wave speed in x-direction (affecting curvature)

# Generate the path with wave effect
path = plan_path_with_wave_effect(start, destination, step_size, max_steps, wave_speed_x)

# Split the path into latitudes and longitudes
route_latitudes, route_longitudes = zip(*path)

# Smooth the path
route_latitudes, route_longitudes = smooth_path(route_latitudes, route_longitudes, interpolation_factor=10)

# Visualize the route on a Folium map
visualize_route_on_map(route_latitudes, route_longitudes, start_lat, start_lon, end_lat, end_lon)