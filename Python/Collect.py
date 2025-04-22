# Import required libraries
import serial       # For UART communication with the microcontroller
import numpy as np  # For numerical operations on point cloud data
import open3d as o3d  # For 3D visualization capabilities
import math         # For trigonometric calculations

# Constants  MUST match the Keil firmware settings
STEPS = 3          # Number of full 360° rotations to perform (matches #define STEPS 11 in Keil)
STEP_DISTANCE = 1000 # Distance (mm) between scan planes (z-axis spacing)

def collect_tof_data():
    """
    Handles all communication with the Keil firmware to collect Time-of-Flight (ToF) sensor data.
    The firmware expects 's' commands and returns distance measurements in specific chunks.
    """
    # Initialize serial port - parameters must match Keil's UART_Init() configuration
    # COM5 is the typical Windows port - adjust for your system
    # 115200 baud rate matches the firmware's UART configuration
    # 30 second timeout prevents hanging if connection fails
    s = serial.Serial('COM5', 115200, timeout=30)
    print("Opening serial connection: " + s.name)
    
    # Clear any residual data in buffers to ensure clean communication
    s.reset_output_buffer()  # Clear output (transmit) buffer
    s.reset_input_buffer()   # Clear input (receive) buffer
    
    # Wait for user readiness - allows time to position the sensor properly
    input("Press Enter to start ToF measurement...")
    measurements = []  # Will store all distance measurements
    
    # Main measurement loop - matches the STEPS value in Keil
    for step in range(STEPS):
        # Send start command ('s') to initiate data collection for this rotation
        # This triggers the takeMeasurement() function in the Keil code
        s.write('s'.encode())  # encode() converts string to bytes for transmission
        
        # Each full rotation consists of 32 measurements (11.25° apart)
        # Data is received in 4 chunks of 8 measurements (matches sendMeasurements() in Keil)
        for chunk in range(4):
            # Request next data chunk - firmware waits for this 's' before sending
            s.write('s'.encode())
            
            # Read one line of data (one chunk of 8 measurements)
            # Firmware sends data as comma-separated values ending with newline
            data = s.readline().decode()  # decode() converts bytes to string
            print("Raw received data:", data)
            
            # Convert string to list of integers (split on commas, convert each value)
            # Example data format: "123,456,789,...,1011" (8 values)
            data = list(map(int, data.split(',')))
            
            # Process each of the 8 measurements in this chunk
            for measurement_num in range(8):
                try:
                    # Convert to float and store the distance measurement (in mm)
                    distance = float(data[measurement_num])
                    measurements.append(distance)
                    print(f"Step {step+1}, Chunk {chunk+1}, Measurement {measurement_num+1}: {distance} mm")
                except (ValueError, IndexError):
                    # Handle potential data corruption or incomplete transmissions
                    print(f"Error processing measurement - invalid data: {data[measurement_num] if measurement_num < len(data) else 'N/A'}")
    
    # Cleanly close the serial connection
    s.close()
    print("Closed serial connection: " + s.name)
    return measurements

def convert_to_xyz(measurements):
    """
    Converts polar coordinates (distance, angle) from the sensor to Cartesian (x,y,z) coordinates.
    The angular increments and z-spacing must match the Keil firmware's movement pattern.
    """
    points = []  # Will store the resulting 3D coordinates
    
    for i, distance in enumerate(measurements):
        # Calculate angle in radians (11.25° per measurement)
        # 11.25° comes from 360°/32 measurements per rotation (matches Keil's 64 steps per measurement)
        angle_deg = i * 11.25  # Degrees
        angle_rad = math.radians(angle_deg)  # Convert to radians for math functions
        
        # Polar to Cartesian conversion in x-y plane:
        # x = radial distance * cos(angle)
        # y = radial distance * sin(angle)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        
        # Z-coordinate increases by STEP_DISTANCE after each full rotation (32 measurements)
        # int(i/32) gives the current rotation number (0 for first 32 points, 1 for next 32, etc.)
        z = (int(i/32)) * STEP_DISTANCE
        
        points.append([x, y, z])
    
    return points

def save_and_visualize(points):
    """
    Saves the 3D point cloud data and creates an interactive visualization.
    The visualization connects points to show the scanning pattern from the firmware.
    """
    # Save points to XYZ file format (standard format for point clouds)
    with open("tof_radar.xyz", "w") as f:
        for point in points:
            # Format: "x y z" on each line
            f.write(f"{point[0]} {point[1]} {point[2]}\n")
    
    # Load the saved point cloud for visualization
    pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")
    print("Point cloud data (first 5 points):")
    print(np.asarray(pcd.points)[:5])  # Print first 5 points as sample
    
    # Create connections between points to visualize the scanning pattern
    lines = []  # Will store pairs of point indices to connect
    num_points = len(points)
    
    # Create connections between consecutive points in each rotation
    for i in range(num_points):
        # Connect to next point in current rotation (with wrap-around)
        # Formula: ((i+1)%32) gets the next point, + (32 * rotation_number) maintains current plane
        next_point = ((i+1)%32) + (32 * (i//32))
        lines.append([i, next_point])
        
        # Connect to corresponding point in next rotation (if exists)
        if i < len(points)-32:  # Don't connect from last rotation
            lines.append([i, i+32])
    
    # Create a LineSet object for visualization
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),  # Point coordinates
        lines=o3d.utility.Vector2iVector(lines)  # Line connections
    )
    
    print("Launching 3D visualization...")
    # Display the interactive 3D visualization
    o3d.visualization.draw_geometries([line_set])

if __name__ == "__main__":
    # Full data processing:
    
    # 1. Collect measurements by communicating with Keil firmware
    print("Starting data collection...")
    measurements = collect_tof_data()
    print(f"Collected {len(measurements)} measurements")
    
    # 2. Convert raw measurements to 3D Cartesian coordinates
    print("Converting to 3D coordinates...")
    points = convert_to_xyz(measurements)
    
    # 3. Save and visualize the resulting 3D scan
    print("Generating visualization...")
    save_and_visualize(points)