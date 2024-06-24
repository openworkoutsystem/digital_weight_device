import pandas as pd
import numpy as np
import os
import math

# Constants
MAX_VOLTAGE_SYSTEM = 48  # Example max voltage of system
MAX_CURRENT_SYSTEM = 50  # Example max current of system
GEAR_RATIO = 4.6  # Example gear ratio
SPOOL_RADIUS = 40 / 1000  # Example spool radius in meters
IBYKV2NM = (3/2)/(math.sqrt(3)*(1/60)*2*math.pi)  # Conversion constant for motor torque
IBYKV2NM = 11  # Conversion constant for motor torque
MAX_MODULATION = 0.9  # Example maximum modulation of motor and controller

def calculate_motor_stats(motor_data):
    # Calculate additional stats
    motor_data['Rated Torque at Max Continuous Current (Nm)'] = IBYKV2NM * motor_data['max continuous current'] / motor_data['kv']
    motor_data['Peak Torque at Max Current (Nm)'] = IBYKV2NM * motor_data['max current'] / motor_data['kv']
    motor_data['No Load RPM'] = motor_data['kv'] * np.minimum(motor_data['max voltage'], MAX_VOLTAGE_SYSTEM * MAX_MODULATION)
    
    # Calculate linear force and speed without gear
    motor_data['Continuous Linear Force (N)'] = motor_data['Rated Torque at Max Continuous Current (Nm)'] / SPOOL_RADIUS
    motor_data['Peak Linear Force (N)'] = motor_data['Peak Torque at Max Current (Nm)'] / SPOOL_RADIUS
    motor_data['Linear Speed (m/s)'] = (motor_data['No Load RPM'] * 2 * 3.14159 * SPOOL_RADIUS) / 60
    motor_data['Linear Speed (ft/s)'] = motor_data['Linear Speed (m/s)'] * 3.28084
    motor_data['Continuous Linear Force (lb)'] = motor_data['Continuous Linear Force (N)'] * 0.224809
    motor_data['Peak Linear Force (lb)'] = motor_data['Peak Linear Force (N)'] * 0.224809
    
    # Calculate linear force and speed with gear
    motor_data['Rated Torque at Max Continuous Current with Gear (Nm)'] = motor_data['Rated Torque at Max Continuous Current (Nm)'] * GEAR_RATIO
    motor_data['Peak Torque at Max Current with Gear (Nm)'] = motor_data['Peak Torque at Max Current (Nm)'] * GEAR_RATIO
    motor_data['No Load RPM with Gear'] = motor_data['No Load RPM'] / GEAR_RATIO
    motor_data['Continuous Linear Force with Gear (N)'] = motor_data['Rated Torque at Max Continuous Current with Gear (Nm)'] / SPOOL_RADIUS
    motor_data['Peak Linear Force with Gear (N)'] = motor_data['Peak Torque at Max Current with Gear (Nm)'] / SPOOL_RADIUS
    motor_data['Linear Speed with Gear (m/s)'] = (motor_data['No Load RPM with Gear'] * 2 * 3.14159 * SPOOL_RADIUS) / 60
    motor_data['Linear Speed with Gear (ft/s)'] = motor_data['Linear Speed with Gear (m/s)'] * 3.28084
    motor_data['Continuous Linear Force with Gear (lb)'] = motor_data['Continuous Linear Force with Gear (N)'] * 0.224809
    motor_data['Peak Linear Force with Gear (lb)'] = motor_data['Peak Linear Force with Gear (N)'] * 0.224809
    
    return motor_data

def generate_markdown(motor_data):
    md_content = "# BLDC Motor Characteristics and Calculated Stats\n\n"
    for _, row in motor_data.iterrows():
        md_content += f"## Motor from {row['supplier']}\n"
        md_content += f"[Link to Motor]({row['link']})\n\n"
        md_content += "| Characteristic | Value |\n"
        md_content += "|----------------|-------|\n"
        md_content += f"| Weight | {row['weight']} grams |\n"
        md_content += f"| KV Rating | {row['kv']} RPM/V |\n"
        md_content += f"| Max Current | {row['max current']} A |\n"
        md_content += f"| Max Voltage | {row['max voltage']} V |\n"
        md_content += f"| Phase Resistance | {row['phase resistance']} ohms |\n"
        md_content += f"| Price | ${row['price']} |\n"
        md_content += f"| Max Continuous Current | {row['max continuous current']} A |\n"
        md_content += f"| Rated Torque at Max Continuous Current | {row['Rated Torque at Max Continuous Current (Nm)']:.2f} Nm |\n"
        md_content += f"| Peak Torque at Max Current | {row['Peak Torque at Max Current (Nm)']:.2f} Nm |\n"
        md_content += f"| No Load RPM | {row['No Load RPM']:.2f} RPM |\n"
        md_content += f"| Continuous Linear Force | {row['Continuous Linear Force (N)']:.2f} N ({row['Continuous Linear Force (lb)']:.2f} lb) |\n"
        md_content += f"| Peak Linear Force | {row['Peak Linear Force (N)']:.2f} N ({row['Peak Linear Force (lb)']:.2f} lb) |\n"
        md_content += f"| Linear Speed | {row['Linear Speed (m/s)']:.2f} m/s ({row['Linear Speed (ft/s)']:.2f} ft/s) |\n"
        md_content += f"| Rated Torque at Max Continuous Current with Gear | {row['Rated Torque at Max Continuous Current with Gear (Nm)']:.2f} Nm |\n"
        md_content += f"| Peak Torque at Max Current with Gear | {row['Peak Torque at Max Current with Gear (Nm)']:.2f} Nm |\n"
        md_content += f"| No Load RPM with Gear | {row['No Load RPM with Gear']:.2f} RPM |\n"
        md_content += f"| Continuous Linear Force with Gear | {row['Continuous Linear Force with Gear (N)']:.2f} N ({row['Continuous Linear Force with Gear (lb)']:.2f} lb) |\n"
        md_content += f"| Peak Linear Force with Gear | {row['Peak Linear Force with Gear (N)']:.2f} N ({row['Peak Linear Force with Gear (lb)']:.2f} lb) |\n"
        md_content += f"| Linear Speed with Gear | {row['Linear Speed with Gear (m/s)']:.2f} m/s ({row['Linear Speed with Gear (ft/s)']:.2f} ft/s) |\n"
        md_content += "\n"
        
    return md_content

def main():
    # Determine the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_path = os.path.join(script_dir, 'motor_data.csv')
    
    # Read the CSV file
    motor_data = pd.read_csv(csv_file_path, skipinitialspace=True)
    
    # Strip whitespace from headers
    motor_data.columns = [col.strip() for col in motor_data.columns]
    
    # Calculate motor stats
    motor_data = calculate_motor_stats(motor_data)
    
    # Generate markdown content
    markdown_content = generate_markdown(motor_data)
    
    # Write markdown content to a file
    md_file_path = os.path.join(script_dir, 'motor_stats.md')
    with open(md_file_path, 'w') as f:
        f.write(markdown_content)
        
    print("Markdown file 'motor_stats.md' generated successfully.")

if __name__ == "__main__":
    main()
