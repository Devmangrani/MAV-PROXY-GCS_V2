import re
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import folium
from folium.plugins import AntPath, MarkerCluster, HeatMap, TimestampedGeoJson
from datetime import datetime
import matplotlib.dates as mdates
from matplotlib.colors import Normalize
from matplotlib import cm
import time


def create_output_directory(output_dir):
    """
    Create the output directory if it doesn't exist.
    Returns the absolute path to the directory.
    """
    # Convert to absolute path
    abs_output_dir = os.path.abspath(output_dir)

    # Create directory if it doesn't exist
    if not os.path.exists(abs_output_dir):
        try:
            os.makedirs(abs_output_dir)
            print(f"Created output directory: {abs_output_dir}")
        except Exception as e:
            print(f"Warning: Could not create output directory: {e}")
            # Fallback to current directory
            abs_output_dir = os.getcwd()
            print(f"Using current directory instead: {abs_output_dir}")

    return abs_output_dir


def find_latest_log_file(log_dir):
    """
    Find the most recent log file in the specified directory.

    Args:
        log_dir: Directory to search for log files

    Returns:
        Path to the most recent log file, or None if no log files found
    """
    # Ensure the directory exists
    if not os.path.exists(log_dir):
        print(f"Warning: Log directory '{log_dir}' not found.")
        return None

    # Look for log files
    log_pattern = os.path.join(log_dir, "*.log")
    log_files = glob.glob(log_pattern)

    # If no log files found, try common alternative extensions
    if not log_files:
        for ext in [".txt", ".log.*", "*sitl*", "*gps*"]:
            log_pattern = os.path.join(log_dir, f"*{ext}")
            log_files.extend(glob.glob(log_pattern))

    if not log_files:
        print(f"No log files found in '{log_dir}'")
        return None

    # Sort by modification time (newest first)
    log_files.sort(key=os.path.getmtime, reverse=True)

    latest_log = log_files[0]
    print(
        f"Found latest log file: {os.path.basename(latest_log)} (modified {datetime.fromtimestamp(os.path.getmtime(latest_log))})")

    return latest_log


def generate_timestamped_filename(base_name, extension):
    """
    Generate a filename with timestamp.

    Args:
        base_name: Base part of the filename
        extension: File extension (e.g., 'html', 'png')

    Returns:
        A string like 'base_name_20250304_123045.extension'
    """
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return f"{base_name}_{timestamp}.{extension}"


def extract_gps_data(log_content):
    """
    Extract GPS coordinates from the log file.
    """
    # Regular expressions for position data
    current_pos_pattern = r"Current Position: (\d+\.\d+), (\d+\.\d+)"
    target_pos_pattern = r"Target Position: (\d+\.\d+), (\d+\.\d+)"
    timestamp_pattern = r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})"
    distance_pattern = r"Distance to target: (\d+\.\d+)m"
    velocity_pattern = r"Velocity Vector: N=(-?\d+\.\d+) m/s, E=(-?\d+\.\d+) m/s"

    data = {
        'timestamps': [],
        'datetime_objects': [],  # Store actual datetime objects for easier manipulation
        'current_lat': [],
        'current_lon': [],
        'target_lat': [],
        'target_lon': [],
        'distances': [],
        'rel_timestamps': [],
        'velocity_north': [],
        'velocity_east': [],
        'velocity_total': []
    }

    lines = log_content.split('\n')
    current_timestamp = None
    start_time = None

    for line in lines:
        # Extract timestamp if present
        ts_match = re.search(timestamp_pattern, line)
        if ts_match:
            current_timestamp = ts_match.group(1)

            # Convert to datetime
            current_dt = datetime.strptime(current_timestamp, '%Y-%m-%d %H:%M:%S,%f')

            # Initialize start time if not set
            if start_time is None:
                start_time = current_dt

        if current_timestamp is None:
            continue

        # Extract current position
        current_match = re.search(current_pos_pattern, line)
        if current_match:
            data['timestamps'].append(current_timestamp)
            current_dt = datetime.strptime(current_timestamp, '%Y-%m-%d %H:%M:%S,%f')
            data['datetime_objects'].append(current_dt)
            data['rel_timestamps'].append((current_dt - start_time).total_seconds())

            lat = float(current_match.group(1))
            lon = float(current_match.group(2))
            data['current_lat'].append(lat)
            data['current_lon'].append(lon)

        # Extract target position (usually on the same line as current position)
        target_match = re.search(target_pos_pattern, line)
        if target_match:
            target_lat = float(target_match.group(1))
            target_lon = float(target_match.group(2))

            # Only append if we have a corresponding current position
            if len(data['current_lat']) > len(data['target_lat']):
                data['target_lat'].append(target_lat)
                data['target_lon'].append(target_lon)

        # Extract distance
        dist_match = re.search(distance_pattern, line)
        if dist_match:
            # Only append if we have a corresponding current position
            if len(data['current_lat']) > len(data['distances']):
                data['distances'].append(float(dist_match.group(1)))

        # Extract velocity if available
        vel_match = re.search(velocity_pattern, line)
        if vel_match:
            vel_north = float(vel_match.group(1))
            vel_east = float(vel_match.group(2))
            vel_total = np.sqrt(vel_north ** 2 + vel_east ** 2)

            # Only append if we have a corresponding current position
            if len(data['current_lat']) > len(data['velocity_north']):
                data['velocity_north'].append(vel_north)
                data['velocity_east'].append(vel_east)
                data['velocity_total'].append(vel_total)

    # Ensure all arrays have the same length
    min_length = min(len(data['current_lat']),
                     len(data['target_lat']) if data['target_lat'] else len(data['current_lat']),
                     len(data['distances']) if data['distances'] else len(data['current_lat']))

    for key in ['current_lat', 'current_lon', 'timestamps', 'datetime_objects', 'rel_timestamps']:
        data[key] = data[key][:min_length]

    if data['target_lat']:
        data['target_lat'] = data['target_lat'][:min_length]
        data['target_lon'] = data['target_lon'][:min_length]

    if data['distances']:
        data['distances'] = data['distances'][:min_length]

    if data['velocity_north']:
        vel_len = min(min_length, len(data['velocity_north']))
        data['velocity_north'] = data['velocity_north'][:vel_len]
        data['velocity_east'] = data['velocity_east'][:vel_len]
        data['velocity_total'] = data['velocity_total'][:vel_len]

    return data


def create_folium_map(data, log_file_name='unknown'):
    """
    Create an interactive Folium map with the drone's path and target positions.
    Also includes time markers and animations.

    Args:
        data: Dictionary containing GPS data
        log_file_name: Name of the original log file for reference
    """
    # Calculate center coordinates for the map
    center_lat = np.mean(data['current_lat'])
    center_lon = np.mean(data['current_lon'])

    # Create the map
    drone_map = folium.Map(location=[center_lat, center_lon], zoom_start=18,
                           tiles='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
                           attr='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors')

    # Add title with log file information
    title_html = f'''
        <h3 align="center" style="font-size:16px"><b>Drone GPS Path from {log_file_name}</b></h3>
        <h4 align="center" style="font-size:14px">Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</h4>
        <h4 align="center" style="font-size:14px">Flight Duration: {data['rel_timestamps'][-1]:.1f} seconds</h4>
    '''
    drone_map.get_root().html.add_child(folium.Element(title_html))

    # Create a feature group for the drone path
    path_group = folium.FeatureGroup(name='Drone Path')

    # Create a drone path with timestamps
    coordinates = list(zip(data['current_lat'], data['current_lon']))

    # Add AntPath for animated path visualization
    ant_path = AntPath(
        locations=coordinates,
        color='blue',
        weight=2.5,
        opacity=0.7,
        dash_array=[10, 20],
        pulse_color='red',
        delay=1000
    )
    path_group.add_child(ant_path)

    # Add start and end markers
    start_marker = folium.Marker(
        location=[data['current_lat'][0], data['current_lon'][0]],
        popup=f"Start: {data['timestamps'][0]}<br>Time: 0.0s<br>Latitude: {data['current_lat'][0]:.6f}<br>Longitude: {data['current_lon'][0]:.6f}",
        icon=folium.Icon(color='green', icon='play', prefix='fa')
    )
    path_group.add_child(start_marker)

    end_marker = folium.Marker(
        location=[data['current_lat'][-1], data['current_lon'][-1]],
        popup=f"End: {data['timestamps'][-1]}<br>Time: {data['rel_timestamps'][-1]:.1f}s<br>Latitude: {data['current_lat'][-1]:.6f}<br>Longitude: {data['current_lon'][-1]:.6f}",
        icon=folium.Icon(color='red', icon='stop', prefix='fa')
    )
    path_group.add_child(end_marker)

    # Add the path feature group to the map
    drone_map.add_child(path_group)

    # Create a feature group for target positions
    target_group = folium.FeatureGroup(name='Target Positions')

    # Add markers for target positions (every 10 points to avoid clutter)
    if data['target_lat']:
        target_indices = range(0, len(data['target_lat']), 10)
        for i in target_indices:
            target_marker = folium.CircleMarker(
                location=[data['target_lat'][i], data['target_lon'][i]],
                radius=3,
                color='orange',
                fill=True,
                fill_color='orange',
                fill_opacity=0.7,
                popup=f"Target at {data['timestamps'][i]}<br>Time: {data['rel_timestamps'][i]:.1f}s<br>Lat: {data['target_lat'][i]:.6f}<br>Lon: {data['target_lon'][i]:.6f}"
            )
            target_group.add_child(target_marker)

        # Add a marker for the final target
        final_target_marker = folium.Marker(
            location=[data['target_lat'][-1], data['target_lon'][-1]],
            popup=f"Final Target: {data['timestamps'][-1]}<br>Time: {data['rel_timestamps'][-1]:.1f}s<br>Latitude: {data['target_lat'][-1]:.6f}<br>Longitude: {data['target_lon'][-1]:.6f}",
            icon=folium.Icon(color='orange', icon='bullseye', prefix='fa')
        )
        target_group.add_child(final_target_marker)

    # Add the target feature group to the map
    drone_map.add_child(target_group)

    # Add a heatmap to show where the drone spent more time
    heat_data = [[lat, lon] for lat, lon in zip(data['current_lat'], data['current_lon'])]
    HeatMap(heat_data, radius=10, blur=15).add_to(folium.FeatureGroup(name='Heat Map').add_to(drone_map))

    # Add time markers group
    time_group = folium.FeatureGroup(name='Time Markers')

    # Add detailed time markers (every 5 seconds)
    interval = 5  # seconds
    for i in range(len(data['rel_timestamps'])):
        if i == 0 or i == len(data['rel_timestamps']) - 1:
            continue  # Skip first and last points as we already have markers

        # Add a marker every interval seconds
        if i > 0 and int(data['rel_timestamps'][i] / interval) > int(data['rel_timestamps'][i - 1] / interval):
            # Create popup content with detailed timing information
            popup_content = f"""
            <b>Time:</b> {data['rel_timestamps'][i]:.1f}s<br>
            <b>Timestamp:</b> {data['timestamps'][i]}<br>
            <b>Position:</b> {data['current_lat'][i]:.6f}, {data['current_lon'][i]:.6f}<br>
            """

            # Add distance info if available
            if i < len(data['distances']):
                popup_content += f"<b>Distance to target:</b> {data['distances'][i]:.2f}m<br>"

            # Add velocity info if available
            if data['velocity_total'] and i < len(data['velocity_total']):
                popup_content += f"<b>Velocity:</b> {data['velocity_total'][i]:.2f}m/s<br>"

            time_marker = folium.CircleMarker(
                location=[data['current_lat'][i], data['current_lon'][i]],
                radius=5,
                color='purple',
                fill=True,
                fill_color='purple',
                fill_opacity=0.7,
                popup=folium.Popup(popup_content, max_width=300)
            )
            time_group.add_child(time_marker)

    # Add the time feature group to the map
    drone_map.add_child(time_group)

    # Create TimestampedGeoJson for time-based animation
    features = []
    for i in range(len(data['current_lat'])):
        # Only add a fraction of points to avoid overloading the browser
        if i % 3 == 0:  # Include every 3rd point
            feature = {
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [data['current_lon'][i], data['current_lat'][i]]
                },
                'properties': {
                    'time': data['datetime_objects'][i].strftime('%Y-%m-%d %H:%M:%S'),
                    'popup': f"Time: {data['rel_timestamps'][i]:.1f}s",
                    'icon': 'circle',
                    'iconstyle': {
                        'fillColor': 'blue',
                        'fillOpacity': 0.6,
                        'stroke': 'true',
                        'radius': 7
                    }
                }
            }
            features.append(feature)

    # Only add TimestampedGeoJson if we have enough points
    if len(features) > 10:
        try:
            time_animation = TimestampedGeoJson(
                {'type': 'FeatureCollection', 'features': features},
                period='PT1S',  # One second per frame
                duration='PT0.5S',  # Half second transition
                auto_play=False,
                loop=False
            )
            time_animation.add_to(drone_map)
        except Exception as e:
            print(f"Warning: Could not create time animation: {e}")

    # Add layer control
    folium.LayerControl().add_to(drone_map)

    return drone_map


def create_interactive_plot(data, log_file_name='unknown'):
    """
    Create an interactive matplotlib plot showing the GPS coordinates with timing information.

    Args:
        data: Dictionary containing GPS data
        log_file_name: Name of the original log file for reference
    """
    # Create a figure with multiple subplots
    fig = plt.figure(figsize=(15, 12))

    # Add title with log file information
    fig.suptitle(f"GPS Analysis of {log_file_name}\nGenerated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                 fontsize=14, fontweight='bold')

    # Create a grid with 2 rows and 2 columns
    gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])

    # Main map plot (upper left)
    ax1 = fig.add_subplot(gs[0, 0])

    # Create a colormap based on timing
    norm = Normalize(vmin=min(data['rel_timestamps']), vmax=max(data['rel_timestamps']))
    cmap = plt.cm.viridis

    # Plot drone path with color based on time
    sc = ax1.scatter(data['current_lon'], data['current_lat'],
                     c=data['rel_timestamps'], cmap=cmap, norm=norm,
                     s=50, alpha=0.7, edgecolors='k', linewidths=0.5)

    # Plot drone path as a line
    ax1.plot(data['current_lon'], data['current_lat'], 'b-', alpha=0.3, linewidth=1.0)

    # Plot target positions if available
    if data['target_lat']:
        ax1.scatter(data['target_lon'], data['target_lat'],
                    color='red', marker='x', s=30, label='Target Positions')

    # Mark start and end points
    ax1.scatter(data['current_lon'][0], data['current_lat'][0],
                color='green', marker='o', s=100, label='Start')
    ax1.scatter(data['current_lon'][-1], data['current_lat'][-1],
                color='red', marker='o', s=100, label='End')

    # Add time labels at regular intervals
    interval = max(1, len(data['rel_timestamps']) // 10)  # Show about 10 labels
    for i in range(0, len(data['rel_timestamps']), interval):
        ax1.annotate(
            f"{data['rel_timestamps'][i]:.1f}s",
            (data['current_lon'][i], data['current_lat'][i]),
            xytext=(5, 5), textcoords='offset points',
            fontsize=8, color='black', backgroundcolor='white'
        )

    # Add colorbar for time
    cbar = plt.colorbar(sc, ax=ax1)
    cbar.set_label('Time (seconds)')

    # Add labels and title
    ax1.set_xlabel('Longitude')
    ax1.set_ylabel('Latitude')
    ax1.set_title('Drone GPS Path (Color = Time)')
    ax1.legend()
    ax1.grid(True)

    # Make the plot aspect ratio equal
    ax1.axis('equal')

    # Adjust the plot limits to add a small margin
    x_range = max(data['current_lon']) - min(data['current_lon'])
    y_range = max(data['current_lat']) - min(data['current_lat'])

    margin = 0.1  # 10% margin
    ax1.set_xlim(min(data['current_lon']) - x_range * margin, max(data['current_lon']) + x_range * margin)
    ax1.set_ylim(min(data['current_lat']) - y_range * margin, max(data['current_lat']) + y_range * margin)

    # Distance vs. Time plot (upper right)
    ax2 = fig.add_subplot(gs[0, 1])
    if data['distances']:
        ax2.plot(data['rel_timestamps'], data['distances'], 'b-', linewidth=2, label='Distance to Target')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Distance (meters)')
        ax2.set_title('Distance to Target vs. Time')
        ax2.grid(True)

        # Add velocity on second y-axis if available
        if data['velocity_total']:
            ax2_twin = ax2.twinx()
            vel_times = data['rel_timestamps'][:len(data['velocity_total'])]
            ax2_twin.plot(vel_times, data['velocity_total'], 'r-', linewidth=1.5, label='Velocity')
            ax2_twin.set_ylabel('Velocity (m/s)', color='r')
            ax2_twin.tick_params(axis='y', labelcolor='r')

            # Add combined legend
            lines1, labels1 = ax2.get_legend_handles_labels()
            lines2, labels2 = ax2_twin.get_legend_handles_labels()
            ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        else:
            ax2.legend()

    # Latitude vs. Time plot (lower left)
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(data['rel_timestamps'], data['current_lat'], 'g-', linewidth=2, label='Latitude')
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Latitude')
    ax3.set_title('Latitude vs. Time')
    ax3.grid(True)

    # Mark time intervals
    interval = max(1, len(data['rel_timestamps']) // 20)  # Show about 20 markers
    for i in range(0, len(data['rel_timestamps']), interval):
        ax3.axvline(x=data['rel_timestamps'][i], color='gray', linestyle='--', alpha=0.3)

    # Longitude vs. Time plot (lower right)
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(data['rel_timestamps'], data['current_lon'], 'm-', linewidth=2, label='Longitude')
    ax4.set_xlabel('Time (seconds)')
    ax4.set_ylabel('Longitude')
    ax4.set_title('Longitude vs. Time')
    ax4.grid(True)

    # Mark same time intervals
    for i in range(0, len(data['rel_timestamps']), interval):
        ax4.axvline(x=data['rel_timestamps'][i], color='gray', linestyle='--', alpha=0.3)

    # Adjust layout
    plt.tight_layout()

    return fig


def main():
    """
    Main function to process logs and save visualizations without display
    """
    # Configuration
    logs_directory = 'logs'
    output_directory = 'gps_analysis_results'

    try:
        # Setup output directory
        output_dir = create_output_directory(output_directory)
        log_file_path = find_latest_log_file(logs_directory)

        if not log_file_path:
            print("No log files found. Exiting.")
            return

        # Process log file
        with open(log_file_path, 'r') as f:
            log_content = f.read()

        data = extract_gps_data(log_content)
        if not data['current_lat']:
            print("No valid GPS data found. Exiting.")
            return

        # Generate filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_filename = f"drone_path_{timestamp}.html"
        plot_filename = f"analysis_{timestamp}.png"
        summary_filename = f"summary_{timestamp}.txt"

        # Save visualizations silently
        # 1. Save Folium map
        try:
            drone_map = create_folium_map(data, os.path.basename(log_file_path))
            map_path = os.path.join(output_dir, map_filename)
            drone_map.save(map_path)
            print(f"Map saved to: {map_path}")
        except Exception as e:
            print(f"Failed to save map: {str(e)}")

        # 2. Save analysis plot
        try:
            fig = create_interactive_plot(data, os.path.basename(log_file_path))
            plot_path = os.path.join(output_dir, plot_filename)
            fig.savefig(plot_path, dpi=300, bbox_inches='tight')
            plt.close(fig)  # Important: release memory
            print(f"Analysis plot saved to: {plot_path}")
        except Exception as e:
            print(f"Failed to save plot: {str(e)}")

        # 3. Save text summary
        try:
            summary_path = os.path.join(output_dir, summary_filename)
            with open(summary_path, 'w') as f:
                f.write(f"GPS Analysis Summary - {timestamp}\n")
                f.write(f"Log file: {os.path.basename(log_file_path)}\n")
                f.write(f"Total points: {len(data['current_lat'])}\n")
                f.write(f"Duration: {data['rel_timestamps'][-1]:.1f} seconds\n")

                if data['distances']:
                    f.write(f"\nDistance Statistics:\n")
                    f.write(f"Average: {np.mean(data['distances']):.2f}m\n")
                    f.write(f"Minimum: {np.min(data['distances']):.2f}m\n")
                    f.write(f"Maximum: {np.max(data['distances']):.2f}m\n")

                if data['velocity_total']:
                    f.write(f"\nVelocity Statistics:\n")
                    f.write(f"Average: {np.mean(data['velocity_total']):.2f}m/s\n")
                    f.write(f"Maximum: {np.max(data['velocity_total']):.2f}m/s\n")

            print(f"Summary saved to: {summary_path}")
        except Exception as e:
            print(f"Failed to save summary: {str(e)}")

        print("\nAnalysis complete. All outputs saved to:", output_dir)

    except Exception as e:
        print(f"Fatal error: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()