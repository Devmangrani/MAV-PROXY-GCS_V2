import re
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RectangleSelector, TextBox
import matplotlib.dates as mdates
from datetime import datetime
import matplotlib as mpl
from matplotlib.ticker import MultipleLocator

# Set style for better visualization
plt.style.use('ggplot')
mpl.rcParams['figure.figsize'] = (14, 10)
mpl.rcParams['font.size'] = 12


def parse_log_file(log_content):
    """
    Parse the log file content to extract relevant data.
    """
    data = {
        'timestamps': [],
        'rel_timestamps': [],
        'pitch': [],
        'roll': [],
        'pitch_acc': [],
        'roll_acc': [],
        'pitch_correction': [],
        'roll_correction': [],
        'corrected_pitch': [],  # New field for corrected pitch values
        'corrected_roll': [],  # New field for corrected roll values
        'distance': [],
        'velocity_north': [],
        'velocity_east': [],
        'event_markers': []  # Store important events for highlighting
    }

    # Regular expressions for different data patterns
    timestamp_pattern = r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})'
    attitude_pattern = r'Attitude: Pitch: (-?\d+\.\d+)° \(acc: (-?\d+\.\d+)°\), Roll: (-?\d+\.\d+)° \(acc: (-?\d+\.\d+)°\)'
    correction_pattern = r'Attitude correction applied - Pitch: (-?\d+\.\d+)° \(corr: (-?\d+\.\d+)\), Roll: (-?\d+\.\d+)° \(corr: (-?\d+\.\d+)\)'
    distance_pattern = r'Distance to target: (\d+\.\d+)m'
    velocity_pattern = r'Velocity Vector: N=(-?\d+\.\d+) m/s, E=(-?\d+\.\d+) m/s'

    lines = log_content.split('\n')
    current_timestamp = None
    start_time = None

    for line in lines:
        # Extract timestamp
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

        # Extract attitude data
        att_match = re.search(attitude_pattern, line)
        if att_match:
            rel_time = (datetime.strptime(current_timestamp, '%Y-%m-%d %H:%M:%S,%f') - start_time).total_seconds()
            data['timestamps'].append(current_timestamp)
            data['rel_timestamps'].append(rel_time)

            pitch = float(att_match.group(1))
            pitch_acc = float(att_match.group(2))
            roll = float(att_match.group(3))
            roll_acc = float(att_match.group(4))

            data['pitch'].append(pitch)
            data['pitch_acc'].append(pitch_acc)
            data['roll'].append(roll)
            data['roll_acc'].append(roll_acc)

            # Placeholder for corrected values, will be updated if correction is found
            data['corrected_pitch'].append(pitch)
            data['corrected_roll'].append(roll)

            # Add event marker if pitch changes significantly
            if len(data['pitch']) > 1 and abs(pitch - data['pitch'][-2]) > 5:
                data['event_markers'].append({
                    'time': rel_time,
                    'type': 'pitch_change',
                    'value': pitch,
                    'description': f"Pitch changed to {pitch:.1f}°"
                })

        # Extract correction data
        corr_match = re.search(correction_pattern, line)
        if corr_match:
            # If correction is found for the same timestamp, update the corrected values
            if current_timestamp in data['timestamps'] and data['timestamps'][-1] == current_timestamp:
                pitch_corr = float(corr_match.group(2))
                roll_corr = float(corr_match.group(4))
                data['pitch_correction'].append(pitch_corr)
                data['roll_correction'].append(roll_corr)

                # Calculate corrected values by adding correction to raw values
                corrected_pitch = data['pitch'][-1] + pitch_corr
                corrected_roll = data['roll'][-1] + roll_corr
                data['corrected_pitch'][-1] = corrected_pitch
                data['corrected_roll'][-1] = corrected_roll

                # Add event marker for significant corrections
                rel_time = (datetime.strptime(current_timestamp, '%Y-%m-%d %H:%M:%S,%f') - start_time).total_seconds()
                if abs(pitch_corr) > 10:
                    data['event_markers'].append({
                        'time': rel_time,
                        'type': 'correction',
                        'value': pitch_corr,
                        'description': f"Correction: {pitch_corr:.1f}°"
                    })
            else:
                # New timestamp with correction data
                rel_time = (datetime.strptime(current_timestamp, '%Y-%m-%d %H:%M:%S,%f') - start_time).total_seconds()
                data['timestamps'].append(current_timestamp)
                data['rel_timestamps'].append(rel_time)

                # Extract raw pitch and roll from the correction line if available
                raw_pitch = float(corr_match.group(1))
                raw_roll = float(corr_match.group(3))
                pitch_corr = float(corr_match.group(2))
                roll_corr = float(corr_match.group(4))

                # If we have a previous attitude, use that for raw values
                if data['pitch']:
                    raw_pitch = data['pitch'][-1]
                    raw_roll = data['roll'][-1]
                    pitch_acc = data['pitch_acc'][-1]
                    roll_acc = data['roll_acc'][-1]
                else:
                    # If no previous attitude, make a best guess
                    raw_pitch = float(corr_match.group(1)) - float(corr_match.group(2))
                    raw_roll = float(corr_match.group(3)) - float(corr_match.group(4))
                    pitch_acc = 0.0
                    roll_acc = 0.0

                data['pitch'].append(raw_pitch)
                data['roll'].append(raw_roll)
                data['pitch_acc'].append(pitch_acc)
                data['roll_acc'].append(roll_acc)
                data['pitch_correction'].append(pitch_corr)
                data['roll_correction'].append(roll_corr)
                data['corrected_pitch'].append(raw_pitch + pitch_corr)
                data['corrected_roll'].append(raw_roll + roll_corr)

        # Extract distance data
        dist_match = re.search(distance_pattern, line)
        if dist_match:
            data['distance'].append(float(dist_match.group(1)))

        # Extract velocity data
        vel_match = re.search(velocity_pattern, line)
        if vel_match:
            data['velocity_north'].append(float(vel_match.group(1)))
            data['velocity_east'].append(float(vel_match.group(2)))

    # Extract PD controller parameters
    pd_params = {}
    position_pattern = r'Position: Kp=(\d+\.\d+), Kd=(\d+\.\d+)'
    velocity_pattern = r'Velocity: Kp=(\d+\.\d+), Kd=(\d+\.\d+)'
    attitude_gain_pattern = r'Gain: (\d+\.\d+)'
    max_pitch_roll_pattern = r'Max Pitch: (\d+\.\d+)°.*Max Roll: (\d+\.\d+)°'

    for line in lines:
        pos_match = re.search(position_pattern, line)
        if pos_match:
            pd_params['Kp_position'] = float(pos_match.group(1))
            pd_params['Kd_position'] = float(pos_match.group(2))

        vel_match = re.search(velocity_pattern, line)
        if vel_match:
            pd_params['Kp_velocity'] = float(vel_match.group(1))
            pd_params['Kd_velocity'] = float(vel_match.group(2))

        att_gain_match = re.search(attitude_gain_pattern, line)
        if att_gain_match and 'attitude_correction_gain' not in pd_params:
            pd_params['attitude_correction_gain'] = float(att_gain_match.group(1))

        max_match = re.search(max_pitch_roll_pattern, line)
        if max_match:
            pd_params['max_pitch'] = float(max_match.group(1))
            pd_params['max_roll'] = float(max_match.group(2))

    return data, pd_params


def analyze_attitude_data(data, pd_params):
    """
    Analyze the attitude data and provide recommendations.
    """
    pitch_data = np.array(data['pitch'])
    roll_data = np.array(data['roll'])
    pitch_acc_data = np.array(data['pitch_acc'])

    # Calculate statistics
    pitch_stats = {
        'mean': np.mean(pitch_data),
        'std': np.std(pitch_data),
        'min': np.min(pitch_data),
        'max': np.max(pitch_data),
        'range': np.max(pitch_data) - np.min(pitch_data)
    }

    roll_stats = {
        'mean': np.mean(roll_data),
        'std': np.std(roll_data),
        'min': np.min(roll_data),
        'max': np.max(roll_data),
        'range': np.max(roll_data) - np.min(roll_data)
    }

    pitch_acc_stats = {
        'mean': np.mean(pitch_acc_data),
        'std': np.std(pitch_acc_data),
        'min': np.min(pitch_acc_data),
        'max': np.max(pitch_acc_data)
    }

    # Extract PD parameters
    current_params = {
        'Kp_position': pd_params.get('Kp_position', 1.0),
        'Kd_position': pd_params.get('Kd_position', 0.5),
        'Kp_velocity': pd_params.get('Kp_velocity', 2.0),
        'Kd_velocity': pd_params.get('Kd_velocity', 0.8),
        'attitude_correction_gain': pd_params.get('attitude_correction_gain', 0.5),
        'max_pitch': pd_params.get('max_pitch', 5.0),
        'max_roll': pd_params.get('max_roll', 5.0)
    }

    # Generate recommendations based on analysis
    pitch_issues = []
    recommendations = []
    new_params = current_params.copy()

    # [Analysis logic from previous version]
    if abs(pitch_stats['mean']) > 10:
        pitch_issues.append(f"Mean pitch is significantly off vertical: {pitch_stats['mean']:.2f}°")
        recommendations.append("Decrease Kp_position to reduce aggressive pitch response")
        new_params['Kp_position'] = max(0.5, current_params['Kp_position'] * 0.8)

    if pitch_stats['range'] > 15:
        pitch_issues.append(f"Large pitch variations detected: {pitch_stats['range']:.2f}° range")
        recommendations.append("Increase Kd_position to dampen pitch oscillations")
        new_params['Kd_position'] = min(1.0, current_params['Kd_position'] * 1.3)

    if abs(pitch_acc_stats['max']) >= 5:
        pitch_issues.append(f"Pitch accumulator reaching high values: {pitch_acc_stats['max']:.2f}°")
        recommendations.append("Lower attitude_correction_gain to reduce accumulation of pitch correction")
        recommendations.append("Or increase max_pitch slightly to allow more pitch variation before correction")
        new_params['attitude_correction_gain'] = max(0.2, current_params['attitude_correction_gain'] * 0.7)
        new_params['max_pitch'] = min(10.0, current_params['max_pitch'] * 1.2)

    # Roll-related issues
    roll_issues = []
    if abs(roll_stats['mean']) > 5:
        roll_issues.append(f"Mean roll is off level: {roll_stats['mean']:.2f}°")

    if roll_stats['range'] > 10:
        roll_issues.append(f"Large roll variations detected: {roll_stats['range']:.2f}° range")

    return {
        'pitch_stats': pitch_stats,
        'roll_stats': roll_stats,
        'pitch_acc_stats': pitch_acc_stats,
        'pitch_issues': pitch_issues,
        'roll_issues': roll_issues,
        'recommendations': recommendations,
        'current_params': current_params,
        'recommended_params': new_params
    }


class InteractiveVisualization:
    def __init__(self, data, analysis_result):
        self.data = data
        self.analysis_result = analysis_result
        self.highlights = []
        self.annotations = []
        self.selected_time_range = (0, max(data['rel_timestamps']))
        self.info_text = None
        self.selection_active = False
        self.init_plots()
        self.add_interactivity()

    def init_plots(self):
        """Initialize the plot layout with four subplots"""
        self.fig = plt.figure(figsize=(14, 10))

        # Create a GridSpec layout for more control
        gs = self.fig.add_gridspec(4, 1, height_ratios=[3, 2, 2, 1], hspace=0.3)

        # Attitude plot (pitch and roll)
        self.ax1 = self.fig.add_subplot(gs[0])

        # Pitch accumulator and correction plot
        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)

        # Distance plot
        self.ax3 = self.fig.add_subplot(gs[2], sharex=self.ax1)

        # Control panel area
        self.ax_control = self.fig.add_subplot(gs[3])
        self.ax_control.axis('off')  # Hide axes

        # Prepare data for plotting
        self.times = self.data['rel_timestamps']
        self.pitch = self.data['pitch']
        self.roll = self.data['roll']
        self.pitch_acc = self.data['pitch_acc']

        # Create empty plot objects that we'll update later
        self.pitch_line, = self.ax1.plot([], [], 'b-', label='Raw Pitch (°)', linewidth=1.5, alpha=0.7)
        self.roll_line, = self.ax1.plot([], [], 'r-', label='Raw Roll (°)', linewidth=1.5, alpha=0.7)
        self.corr_pitch_line, = self.ax1.plot([], [], 'b--', label='Corrected Pitch (°)', linewidth=1.5)
        self.corr_roll_line, = self.ax1.plot([], [], 'r--', label='Corrected Roll (°)', linewidth=1.5)

        # Reference lines
        self.ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        max_pitch = self.analysis_result['current_params']['max_pitch']
        max_roll = self.analysis_result['current_params']['max_roll']
        self.ax1.axhline(y=max_pitch, color='b', linestyle='--', alpha=0.5)
        self.ax1.axhline(y=-max_pitch, color='b', linestyle='--', alpha=0.5)
        self.ax1.axhline(y=max_roll, color='r', linestyle='--', alpha=0.5)
        self.ax1.axhline(y=-max_roll, color='r', linestyle='--', alpha=0.5)

        # Pitch accumulator plot
        self.acc_line, = self.ax2.plot([], [], 'g-', label='Pitch Accumulator (°)', linewidth=2)
        self.corr_line, = self.ax2.plot([], [], 'm-', label='Pitch Correction (°)', linewidth=1.5, alpha=0.7)

        # Distance plot
        self.dist_line, = self.ax3.plot([], [], 'c-', label='Distance to Target (m)', linewidth=2)

        # Add velocity plot on secondary axis if data is available
        if self.data['velocity_north'] and self.data['velocity_east']:
            self.ax3_twin = self.ax3.twinx()
            self.vel_times = self.times[:len(self.data['velocity_north'])]
            self.velocity_total = [np.sqrt(n ** 2 + e ** 2) for n, e in
                                   zip(self.data['velocity_north'], self.data['velocity_east'])]
            self.vel_line, = self.ax3_twin.plot([], [], 'y-', label='Velocity (m/s)', linewidth=1.5)
            self.ax3_twin.set_ylabel('Velocity (m/s)', color='y')
            self.ax3_twin.tick_params(axis='y', labelcolor='y')

        # Set labels and titles
        self.ax1.set_ylabel('Angle (degrees)')
        self.ax1.set_title('Raw and Corrected Pitch/Roll vs Time')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True)

        self.ax2.set_ylabel('Value (degrees)')
        self.ax2.set_title('Pitch Accumulator and Correction vs Time')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True)

        self.ax3.set_xlabel('Time (seconds)')
        self.ax3.set_ylabel('Distance (meters)')
        self.ax3.set_title('Distance to Target vs Time')
        self.ax3.legend(loc='upper left')
        self.ax3.grid(True)

        # Add event markers from the data
        for event in self.data['event_markers']:
            self.add_event_marker(event)

        # Create info text for data inspection
        self.info_text = self.ax1.text(0.02, 0.02, "", transform=self.ax1.transAxes,
                                       bbox=dict(facecolor='white', alpha=0.7))

        # Update the plot data
        self.update_plot_data()

        plt.tight_layout()

    def add_event_marker(self, event):
        """Add a marker for significant events in the flight"""
        if event['type'] == 'pitch_change':
            self.ax1.plot(event['time'], event['value'], 'ro', markersize=8)
            self.ax1.annotate(event['description'],
                              xy=(event['time'], event['value']),
                              xytext=(10, 10), textcoords='offset points',
                              bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.7))
        elif event['type'] == 'correction':
            self.ax2.plot(event['time'], event['value'], 'bo', markersize=8)

    def update_plot_data(self):
        """Update the plot with current data and time range"""
        # Filter data to selected time range
        start_time, end_time = self.selected_time_range
        mask = [(t >= start_time) and (t <= end_time) for t in self.times]

        filtered_times = [self.times[i] for i in range(len(self.times)) if mask[i]]
        filtered_pitch = [self.pitch[i] for i in range(len(self.pitch)) if mask[i]]
        filtered_roll = [self.roll[i] for i in range(len(self.roll)) if mask[i]]

        # Update the data in each plot
        self.pitch_line.set_data(filtered_times, filtered_pitch)
        self.roll_line.set_data(filtered_times, filtered_roll)

        if 'corrected_pitch' in self.data and self.data['corrected_pitch']:
            filtered_corr_pitch = [self.data['corrected_pitch'][i] for i in range(len(self.data['corrected_pitch'])) if
                                   i < len(mask) and mask[i]]
            filtered_corr_roll = [self.data['corrected_roll'][i] for i in range(len(self.data['corrected_roll'])) if
                                  i < len(mask) and mask[i]]
            self.corr_pitch_line.set_data(filtered_times, filtered_corr_pitch)
            self.corr_roll_line.set_data(filtered_times, filtered_corr_roll)

        # Update pitch accumulator and correction
        filtered_acc = [self.pitch_acc[i] for i in range(len(self.pitch_acc)) if mask[i]]
        self.acc_line.set_data(filtered_times, filtered_acc)

        if 'pitch_correction' in self.data and self.data['pitch_correction']:
            # Find the timestamps that have corrections
            corr_indices = [i for i in range(len(self.data['pitch_correction'])) if i < len(mask) and mask[i]]
            if corr_indices:
                corr_times = [self.times[i] for i in corr_indices]
                corr_values = [self.data['pitch_correction'][i] for i in range(len(self.data['pitch_correction'])) if
                               i in corr_indices]
                self.corr_line.set_data(corr_times, corr_values)

        # Update distance plot
        if 'distance' in self.data and self.data['distance']:
            # Match the timestamp indices for distance data
            dist_len = min(len(self.data['distance']), len(mask))
            filtered_dist = [self.data['distance'][i] for i in range(dist_len) if mask[i]]
            dist_times = [self.times[i] for i in range(dist_len) if mask[i]]
            self.dist_line.set_data(dist_times, filtered_dist)

        # Update velocity plot if available
        if hasattr(self, 'vel_line') and self.data['velocity_north'] and self.data['velocity_east']:
            vel_len = min(len(self.velocity_total), len(mask))
            filtered_vel = [self.velocity_total[i] for i in range(vel_len) if mask[i]]
            vel_times = [self.times[i] for i in range(vel_len) if mask[i]]
            self.vel_line.set_data(vel_times, filtered_vel)

        # Update the axes limits
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.ax3.relim()
        self.ax3.autoscale_view()
        if hasattr(self, 'ax3_twin'):
            self.ax3_twin.relim()
            self.ax3_twin.autoscale_view()

        # Redraw the figure
        self.fig.canvas.draw_idle()

    def add_interactivity(self):
        """Add interactive elements to the visualization"""
        # Add a button to reset the view
        self.reset_button_ax = plt.axes([0.8, 0.01, 0.1, 0.04])
        self.reset_button = Button(self.reset_button_ax, 'Reset View')
        self.reset_button.on_clicked(self.reset_view)

        # Add a button to show statistics
        self.stats_button_ax = plt.axes([0.65, 0.01, 0.1, 0.04])
        self.stats_button = Button(self.stats_button_ax, 'Statistics')
        self.stats_button.on_clicked(self.show_statistics)

        # Add a button to show recommendations
        self.rec_button_ax = plt.axes([0.5, 0.01, 0.1, 0.04])
        self.rec_button = Button(self.rec_button_ax, 'Recommendations')
        self.rec_button.on_clicked(self.show_recommendations)

        # Add a button to toggle the selection tool
        self.select_button_ax = plt.axes([0.35, 0.01, 0.1, 0.04])
        self.select_button = Button(self.select_button_ax, 'Select Region')
        self.select_button.on_clicked(self.toggle_selection)

        # Add a search box to find specific timestamp
        self.search_box_ax = plt.axes([0.05, 0.01, 0.1, 0.04])
        self.search_box = TextBox(self.search_box_ax, 'Time:', initial='')
        self.search_box.on_submit(self.goto_time)

        # Add mouse position tracking for detailed data inspection
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)

        # Rectangle selector for zooming into specific regions
        self.rect_selector = RectangleSelector(
            self.ax1, self.on_select_region,
            useblit=True,
            button=[1],  # left mouse button
            minspanx=5,
            minspany=5,
            spancoords='pixels',
            interactive=True,
            props=dict(facecolor='blue', alpha=0.2)
        )
        self.rect_selector.set_active(False)  # Start with selection disabled

    def on_mouse_move(self, event):
        """Update the info text with data at mouse position"""
        if event.inaxes == self.ax1:
            # Find the closest data point
            if not self.times:
                return

            x = event.xdata
            if x is None:
                return

            # Find the closest time point
            idx = np.argmin(np.abs(np.array(self.times) - x))

            if idx < len(self.pitch) and idx < len(self.roll):
                pitch_val = self.pitch[idx]
                roll_val = self.roll[idx]
                time_val = self.times[idx]

                # Get corrected values if available
                corr_pitch = self.data['corrected_pitch'][idx] if idx < len(self.data['corrected_pitch']) else "N/A"
                corr_roll = self.data['corrected_roll'][idx] if idx < len(self.data['corrected_roll']) else "N/A"

                # Get acc value
                acc_val = self.pitch_acc[idx] if idx < len(self.pitch_acc) else "N/A"

                # Create info text
                info = f"Time: {time_val:.2f}s\nPitch: {pitch_val:.2f}° (Corr: {corr_pitch:.2f}°)\nRoll: {roll_val:.2f}° (Corr: {corr_roll:.2f}°)\nAccumulator: {acc_val:.2f}°"

                # Get distance if available
                if 'distance' in self.data and self.data['distance'] and idx < len(self.data['distance']):
                    info += f"\nDistance: {self.data['distance'][idx]:.2f}m"

                # Update the info text
                self.info_text.set_text(info)
                self.fig.canvas.draw_idle()

    def on_select_region(self, eclick, erelease):
        """Handle the region selection for zooming"""
        if not self.selection_active:
            return

        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata

        # Ensure x1 <= x2
        if x1 > x2:
            x1, x2 = x2, x1

        # Update the time range and replot
        self.selected_time_range = (x1, x2)
        self.update_plot_data()

        # Update all axis limits to focus on the selected region
        self.ax1.set_xlim(x1, x2)
        self.ax2.set_xlim(x1, x2)
        self.ax3.set_xlim(x1, x2)

    def toggle_selection(self, event):
        """Toggle the rectangle selector for region selection"""
        self.selection_active = not self.selection_active
        self.rect_selector.set_active(self.selection_active)

        # Update button text
        if self.selection_active:
            self.select_button.label.set_text('Selecting...')
        else:
            self.select_button.label.set_text('Select Region')

        self.fig.canvas.draw_idle()

    def reset_view(self, event):
        """Reset to the full time range view"""
        self.selected_time_range = (min(self.times), max(self.times))
        self.update_plot_data()

        # Reset all axes
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.ax3.relim()
        self.ax3.autoscale_view()
        if hasattr(self, 'ax3_twin'):
            self.ax3_twin.relim()
            self.ax3_twin.autoscale_view()

        self.fig.canvas.draw_idle()

    def goto_time(self, text):
        """Jump to a specific time value"""
        try:
            time_val = float(text)
            if min(self.times) <= time_val <= max(self.times):
                # Set a small window around the specified time
                window = (max(self.times) - min(self.times)) * 0.1  # 10% of total range
                self.selected_time_range = (time_val - window / 2, time_val + window / 2)
                self.update_plot_data()

                # Update all axis limits
                self.ax1.set_xlim(self.selected_time_range)
                self.ax2.set_xlim(self.selected_time_range)
                self.ax3.set_xlim(self.selected_time_range)

                self.fig.canvas.draw_idle()
            else:
                print(f"Time value {time_val} is outside the data range")
        except ValueError:
            print(f"Invalid time value: {text}")

    def show_statistics(self, event):
        """Display statistics in a separate window"""
        stats_fig = plt.figure(figsize=(10, 6))
        ax = stats_fig.add_subplot(111)
        ax.axis('off')

        # Format statistics text
        stats_text = "=== ATTITUDE ANALYSIS RESULTS ===\n\n"

        stats_text += "Pitch Statistics:\n"
        stats_text += f"  Mean: {self.analysis_result['pitch_stats']['mean']:.2f}°\n"
        stats_text += f"  Std Dev: {self.analysis_result['pitch_stats']['std']:.2f}°\n"
        stats_text += f"  Range: {self.analysis_result['pitch_stats']['min']:.2f}° to {self.analysis_result['pitch_stats']['max']:.2f}°\n"
        stats_text += f"  Total Range: {self.analysis_result['pitch_stats']['range']:.2f}°\n\n"

        stats_text += "Roll Statistics:\n"
        stats_text += f"  Mean: {self.analysis_result['roll_stats']['mean']:.2f}°\n"
        stats_text += f"  Std Dev: {self.analysis_result['roll_stats']['std']:.2f}°\n"
        stats_text += f"  Range: {self.analysis_result['roll_stats']['min']:.2f}° to {self.analysis_result['roll_stats']['max']:.2f}°\n"
        stats_text += f"  Total Range: {self.analysis_result['roll_stats']['range']:.2f}°\n\n"

        stats_text += "Pitch Accumulator Statistics:\n"
        stats_text += f"  Mean: {self.analysis_result['pitch_acc_stats']['mean']:.2f}°\n"
        stats_text += f"  Min: {self.analysis_result['pitch_acc_stats']['min']:.2f}°\n"
        stats_text += f"  Max: {self.analysis_result['pitch_acc_stats']['max']:.2f}°\n\n"

        if self.analysis_result['pitch_issues']:
            stats_text += "Pitch Issues Detected:\n"
            for issue in self.analysis_result['pitch_issues']:
                stats_text += f"  - {issue}\n"
            stats_text += "\n"

        if self.analysis_result['roll_issues']:
            stats_text += "Roll Issues Detected:\n"
            for issue in self.analysis_result['roll_issues']:
                stats_text += f"  - {issue}\n"
            stats_text += "\n"

        ax.text(0.05, 0.95, stats_text, fontsize=12, va='top', ha='left',
                transform=ax.transAxes, fontfamily='monospace')

        stats_fig.tight_layout()
        stats_fig.canvas.manager.set_window_title('Attitude Statistics')
        stats_fig.show()

    def show_recommendations(self, event):
        """Display PD parameter recommendations in a separate window"""
        rec_fig = plt.figure(figsize=(10, 6))
        ax = rec_fig.add_subplot(111)
        ax.axis('off')

        # Format recommendations text
        rec_text = "=== PD PARAMETER RECOMMENDATIONS ===\n\n"

        rec_text += "Current PD Parameters:\n"
        for param, value in self.analysis_result['current_params'].items():
            rec_text += f"  {param}: {value}\n"
        rec_text += "\n"

        rec_text += "Recommended PD Parameter Adjustments:\n"
        for param in self.analysis_result['current_params']:
            current = self.analysis_result['current_params'][param]
            recommended = self.analysis_result['recommended_params'][param]
            if current != recommended:
                rec_text += f"  {param}: {current} -> {recommended}\n"
        rec_text += "\n"

        rec_text += "Recommendations:\n"
        for i, rec in enumerate(self.analysis_result['recommendations'], 1):
            rec_text += f"  {i}. {rec}\n"

        ax.text(0.05, 0.95, rec_text, fontsize=12, va='top', ha='left',
                transform=ax.transAxes, fontfamily='monospace')

        rec_fig.tight_layout()
        rec_fig.canvas.manager.set_window_title('PD Parameter Recommendations')
        rec_fig.show()


def main():
    """Main function to read log file and launch interactive visualization"""
    # Log file path - use current directory by default
    log_file_path = 'logs/gps_sitl_20250303_233957.log'

    try:
        # Read log file
        with open(log_file_path, 'r') as f:
            log_content = f.read()

        print(f"Log file loaded successfully. Processing data...")

        # Parse log data
        data, pd_params = parse_log_file(log_content)

        print(f"Log data parsed. Found {len(data['timestamps'])} data points.")

        # Analyze data
        analysis_result = analyze_attitude_data(data, pd_params)

        # Print brief summary
        print("\n=== ANALYSIS SUMMARY ===")
        print(
            f"Pitch mean: {analysis_result['pitch_stats']['mean']:.2f}°, range: {analysis_result['pitch_stats']['range']:.2f}°")
        print(
            f"Roll mean: {analysis_result['roll_stats']['mean']:.2f}°, range: {analysis_result['roll_stats']['range']:.2f}°")
        print(f"Pitch accumulator max: {analysis_result['pitch_acc_stats']['max']:.2f}°")
        print("\nLaunching interactive visualization...")

        # Launch interactive visualization
        viz = InteractiveVisualization(data, analysis_result)
        plt.show()

    except FileNotFoundError:
        print(f"Error: Log file '{log_file_path}' not found.")
        print("Make sure the file exists in the current directory or provide the full path.")
    except Exception as e:
        print(f"Error analyzing log file: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()