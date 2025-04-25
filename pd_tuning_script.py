#!/usr/bin/env python3
"""
Modified PID Controller Parameter Tester
Testing P, I, and D values
"""

import os
import sys
import time
import signal
import subprocess
import numpy as np
import pandas as pd
from datetime import datetime
import re
import shutil

# Configuration
CONFIG = {
    # Test parameters list with comprehensive PID combinations
    # PID tuning combinations for circular drone motion
    'test_parameters': [
        # Test 1: Baseline - Low P, moderate D, minimal I
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },

        # Test 2-7: Fixed P with varying D (position focus)
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.5,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 0.8,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 0.5,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 0.3,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 2.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 2.5,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },

        # Test 8-13: Fixed P with varying D (velocity focus)
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.5,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.8,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.5,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.3,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 2.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 2.5,
            'max_accel_value': 0.4
        },

        # Test 14-19: Varying P values (position focus)
        {
            'kp_position_value': 0.05,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.03,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.12,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.15,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.2,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.01,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },

        # Test 20-25: Varying P values (velocity focus)
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.1,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.05,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.2,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.25,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.3,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.01,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },

        # Test 26-31: Varying I values (position focus)
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0001,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0003,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0005,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0008,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.001,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.00005,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },

        # Test 32-37: Varying I values (velocity focus)
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.00005,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0002,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0003,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0005,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0008,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.00001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.4
        },

        # Test 38-43: Varying acceleration limits
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.2
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.3
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.5
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.6
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.8
        },
        {
            'kp_position_value': 0.08,
            'ki_position_value': 0.0002,
            'kd_position_value': 1.0,
            'kp_velocity_value': 0.15,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 1.0,
            'max_accel_value': 0.1
        },

        # Test 44-49: Low P, high D combinations
        {
            'kp_position_value': 0.05,
            'ki_position_value': 0.0002,
            'kd_position_value': 2.0,
            'kp_velocity_value': 0.1,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 2.0,
            'max_accel_value': 0.3
        },
        {
            'kp_position_value': 0.03,
            'ki_position_value': 0.0002,
            'kd_position_value': 2.5,
            'kp_velocity_value': 0.08,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 2.5,
            'max_accel_value': 0.3
        },
        {
            'kp_position_value': 0.02,
            'ki_position_value': 0.0002,
            'kd_position_value': 3.0,
            'kp_velocity_value': 0.05,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 3.0,
            'max_accel_value': 0.3
        },
        {
            'kp_position_value': 0.01,
            'ki_position_value': 0.0002,
            'kd_position_value': 3.5,
            'kp_velocity_value': 0.03,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 3.5,
            'max_accel_value': 0.3
        },
        {
            'kp_position_value': 0.01,
            'ki_position_value': 0.0001,
            'kd_position_value': 4.0,
            'kp_velocity_value': 0.02,
            'ki_velocity_value': 0.00005,
            'kd_velocity_value': 4.0,
            'max_accel_value': 0.3
        },
        {
            'kp_position_value': 0.005,
            'ki_position_value': 0.0001,
            'kd_position_value': 5.0,
            'kp_velocity_value': 0.01,
            'ki_velocity_value': 0.00005,
            'kd_velocity_value': 5.0,
            'max_accel_value': 0.3
        },

        # Test 50-55: High P, low D combinations (for testing contrast)
        {
            'kp_position_value': 0.3,
            'ki_position_value': 0.0001,
            'kd_position_value': 0.5,
            'kp_velocity_value': 0.3,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.5,
            'max_accel_value': 0.5
        },
        {
            'kp_position_value': 0.4,
            'ki_position_value': 0.0001,
            'kd_position_value': 0.4,
            'kp_velocity_value': 0.4,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.4,
            'max_accel_value': 0.5
        },
        {
            'kp_position_value': 0.5,
            'ki_position_value': 0.0001,
            'kd_position_value': 0.3,
            'kp_velocity_value': 0.5,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.3,
            'max_accel_value': 0.5
        },
        {
            'kp_position_value': 0.6,
            'ki_position_value': 0.0001,
            'kd_position_value': 0.2,
            'kp_velocity_value': 0.6,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.2,
            'max_accel_value': 0.5
        },
        {
            'kp_position_value': 0.7,
            'ki_position_value': 0.0001,
            'kd_position_value': 0.1,
            'kp_velocity_value': 0.7,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.1,
            'max_accel_value': 0.5
        },
        {
            'kp_position_value': 0.8,
            'ki_position_value': 0.0001,
            'kd_position_value': 0.05,
            'kp_velocity_value': 0.8,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 0.05,
            'max_accel_value': 0.5
        },

        # Test 56-60: Balanced PID combinations with varying acceleration
        {
            'kp_position_value': 0.06,
            'ki_position_value': 0.0003,
            'kd_position_value': 1.5,
            'kp_velocity_value': 0.12,
            'ki_velocity_value': 0.0002,
            'kd_velocity_value': 1.5,
            'max_accel_value': 0.25
        },
        {
            'kp_position_value': 0.04,
            'ki_position_value': 0.0004,
            'kd_position_value': 1.8,
            'kp_velocity_value': 0.1,
            'ki_velocity_value': 0.0002,
            'kd_velocity_value': 1.8,
            'max_accel_value': 0.2
        },
        {
            'kp_position_value': 0.03,
            'ki_position_value': 0.0003,
            'kd_position_value': 2.2,
            'kp_velocity_value': 0.08,
            'ki_velocity_value': 0.0002,
            'kd_velocity_value': 2.0,
            'max_accel_value': 0.15
        },
        {
            'kp_position_value': 0.02,
            'ki_position_value': 0.0002,
            'kd_position_value': 2.5,
            'kp_velocity_value': 0.06,
            'ki_velocity_value': 0.0001,
            'kd_velocity_value': 2.2,
            'max_accel_value': 0.1
        },
        {
            'kp_position_value': 0.015,
            'ki_position_value': 0.0001,
            'kd_position_value': 3.0,
            'kp_velocity_value': 0.04,
            'ki_velocity_value': 0.00005,
            'kd_velocity_value': 2.5,
            'max_accel_value': 0.08
        }
    ],

    # Test configuration
    'test_duration': 60,  # 2.5 minutes per test
    'results_dir': 'pd_test_results',
    'log_dir': 'pd_test_logs',

    # Process commands
    'simulator_cmd': 'python3 ublox_m8n_sim.py --virtual  --lat 23.194101 --lon 77.365802 --pattern circle --radius 100',
    'follow_cmd': 'python3 sitl_follow_test_v6.py --gps-port=/tmp/vgps2 --altitude=10',
    'visualization_cmd': 'python3 gps_map_visualization.py'
}


class PD_Tester:
    def __init__(self, config):
        self.config = config
        self.results = []
        self.processes = {}

        # Setup directories
        os.makedirs(config['results_dir'], exist_ok=True)
        os.makedirs(config['log_dir'], exist_ok=True)

        # Create session directory
        self.session_dir = os.path.join(
            config['results_dir'],
            f"pd_test_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        os.makedirs(self.session_dir)

        signal.signal(signal.SIGINT, self.signal_handler)

    def modify_parameters(self, kp_p, ki_p, kd_p, kp_v, ki_v, kd_v):
        """Create temp script with modified parameters"""
        temp_script = 'sitl_follow_temp.py'
        with open('sitl_follow_test_v6.py', 'r') as f:
            content = f.read()

        content = re.sub(r'self\.Kp_position\s*=\s*[\d\.e-]+',
                         f'self.Kp_position = {kp_p}', content)
        content = re.sub(r'self\.Ki_position\s*=\s*[\d\.e-]+',
                         f'self.Ki_position = {ki_p}', content)
        content = re.sub(r'self\.Kd_position\s*=\s*[\d\.e-]+',
                         f'self.Kd_position = {kd_p}', content)
        content = re.sub(r'self\.Kp_velocity\s*=\s*[\d\.e-]+',
                         f'self.Kp_velocity = {kp_v}', content)
        content = re.sub(r'self\.Ki_velocity\s*=\s*[\d\.e-]+',
                         f'self.Ki_velocity = {ki_v}', content)
        content = re.sub(r'self\.Kd_velocity\s*=\s*[\d\.e-]+',
                         f'self.Kd_velocity = {kd_v}', content)

        with open(temp_script, 'w') as f:
            f.write(content)

        return temp_script

    def run_test(self, kp_p, ki_p, kd_p, kp_v, ki_v, kd_v):
        """Run single test iteration"""
        print(
            f"\nTesting: Kp_p={kp_p:.4f}, Ki_p={ki_p:.6f}, Kd_p={kd_p:.2f}, Kp_v={kp_v:.4f}, Ki_v={ki_v:.6f}, Kd_v={kd_v:.2f}")

        try:
            # Start GPS simulator
            self.processes['sim'] = subprocess.Popen(
                self.config['simulator_cmd'].split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)

            # Create temp script and run test
            temp_script = self.modify_parameters(kp_p, ki_p, kd_p, kp_v, ki_v, kd_v)
            test_cmd = self.config['follow_cmd'].replace('sitl_follow_test_v6.py', temp_script)

            self.processes['test'] = subprocess.Popen(
                test_cmd.split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Wait for test duration
            time.sleep(self.config['test_duration'])
            self.cleanup_processes()

            # Run visualization
            subprocess.run(self.config['visualization_cmd'].split(), check=True)

            # Process results
            log_file = self.get_latest_log()
            metrics = self.extract_metrics(log_file)

            # Save parameters and metrics
            result = {
                'kp_position': kp_p,
                'ki_position': ki_p,
                'kd_position': kd_p,
                'kp_velocity': kp_v,
                'ki_velocity': ki_v,
                'kd_velocity': kd_v,
                'log_file': log_file,
                **metrics
            }

            self.save_iteration_result(result)
            return result

        finally:
            # Cleanup temp files
            if os.path.exists(temp_script):
                os.remove(temp_script)

    def extract_metrics(self, log_file):
        """Extract performance metrics from log"""
        metrics = {'avg_distance': 0, 'max_distance': 0, 'min_distance': 0}
        try:
            with open(log_file, 'r') as f:
                log = f.read()

            distances = re.findall(r'Distance to target: ([\d\.]+)m', log)
            if distances:
                distances = list(map(float, distances))
                metrics.update({
                    'avg_distance': np.mean(distances),
                    'max_distance': np.max(distances),
                    'min_distance': np.min(distances)
                })
        except Exception as e:
            print(f"Error extracting metrics: {e}")
        return metrics

    def get_latest_log(self):
        """Get most recent log file"""
        logs = [os.path.join(self.config['log_dir'], f)
                for f in os.listdir(self.config['log_dir']) if f.endswith('.log')]
        return max(logs, key=os.path.getctime) if logs else None

    def save_iteration_result(self, result):
        """Save results for current iteration"""
        # Save to CSV
        df = pd.DataFrame([result])
        csv_path = os.path.join(self.session_dir, 'pd_test_results.csv')
        df.to_csv(csv_path, mode='a', header=not os.path.exists(csv_path), index=False)

        # Save to text file
        txt_path = os.path.join(self.session_dir, 'parameter_log.txt')
        with open(txt_path, 'a') as f:
            f.write(
                f"Kp_p: {result['kp_position']:.4f}, "
                f"Ki_p: {result['ki_position']:.6f}, "
                f"Kd_p: {result['kd_position']:.2f}, "
                f"Kp_v: {result['kp_velocity']:.4f}, "
                f"Ki_v: {result['ki_velocity']:.6f}, "
                f"Kd_v: {result['kd_velocity']:.2f}, "
                f"MaxAccel: {result.get('max_accel', 'N/A')}, "
                f"AvgDist: {result['avg_distance']:.2f}m\n"
            )

    def cleanup_processes(self):
        """Terminate running processes"""
        for name, proc in self.processes.items():
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C interruption"""
        print("\nTesting interrupted. Saving results...")
        self.cleanup_processes()
        if self.results:
            self.save_results()
        sys.exit(0)

    def save_results(self):
        """Save all results to a final report"""
        if not self.results:
            return

        # Create DataFrame with all results
        df = pd.DataFrame(self.results)

        # Sort by average distance
        df_sorted = df.sort_values('avg_distance')

        # Save sorted results
        csv_path = os.path.join(self.session_dir, 'final_results_sorted.csv')
        df_sorted.to_csv(csv_path, index=False)

        # Print best result
        best = df_sorted.iloc[0]
        print(f"\nBest parameters:")
        print(f"Kp_position: {best['kp_position']:.4f}")
        print(f"Ki_position: {best['ki_position']:.6f}")
        print(f"Kd_position: {best['kd_position']:.2f}")
        print(f"Kp_velocity: {best['kp_velocity']:.4f}")
        print(f"Ki_velocity: {best['ki_velocity']:.6f}")
        print(f"Kd_velocity: {best['kd_velocity']:.2f}")
        if 'max_accel' in best:
            print(f"Max acceleration: {best['max_accel']:.2f} m/s²")
        print(f"Average distance: {best['avg_distance']:.2f}m")

    def run_testing(self):
        """Main testing loop"""
        print(f"Starting PID testing with varying parameters in {self.session_dir}")

        # Save test configuration
        with open(os.path.join(self.session_dir, 'test_config.txt'), 'w') as f:
            f.write("PID Test Configuration:\n")
            f.write(f"Number of test cases: {len(self.config['test_parameters'])}\n")
            f.write("Testing various combinations of P, I, D values and acceleration limits\n")

        # Run tests with the parameter combinations from the list
        for i, params in enumerate(self.config['test_parameters']):
            print(f"\nTest {i + 1}/{len(self.config['test_parameters'])}")

            # Extract parameters for this test
            kp_p = params['kp_position_value']
            ki_p = params['ki_position_value']
            kd_p = params['kd_position_value']
            kp_v = params['kp_velocity_value']
            ki_v = params['ki_velocity_value']
            kd_v = params['kd_velocity_value']
            max_accel = params['max_accel_value']

            # Update the follow command with max acceleration parameter
            follow_cmd = self.config['follow_cmd'] + f" --max-accel={max_accel}"
            self.config['follow_cmd'] = follow_cmd

            # Run the test with this parameter set
            result = self.run_test(kp_p, ki_p, kd_p, kp_v, ki_v, kd_v)

            # Add max_accel to result
            result['max_accel'] = max_accel

            self.results.append(result)
            time.sleep(2)  # Cool-down period

        self.save_results()
        print("\nPID testing complete!")
        print(f"Results saved to {self.session_dir}")


if __name__ == "__main__":
    tester = PD_Tester(CONFIG)
    tester.run_testing()