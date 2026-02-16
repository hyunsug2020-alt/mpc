#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import time
from datetime import datetime
from itertools import product

import pandas as pd
import yaml


class MPCParameterTuner:
    def __init__(self):
        self.config_path = os.path.expanduser(
            '~/Mobility_Challenge_Simulator/install/bisa/share/bisa/config/cav_config.yaml'
        )
        self.base_config = self.load_config()

        # Parameter grid
        self.param_grid = {
            'Q_pos': [50, 100, 143.6, 200],
            'Q_heading': [50, 100, 139.4, 200],
            'R_v': [0.3, 0.5, 0.8],
            'R_w': [1.5, 2.9, 4.0],
            'horizon': [100, 150, 200, 250]
        }

        self.results = []

    def load_config(self):
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)

    def save_config(self, config):
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

    def update_params(self, Q_pos, Q_heading, R_v, R_w, horizon):
        config = self.base_config.copy()

        # Update Slot 01 (CAV 1)
        config['mpc_tracker_cav01']['ros__parameters']['Q_pos'] = float(Q_pos)
        config['mpc_tracker_cav01']['ros__parameters']['Q_heading'] = float(Q_heading)
        config['mpc_tracker_cav01']['ros__parameters']['R_v'] = float(R_v)
        config['mpc_tracker_cav01']['ros__parameters']['R_w'] = float(R_w)
        config['mpc_tracker_cav01']['ros__parameters']['horizon'] = int(horizon)

        self.save_config(config)
        print(f"Updated config: Q_pos={Q_pos}, Q_heading={Q_heading}, "
              f"R_v={R_v}, R_w={R_w}, horizon={horizon}")

    def run_simulation(self, duration=60):
        """Run simulation and collect bag"""
        print(f"Running simulation for {duration}s...")

        # Start launch file
        launch_cmd = [
            'bash', '-c',
            'source /opt/ros/foxy/setup.bash && '
            'cd ~/Mobility_Challenge_Simulator && '
            'source install/setup.bash && '
            'ros2 launch bisa problem3_cpp_launch.py'
        ]

        proc = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(10)  # Wait for startup

        # Record bag
        bag_dir = f'/tmp/mpc_tuning_{int(time.time())}'
        bag_cmd = [
            'bash', '-c',
            f'source /opt/ros/foxy/setup.bash && '
            f'ros2 bag record -d {duration} -o {bag_dir} '
            f'/CAV_01 /local_path_cav01 /mpc_performance'
        ]

        bag_proc = subprocess.Popen(bag_cmd)
        bag_proc.wait()

        # Kill launch
        proc.terminate()
        proc.wait(timeout=5)

        return bag_dir

    def analyze_bag(self, bag_dir):
        """Analyze bag and extract metrics"""
        # Placeholder: In real implementation, use rosbag2 API

        import random
        metrics = {
            'mean_lateral_error': random.uniform(0.05, 0.30),
            'max_lateral_error': random.uniform(0.20, 0.80),
            'mean_velocity': random.uniform(0.8, 1.2),
            'velocity_variance': random.uniform(0.01, 0.10),
            'mean_compute_time': random.uniform(300, 800),
            'p99_compute_time': random.uniform(600, 1200),
        }

        return metrics

    def run_tuning(self, max_iterations=None):
        """Run parameter sweep"""

        # Generate combinations
        keys = list(self.param_grid.keys())
        values = list(self.param_grid.values())
        combinations = list(product(*values))

        if max_iterations:
            combinations = combinations[:max_iterations]

        print(f"\n{'='*60}")
        print("MPC PARAMETER TUNING")
        print(f"Total combinations: {len(combinations)}")
        print(f"{'='*60}\n")

        for i, combo in enumerate(combinations, 1):
            params = dict(zip(keys, combo))

            print(f"\n[{i}/{len(combinations)}] Testing: {params}")

            # Update config
            self.update_params(**params)

            # Run simulation
            bag_dir = self.run_simulation(duration=30)

            # Analyze
            metrics = self.analyze_bag(bag_dir)

            # Store results
            result = {**params, **metrics, 'bag_dir': bag_dir}
            self.results.append(result)

            print(f"  Mean Error: {metrics['mean_lateral_error']:.4f}m")
            print(f"  Max Error: {metrics['max_lateral_error']:.4f}m")
            print(f"  Compute Time: {metrics['mean_compute_time']:.1f}us")

            # Cleanup
            subprocess.run(['rm', '-rf', bag_dir], check=False)

        # Save results
        self.save_results()

        # Find best
        self.find_best()

    def save_results(self):
        """Save results to CSV"""
        df = pd.DataFrame(self.results)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.expanduser(
            f'~/Mobility_Challenge_Simulator/logs/mpc_tuning_{timestamp}.csv'
        )

        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        df.to_csv(csv_path, index=False)

        print(f"\nSaved results to: {csv_path}")

    def find_best(self):
        """Find and display best parameters"""
        df = pd.DataFrame(self.results)

        # Multi-objective: minimize error and compute time
        df['score'] = (
            df['mean_lateral_error'] / df['mean_lateral_error'].max() * 0.5 +
            df['max_lateral_error'] / df['max_lateral_error'].max() * 0.3 +
            df['mean_compute_time'] / df['mean_compute_time'].max() * 0.2
        )

        df_sorted = df.sort_values('score')

        print("\n" + "=" * 60)
        print("TOP 5 PARAMETER SETS")
        print("=" * 60)

        for i, row in df_sorted.head(5).iterrows():
            print(f"\n#{i+1}:")
            print(f"  Q_pos: {row['Q_pos']}")
            print(f"  Q_heading: {row['Q_heading']}")
            print(f"  R_v: {row['R_v']}")
            print(f"  R_w: {row['R_w']}")
            print(f"  horizon: {int(row['horizon'])}")
            print(f"  Mean Error: {row['mean_lateral_error']:.4f}m")
            print(f"  Max Error: {row['max_lateral_error']:.4f}m")
            print(f"  Compute: {row['mean_compute_time']:.1f}us")
            print(f"  Score: {row['score']:.4f}")

        print("\n" + "=" * 60)

        # Save best config
        best = df_sorted.iloc[0]
        best_config = {
            'Q_pos': float(best['Q_pos']),
            'Q_heading': float(best['Q_heading']),
            'R_v': float(best['R_v']),
            'R_w': float(best['R_w']),
            'horizon': int(best['horizon'])
        }

        yaml_path = os.path.expanduser(
            '~/Mobility_Challenge_Simulator/src/bisa/config/cav_config_optimized.yaml'
        )

        with open(yaml_path, 'w') as f:
            yaml.dump({'mpc_tracker_cav01': {'ros__parameters': best_config}}, f)

        print(f"\nSaved best config to: {yaml_path}\n")


def main():
    tuner = MPCParameterTuner()

    # Run limited sweep (full would take hours)
    tuner.run_tuning(max_iterations=10)


if __name__ == '__main__':
    main()
