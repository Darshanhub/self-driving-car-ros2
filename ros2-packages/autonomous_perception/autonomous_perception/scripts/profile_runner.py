import argparse
import csv
import time
from pathlib import Path

import psutil


def profile_command(command: str, duration: float, output: str):
    proc = psutil.Popen(command, shell=True)
    stats = []
    start = time.time()
    try:
        while proc.is_running() and (time.time() - start) < duration:
            cpu = proc.cpu_percent(interval=0.5)
            mem = proc.memory_info().rss / (1024 * 1024)
            stats.append({'ts': time.time() - start, 'cpu_percent': cpu, 'ram_mb': mem})
    finally:
        proc.terminate()

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=['ts', 'cpu_percent', 'ram_mb'])
        writer.writeheader()
        writer.writerows(stats)


def main():
    parser = argparse.ArgumentParser(description='Profile a ROS 2 launch command.')
    parser.add_argument('--command', required=True, help='Command to profile, e.g. "ros2 launch ..."')
    parser.add_argument('--duration', type=float, default=60.0, help='Maximum profiling duration (s).')
    parser.add_argument('--output', default='profile.csv', help='CSV file to store metrics.')
    args = parser.parse_args()
    profile_command(args.command, args.duration, args.output)


if __name__ == '__main__':
    main()
