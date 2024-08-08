#!/usr/bin/env python3

import argparse
import subprocess
import os
import shutil
from ament_index_python.packages import get_package_share_directory

DEFAULT_DOWNLOAD_DIR = "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip"


def run(cmd):
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd='.')
    return process_handle

def main():
    parser = argparse.ArgumentParser(description='Gazebo simulation')

    parser.add_argument('--world', help='World to run in Gazebo', required=False, default="tucan")
    parser.add_argument('--gz_partition', help='Gazebo partition to run in', required=False)
    parser.add_argument('--gz_ip', help='Outgoing network interface to use for traffic',required=False)
    parser.add_argument('--interactive',help='Run in interactive mode', required=False, default=False, action='store_true')
    parser.add_argument('--model_download_source', help='Path to directory containing model files', required=False, default=DEFAULT_DOWNLOAD_DIR)
    parser.add_argument('--model_store', help='Path to model storage directory', required=False, default=(get_package_share_directory('tucan_sim')))
    parser.add_argument('--overwrite', help='Overwrite existing model directories', required=False, default=False, action='store_true')
    parser.add_argument('--dryrun', help='Test in dryrun. Do not launch gazebo', required=False, default=False, action='store_true')
    parser.add_argument('--headless', help='Run Gazebo without GUI', required=False, default=False, action='store_true')

    args = parser.parse_args()

    # Set up environment variables to look for models for simulation
    args.model_store = os.path.expanduser(args.model_store)

    model_count = int(subprocess.check_output(f'find {args.model_store} -type f | wc -l', shell=True, text=True))
    print(f"Found: {model_count} files in {args.model_store}")

    # Launch gazebo simulation
    print('> Launching gazebo simulation...')
    if not args.dryrun:
        config_dir = os.path.join(get_package_share_directory('tucan_sim'), 'gz_config', 'default.config')
        cmd = f'GZ_SIM_RESOURCE_PATH={args.model_store}/models gz sim -r {args.model_store}/worlds/{args.world}.sdf --gui-config {config_dir}'
        if args.headless:
            cmd = f'{cmd} -s'

        if args.gz_partition:
            cmd = f'GZ_PARTITION={args.gz_partition} {cmd}'
        if args.gz_ip:
            cmd = f'GZ_IP={args.gz_ip} {cmd}'

        try:
            process_handle = run(cmd)

            while process_handle.poll() is None:
                process_handle.wait()

        except KeyboardInterrupt:
            exit(0)

if __name__ == "__main__":
    main()
