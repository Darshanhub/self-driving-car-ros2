import argparse
from pathlib import Path


def convert_split(dataset_root: str, split: str, output: str):
    # Placeholder for actual conversion. Documented expectations in README.
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    with open(output, 'w', encoding='utf-8') as fp:
        fp.write('MCAP placeholder. Run the actual conversion script with nuScenes devkit.')


def main():
    parser = argparse.ArgumentParser(description='Convert nuScenes split to ROS 2 MCAP bag.')
    parser.add_argument('--dataset-root', required=True)
    parser.add_argument('--split', default='mini_val')
    parser.add_argument('--output', default='nuscenes.mcap')
    args = parser.parse_args()
    convert_split(args.dataset_root, args.split, args.output)


if __name__ == '__main__':
    main()
