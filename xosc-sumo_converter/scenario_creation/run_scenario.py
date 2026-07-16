import argparse
from pathlib import Path
import subprocess
import os


def main():
    parser = argparse.ArgumentParser(description='Run a newly created scenario with esmini')
    parser.add_argument("--xosc", type=Path, default=None, help="xosc file created by josm_tool")
    args = parser.parse_args()

    esmini_path = "bin/esmini"
    
    command = [
        esmini_path,
        "--window", "60", "60", "1024", "576",
        "--osc", args.xosc
    ]

    subprocess.run(command)


if __name__ == "__main__":
    main()