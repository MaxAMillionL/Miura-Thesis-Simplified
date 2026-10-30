"""Run a MuJoCo MJCF XML model in a real-time viewer."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from simulation_config import load_model
from module_grid import positive_int


def main():
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument('-h', '--help', action='help', help='Show this help message and exit')
    parser.add_argument('-w', '--width', type=positive_int, default=1, help='Number of module columns (default: 1)')
    parser.add_argument('-l', '--length', type=positive_int, default=1, help='Number of module rows (default: 1)')
    parser.add_argument("xml", type=Path, nargs="?", default=Path(__file__).with_name("robot.xml"), help="Path to a MuJoCo MJCF XML file")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"), help="Physics configuration (default: config.json beside this script)")
    args = parser.parse_args()

    model = load_model(args.xml, args.config, width=args.width, length=args.length)
    print(f'{args.width} x {args.length} grid: {args.width * args.length} modules, {model.nu} motor controls')
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.monotonic()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.monotonic() - start)))


if __name__ == "__main__":
    main()
