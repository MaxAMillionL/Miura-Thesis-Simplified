"""Run a MuJoCo MJCF XML model in a real-time viewer."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from simulation_config import load_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml", type=Path, nargs="?", default=Path(__file__).with_name("robot.xml"), help="Path to a MuJoCo MJCF XML file")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"), help="Physics configuration (default: config.json beside this script)")
    args = parser.parse_args()

    model = load_model(args.xml, args.config)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.monotonic()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.monotonic() - start)))


if __name__ == "__main__":
    main()
