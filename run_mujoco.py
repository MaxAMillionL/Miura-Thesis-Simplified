"""Run a MuJoCo MJCF XML model in a real-time viewer."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml", type=Path, help="Path to a MuJoCo MJCF XML file")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.xml.resolve()))
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.monotonic()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.monotonic() - start)))


if __name__ == "__main__":
    main()
