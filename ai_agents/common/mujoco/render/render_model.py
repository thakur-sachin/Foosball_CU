import mujoco
from mujoco import viewer
import argparse

DEFAULT_PATH = "/Research/Foosball_CU/foosball_sim/v1/foosball_sim.xml"

def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--xml_file_path", type=str, default=DEFAULT_PATH)

    args = arg_parser.parse_args()
    xml_file_path = args.xml_file_path

    print("Loading model from XML file:", xml_file_path)

    model = mujoco.MjModel.from_xml_path(xml_file_path)
    data = mujoco.MjData(model)

    with viewer.launch(model, data) as v:
        while v.is_running():
            mujoco.mj_step(model, data)

            v.render()

if __name__ == "__main__":
    main()