#!/usr/bin/env python3

"""
Simple Isaac Sim Robot Example
This script demonstrates how to create and control a simple robot in Isaac Sim
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np
import carb


def main():
    # Initialize the world
    world = World(stage_units_in_meters=1.0)

    # Get the assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    # Add a simple robot to the stage
    add_reference_to_stage(
        usd_path=f"{assets_root_path}/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/Robot"
    )

    # Reset the world to initialize physics
    world.reset()

    print("Simple Isaac Sim robot example initialized")
    print("Robot loaded at /World/Robot")

    # Example of how to access robot properties
    robot_prim = get_prim_at_path("/World/Robot")
    if robot_prim.IsValid():
        print(f"Robot prim is valid: {robot_prim.GetPrimTypeInfo().GetTypeName()}")

    # Example of simple control loop
    for i in range(100):  # Run for 100 steps
        if i % 10 == 0:  # Print every 10 steps
            print(f"Simulation step: {i}")

        # Step the world
        world.step(render=True)

        # Here you would typically control the robot
        # For example, setting joint positions or applying forces

    print("Simple robot control example completed")


if __name__ == "__main__":
    main()