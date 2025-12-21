---
sidebar_position: 2
title: "Isaac Sim Fundamentals"
---

import CodeSandbox from '@site/src/components/CodeSandbox';

# Isaac Sim Fundamentals

## Introduction to Isaac Sim

Isaac Sim is NVIDIA's robotics simulation environment built on the Omniverse platform. It provides photorealistic simulation with accurate physics and is tightly integrated with the Isaac ecosystem.

## Setting up a Basic Scene

Here's how to create a basic scene in Isaac Sim using Python:

```python
import omni
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom
import carb

# Create a new stage
stage = omni.usd.get_context().get_stage()

# Create a ground plane
plane_path = Sdf.Path("/World/ground_plane")
omni.kit.commands.execute(
    "CreateMeshPrimWithDefaultXform",
    prim_type="Plane",
    prim_path=plane_path,
    u_patches=1,
    v_patches=1,
    width=10.0,
    height=10.0
)

# Add physics to the plane
plane_prim = stage.GetPrimAtPath(plane_path)
omni.kit.commands.execute(
    "AddPhysicsMaterial",
    path=plane_path.AppendChild("physicsMaterial"),
    dynamic_friction=0.5,
    static_friction=0.5,
    restitution=0.1
)

# Create a cube
cube_path = Sdf.Path("/World/cube")
omni.kit.commands.execute(
    "CreateMeshPrimWithDefaultXform",
    prim_type="Cube",
    prim_path=cube_path,
    width=0.5,
    height=0.5,
    depth=0.5
)

# Position the cube above the ground
cube_prim = stage.GetPrimAtPath(cube_path)
xform = UsdGeom.Xformable(cube_prim)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))

# Add rigid body physics to the cube
omni.kit.commands.execute(
    "AddRigidBody",
    path=cube_path,
    approximation_shape="convexHull"
)

print("Basic scene created with ground plane and falling cube")
```

## Working with Robots in Isaac Sim

Isaac Sim provides excellent support for complex robot models. Here's an example of loading and controlling a robot:

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create the world
my_world = World(stage_units_in_meters=1.0)

# Add a robot to the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
else:
    # Load a sample robot
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/Robot"
    )

    print("Robot loaded successfully")

# Reset the world
my_world.reset()

# Example of controlling the robot (simplified)
def move_robot():
    print("Robot movement control example")
    # In a real scenario, you would control the robot's joints here
    # For example: robot.get_articulation_controller().apply_efforts(joint_efforts)

move_robot()
```

<CodeSandbox
  title="Isaac Sim Scene Setup"
  language="python"
  code={`import omni
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom
import carb

# Create a new stage
stage = omni.usd.get_context().get_stage()

# Create a ground plane
plane_path = Sdf.Path("/World/ground_plane")
omni.kit.commands.execute(
    "CreateMeshPrimWithDefaultXform",
    prim_type="Plane",
    prim_path=plane_path,
    u_patches=1,
    v_patches=1,
    width=10.0,
    height=10.0
)

# Add physics to the plane
plane_prim = stage.GetPrimAtPath(plane_path)
omni.kit.commands.execute(
    "AddPhysicsMaterial",
    path=plane_path.AppendChild("physicsMaterial"),
    dynamic_friction=0.5,
    static_friction=0.5,
    restitution=0.1
)

# Create a cube
cube_path = Sdf.Path("/World/cube")
omni.kit.commands.execute(
    "CreateMeshPrimWithDefaultXform",
    prim_type="Cube",
    prim_path=cube_path,
    width=0.5,
    height=0.5,
    depth=0.5
)

# Position the cube above the ground
cube_prim = stage.GetPrimAtPath(cube_path)
xform = UsdGeom.Xformable(cube_prim)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))

# Add rigid body physics to the cube
omni.kit.commands.execute(
    "AddRigidBody",
    path=cube_path,
    approximation_shape="convexHull"
)

print("Basic scene created with ground plane and falling cube")`}
/>

<CodeSandbox
  title="Isaac Sim Robot Control"
  language="python"
  code={`import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create the world
my_world = World(stage_units_in_meters=1.0)

# Add a robot to the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
else:
    # Load a sample robot
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/Robot"
    )

    print("Robot loaded successfully")

# Reset the world
my_world.reset()

# Example of controlling the robot (simplified)
def move_robot():
    print("Robot movement control example")
    # In a real scenario, you would control the robot's joints here
    # For example: robot.get_articulation_controller().apply_efforts(joint_efforts)

move_robot()`}
/>