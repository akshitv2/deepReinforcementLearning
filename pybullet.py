import pybullet as p
import pybullet_data
import time

# Connect to PyBullet (GUI or DIRECT)
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version

# Set up simulation environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf and other assets
p.setGravity(0, 0, -9.8)

# Load a ground plane
planeId = p.loadURDF("plane.urdf")

# Define the box size and shape
box_size = [0.2, 0.2, 0.2]  # Half extents in x, y, z
box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_size)
box_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=box_size, rgbaColor=[1, 0, 0, 1])

# Create the box body
box_position = [0, 0, 1]  # Starting position (x, y, z)
box_body = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=box_collision_shape,
    baseVisualShapeIndex=box_visual_shape,
    basePosition=box_position
)

# Run the simulation for a bit
for _ in range(240):  # Simulate 4 seconds at 60Hz
    p.stepSimulation()
    time.sleep(1. / 60.)

# Disconnect from simulation
p.disconnect()
