import trimesh

# Load the obj file
mesh = trimesh.load('/home/steve/mujoco_deepmind/model/foosball/asset/foosball_table/foosball_table_nofloor.obj')

# Compute the inertia tensor
inertia_tensor = mesh.moment_inertia
mass = mesh.mass  # Use your assigned mass or calculate based on volume and density

# Center of mass
com = mesh.center_mass

print("Inertia Tensor:", inertia_tensor)
print("Center of Mass:", com)
print("Mass:", mass)