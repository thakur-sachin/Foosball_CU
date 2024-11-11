import trimesh

# Load the STL file
mesh = trimesh.load_mesh("/home/steve/foosball_model_file/Tornado_Foosman_Friendlier.stl")

# Move the mesh to the origin by translating its centroid to (0, 0, 0)
mesh.apply_translation(-mesh.centroid)

# Save the transformed STL file
mesh.export("/home/steve/foosball_model_file/centered/centered_Tornado_Foosman_Friendlier.stl")

