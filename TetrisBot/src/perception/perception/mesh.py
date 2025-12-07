import trimesh
import io
import os
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point

def get_collision_mesh(logger, file_path):
    """
    Loads a 3D mesh file and converts it to a ROS shape_msgs/Mesh.
    """
    logger.info(f'Directory: {os.listdir()}')
    logger.info(f'CWD: {os.getcwd()}')
    mesh_data = None
    try:
        with open(file_path, 'rb') as f:
            mesh_data = trimesh.load(file_obj=f, file_type='stl')

        # mesh_data = trimesh.util.concatenate(tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces) for g in mesh_data.geometry.values()))
    except Exception as e:
        logger.info(f"Failed to load mesh from {file_path}: {e}")
        return None

    # Create the ROS Mesh message
    mesh = Mesh()

    # 1. Populate Vertices
    # trimesh.vertices is an Nx3 numpy array
    for vertex in mesh_data.vertices:
        p = Point()
        p.x = float(vertex[0])
        p.y = float(vertex[1])
        p.z = float(vertex[2])
        mesh.vertices.append(p)

    # 2. Populate Triangles (Faces)
    # trimesh.faces is an Nx3 numpy array of vertex indices
    for face in mesh_data.faces:
        triangle = MeshTriangle()
        # Ensure indices are Python ints (not numpy types) for ROS serialization
        triangle.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
        mesh.triangles.append(triangle)

    return mesh