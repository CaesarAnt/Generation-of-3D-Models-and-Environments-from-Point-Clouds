# -*- coding: utf-8 -*-
"""
Created on Tue Feb 27 10:53:21 2024

@author: cahh1
"""
#Instalamos el numpy y el Open3D, para que funcione todo el codigo, se 
#les asigna nombres especificos para poder hacer referencia en ellos dentro del codigo

import numpy as np
import open3d as o3d

#create paths and load data
input_path="models/"
output_path="results/"
dataname="sample_w_normals.xyz"
point_cloud= np.loadtxt(input_path+dataname,skiprows=1)

#Format to open3d usable objects
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
pcd.colors = o3d.utility.Vector3dVector(point_cloud[:,3:6]/255)
pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,6:9])

#radius determination
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

#computing the mesh
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))

o3d.visualization.draw_geometries([bpa_mesh], window_name="BPA_mesh")

#decimating the mesh
dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

#computing the mesh
poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]

#cropping
bbox = pcd.get_axis_aligned_bounding_box()
p_mesh_crop = poisson_mesh.crop(bbox)

#export
o3d.io.write_triangle_mesh(output_path+"bpa_mesh.ply", dec_mesh)
o3d.io.write_triangle_mesh(output_path+"p_mesh_c.ply", p_mesh_crop)

#function creation
def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods={}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        o3d.io.write_triangle_mesh(path+"lod_"+str(i)+extension, mesh_lod)
        mesh_lods[i]=mesh_lod
    print("generation of "+str(i)+" LoD successful")
    return mesh_lods

#execution of function
my_lods = lod_mesh_export(bpa_mesh, [10000], ".ply", output_path)

#execution of function
my_lods2 = lod_mesh_export(bpa_mesh, [8000], ".ply", output_path)






