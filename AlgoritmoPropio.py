# -*- coding: utf-8 -*-
"""
Created on Tue Feb 27 10:53:21 2024

@author: cahh1
"""
#Instalamos el numpy y el Open3D, para que funcione todo el codigo, se les asigna nombres especificos para poder hacer referencia en ellos dentro del codigo
import open3d as o3d; print(o3d.__version__)

import numpy as np

#Intrsuccion de inicio, no necesaria pero recomendada
print("Load a ply point cloud, print it, and render it")

#Se le especifica que tipo de archivo leera y la libreria que utilizara para leerlo
ply_point_cloud = o3d.data.PLYPointCloud()

#Se introduce el nombre del archivo que leera y se le asignara tambien un nombre en este caso pcd, se debe colocar el archivo con todo y extension
pcd = o3d.io.read_point_cloud("Downtown.ply")



#El valor anterior se metera dentro de una variable llamada downpcd para poder hacer mas calculos sin la necesidad de introducir el nombre dle archivo nuevamente
downpcd=pcd

#En esta funcion se calculan las normales del ply, esto para que se pueda generar el modelo 3d posteriormente, se pueden modificar los valores
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

#Imprime el numero de PointClouds que tiene nuestro archivo
print(pcd)

#Imprime las posiciones de los PointClouds que tiene nuestro archivo
print(np.asarray(pcd.points))

#VIsualiza el producto final solo con los point clouds
o3d.visualization.draw_geometries([pcd])

print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
print(mesh)
o3d.visualization.draw_geometries([mesh])



print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
o3d.visualization.draw_geometries([mesh])







#HAce un renderizado basico del modelo, solo crea triangulos a partir de los puntos, aun necesita mas instrucciones de codificacion

#
#distances = downpcd.compute_nearest_neighbor_distance()
#avg_dist = np.mean(distances)
#radius = 8 * avg_dist
#bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 4]))


#o3d.visualization.draw_geometries([bpa_mesh],window_name="PBA Mesh")

#se parte la malla, hasta este punto hay menos caras y se ve un poco mas smoth, pero siguen habiendo emptys

#dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

#dec_mesh.remove_degenerate_triangles()
#dec_mesh.remove_duplicated_triangles()
#dec_mesh.remove_duplicated_vertices()
#dec_mesh.remove_non_manifold_edges()

#o3d.visualization.draw_geometries([dec_mesh],window_name="BPA Mesh Post-Processed")


#Reconstruccion de la malla, le da mas volumen al modelo, pero se sigue viendo indefinido, genera mas malla de la normal
#poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=.0, scale=1.1, linear_fit=False)[0]

#o3d.visualization.draw_geometries([poisson_mesh],window_name="Mesh Poisson")

#corta la malla exedente
#bbox = pcd.get_axis_aligned_bounding_box()
#p_mesh_crop = poisson_mesh.crop(bbox)

#o3d.visualization.draw_geometries([p_mesh_crop],window_name="Mesh Poisson Cropped")

#p_mesh_crop.compute_triangle_normals()
#o3d.visualization.draw_geometries([p_mesh_crop],window_name="Mesh Poisson Crooped Normals")







