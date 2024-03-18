# Generation-of-3D-Models-and-Environments-from-Point-Clouds
With LiAR data, we seek to create 3D models automatically. By simply entering the data, we will obtain exportable models for use in different programs such as game engines for different purposes.

The document "AlgoritmoPropio" is developed in spyder with Python 3.11.7, it generates a 3D model from the introduction of a laz file, or las and and exports an fbx document that can be used in 3D editing software.

The "lidar_vectorization" document is developed to be executed in Jupyter with Python 3.10.13, from a las format document, which covers a large surface of some environment (for example, a segment of a city scanned with LiDAR) generates a map with basic 3D models, which can be used in 3D editing software for multiple purposes

Both documents use Open3D as the main library, which helps us generate 3D shapes for different purposes.
