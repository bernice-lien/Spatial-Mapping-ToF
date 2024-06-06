#bernice lien 400382544
import numpy as np
import open3d as o3d

filename = "tof_radar.xyz"

pcd = o3d.io.read_point_cloud(filename, format='xyz')
o3d.visualization.draw_geometries([pcd], window_name = "Point Cloud", width=800, height=800)
numpoints = len(pcd.points)

lines = []

depth = 0
while (depth < (numpoints/32)):
    lines.append([0+(32*depth),31+(32*depth)])
    for x in range(31):
        lines.append([x+(depth*32),x+1+(depth*32)])
    depth += 1
    
depth = 1
while (depth < (numpoints/32)):
    lines.append([0+(32*(depth-1)),0+(32*depth)]) 
    lines.append([4+(32*(depth-1)),4+(32*depth)])
    lines.append([8+(32*(depth-1)),8+(32*depth)])
    lines.append([12+(32*(depth-1)),12+(32*depth)])
    lines.append([16+(32*(depth-1)),16+(32*depth)]) 
    lines.append([20+(32*(depth-1)),20+(32*depth)])
    lines.append([24+(32*(depth-1)),24+(32*depth)])
    lines.append([28+(32*(depth-1)),28+(32*depth)])
    depth += 1
    
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))

o3d.visualization.draw_geometries([line_set])