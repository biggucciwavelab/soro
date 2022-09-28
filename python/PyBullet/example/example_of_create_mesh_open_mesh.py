# -*- coding: utf-8 -*-
"""
Created on Sun Jul 18 19:33:37 2021

@author: dmulr
"""

import openmesh as om
import numpy as np

mesh = om.TriMesh()


# add a a couple of vertices to the mesh
vh0 = mesh.add_vertex([0, 1, 0])
vh1 = mesh.add_vertex([1, 0, 0])
vh2 = mesh.add_vertex([2, 1, 0])
vh3 = mesh.add_vertex([0,-1, 0])
vh4 = mesh.add_vertex([2,-1, 0])


fh0 = mesh.add_face(vh0, vh1, vh2)
fh1 = mesh.add_face(vh1, vh3, vh4)
fh2 = mesh.add_face(vh0, vh3, vh1)


vh_list = [vh2, vh1, vh4]
fh3 = mesh.add_face(vh_list)


for i, vh in enumerate(mesh.vertices()):                                                                                                                                    
    print(vh.idx())
    
    
    
for vh in mesh.vv(vh0):                                                                                                                                                     
    print(vh.idx())



tc = mesh.texcoord2D(vh)


om.write_mesh('test_out.obj', mesh, vertex_tex_coord=True)