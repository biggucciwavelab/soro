# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Membrane Studies\two_node_octagon\Solidworks\two_node_6750.SLDASM


import pychrono as chrono 
import builtins 

#########################################################
import math

ang = math.pi*3/8
d = 0.05
#########################################################

shapes_dir = 'two_node_6750_study_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('node_6750-2')
####################################################################################
body_1.SetPos(chrono.ChVectorD(-d*math.cos(ang),0,-d*math.sin(ang)))
####################################################################################
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(0.05)
body_1.SetInertiaXX(chrono.ChVectorD(6.76671473757401e-06,1.04166666666667e-05,4.23920758008145e-06))
#body_1.SetInertiaXY(chrono.ChVectorD(-6.05768189150855e-23,-4.24087846081782e-22,8.51316080384283e-23))
body_1.SetInertiaXY(chrono.ChVectorD(0,0,0))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.83568009196686e-19,0.005,-4.00251349775977e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape)
#########################################################################
octagon2_texture = chrono.ChTexture()
octagon2_texture.SetTextureFilename('.\data\octB.png')
body_1_1_level.GetAssets().push_back(octagon2_texture)
#########################################################################
body_1.GetAssets().push_back(body_1_1_level) 

# Collision parameters 
body_1.GetMaterialSurfaceNSC().SetFriction(0.4);
body_1.GetMaterialSurfaceNSC().SetRollingFriction(0.01);
body_1.GetCollisionModel().SetEnvelope(0.03);
body_1.GetCollisionModel().SetSafeMargin(0.01);

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(-3.29597460435593E-17,0.01,-0.0270598050073099))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0.01,-0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0.01,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0,0.01,0.0270598050073098))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182544,0.01,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545,0.01,-0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(-3.29597460435593E-17,0,-0.0270598050073099))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0,-0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0,0,0.0270598050073098))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182544,0,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545,0,-0.0191341716182545))
body_1.GetCollisionModel().AddConvexHull(pt_vect)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('node_6750-1')
body_2.SetPos(chrono.ChVectorD(0,0,0))
body_2.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2.SetMass(0.05)
body_2.SetInertiaXX(chrono.ChVectorD(6.76671473757401e-06,1.04166666666667e-05,4.23920758008145e-06))
#body_2.SetInertiaXY(chrono.ChVectorD(-6.05768189150855e-23,-4.24087846081782e-22,8.51316080384283e-23))
body_2.SetInertiaXY(chrono.ChVectorD(0,0,0))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.83568009196686e-19,0.005,-4.00251349775977e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
#########################################################################
octagon1_texture = chrono.ChTexture()
octagon1_texture.SetTextureFilename('.\data\octA.png')
body_1_1_level.GetAssets().push_back(octagon1_texture)
#########################################################################
body_2.GetAssets().push_back(body_1_1_level) 

# Collision parameters 
body_2.GetMaterialSurfaceNSC().SetFriction(0.4);
body_2.GetMaterialSurfaceNSC().SetRollingFriction(0.01);
body_2.GetCollisionModel().SetEnvelope(0.03);
body_2.GetCollisionModel().SetSafeMargin(0.01);

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(-3.29597460435593E-17,0.01,-0.0270598050073099))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0.01,-0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0.01,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0,0.01,0.0270598050073098))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182544,0.01,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545,0.01,-0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(-3.29597460435593E-17,0,-0.0270598050073099))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0,-0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0.0191341716182545,0,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(0,0,0.0270598050073098))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182544,0,0.0191341716182545))
pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545,0,-0.0191341716182545))
body_2.GetCollisionModel().AddConvexHull(pt_vect)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('ground-1')
body_3.SetPos(chrono.ChVectorD(0,0,0))
body_3.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3.SetMass(4.8)
body_3.SetInertiaXX(chrono.ChVectorD(0.06404,0.208,0.14404))
body_3.SetInertiaXY(chrono.ChVectorD(-1.80700362080917e-19,9.20328628121562e-18,-1.89120998953888e-19))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(9.88340630401578e-17,-0.005,6.18853565036622e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
#########################################################################
ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename('.\data\ground.png')
body_3_1_level.GetAssets().push_back(ground_texture)
#########################################################################
body_3.GetAssets().push_back(body_3_1_level) 

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=9.25185853854297E-17 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.3,0.2,0.005,chrono.ChVectorD(1.11022302462516E-16,-0.005,8.32667268468867E-17),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)




# Mate constraint: Coincident2 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: ground-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: two_node_6750 ,  SW ref.type:4 (4)

link_1 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(0,1,0)
link_1.Initialize(body_3,body_0,False,cA,cB,dB)
link_1.SetDistance(0)
link_1.SetName("Coincident2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0,0,0)
dB = chrono.ChVectorD(0,1,0)
link_2.Initialize(body_3,body_0,False,cA,cB,dA,dB)
link_2.SetName("Coincident2")
exported_items.append(link_2)


# Mate constraint: Coincident3 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: ground-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: two_node_6750 ,  SW ref.type:4 (4)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(1,0,0)
dB = chrono.ChVectorD(1,0,0)
link_3.Initialize(body_3,body_0,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident3")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(0,0,0)
dB = chrono.ChVectorD(1,0,0)
link_4.Initialize(body_3,body_0,False,cA,cB,dA,dB)
link_4.SetName("Coincident3")
exported_items.append(link_4)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: ground-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: two_node_6750 ,  SW ref.type:4 (4)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(0,0,1)
link_5.Initialize(body_3,body_0,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident4")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0,0,0)
dB = chrono.ChVectorD(0,0,1)
link_6.Initialize(body_3,body_0,False,cA,cB,dA,dB)
link_6.SetName("Coincident4")
exported_items.append(link_6)


# Mate constraint: Coincident5 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: node_6750-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: two_node_6750 ,  SW ref.type:4 (4)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(0,0,1)
link_7.Initialize(body_2,body_0,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident5")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0,0,0)
dB = chrono.ChVectorD(0,0,1)
link_8.Initialize(body_2,body_0,False,cA,cB,dA,dB)
link_8.SetName("Coincident5")
exported_items.append(link_8)


# Mate constraint: Coincident6 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: node_6750-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: two_node_6750 ,  SW ref.type:4 (4)

link_9 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(0,1,0)
link_9.Initialize(body_2,body_0,False,cA,cB,dB)
link_9.SetDistance(0)
link_9.SetName("Coincident6")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0,0,0)
dB = chrono.ChVectorD(0,1,0)
link_10.Initialize(body_2,body_0,False,cA,cB,dA,dB)
link_10.SetName("Coincident6")
exported_items.append(link_10)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: node_6750-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: two_node_6750 ,  SW ref.type:4 (4)

link_11 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(1,0,0)
dB = chrono.ChVectorD(1,0,0)
link_11.Initialize(body_2,body_0,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident7")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(0,0,0)
dB = chrono.ChVectorD(1,0,0)
link_12.Initialize(body_2,body_0,False,cA,cB,dA,dB)
link_12.SetName("Coincident7")
exported_items.append(link_12)

