# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\Koki\Google Drive (67k.tanaka@gmail.com)\PycharmProjects\chrono_solidworks_learning\Origami_robot\CAD\Unit.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'Unit_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('Mem_Out-1')
body_1.SetPos(chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0270305518063804))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(2.2536168627949e-05)
body_1.SetInertiaXX(chrono.ChVectorD(2.00386249752361e-08,3.01008464922656e-08,1.92925474675924e-08))
body_1.SetInertiaXY(chrono.ChVectorD(7.09999087402884e-25,1.13852117290588e-23,-6.77044699998839e-25))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.954264636246e-17,1.22819926360484e-18,0.0317557565878254),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Auxiliary marker (coordinate system feature)
marker_1_1 =chrono.ChMarker()
marker_1_1.SetName('Marker_Joint_Male')
body_1.AddMarker(marker_1_1)
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0120305518063804),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_1.GetCollisionModel().AddBox(0.0225,0.0175,0.0005,chrono.ChVectorD(0,0,0.0005),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('Mem_In-1')
body_2.SetPos(chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.177030551806381))
body_2.SetRot(chrono.ChQuaternionD(-3.1607606292061e-17,1.65762494845746e-33,1,5.24438621874953e-17))
body_2.SetMass(2.42276464906947e-05)
body_2.SetInertiaXX(chrono.ChVectorD(1.94637581913852e-08,2.78446735541906e-08,1.77079800210354e-08))
body_2.SetInertiaXY(chrono.ChVectorD(-5.21718101058074e-25,1.27882941073299e-23,-1.90211432046202e-16))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.01877542161376e-17,2.25322442783765e-10,0.0309065385944859),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_2.GetAssets().push_back(body_2_1_level) 

# Auxiliary marker (coordinate system feature)
marker_2_1 =chrono.ChMarker()
marker_2_1.SetName('Marker_Joint_Female')
body_2.AddMarker(marker_2_1)
marker_2_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.192030551806381),chrono.ChQuaternionD(-3.1607606292061E-17,1.65762494845746E-33,1,5.24438621874953E-17)))

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-1.73472347597681E-18,0.01775,0.0225),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=7.70988211545248E-17; mr[2,0]=0 
mr[0,1]=-1.0842021724855E-16; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_2.GetCollisionModel().AddBox(0.0225,0.016,0.0005,chrono.ChVectorD(-5.20417042793042E-18,0,0.0005),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=1; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-3.46944695195361E-18,-0.01775,0.0225),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('Test_Cylinder-1')
body_3.SetPos(chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0430305518063804))
body_3.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3.SetMass(0.00314504840550874)
body_3.SetInertiaXX(chrono.ChVectorD(1.04834946850291e-07,1.04834946850291e-07,1.57252420275437e-07))
body_3.SetInertiaXY(chrono.ChVectorD(0,2.87301610011766e-40,-3.03885286528772e-40))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.31932292919015e-19,0,0),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_3_1 =chrono.ChMarker()
marker_3_1.SetName('Marker_Connector')
body_3.AddMarker(marker_3_1)
marker_3_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0430305518063804),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_3.GetCollisionModel().AddCylinder(0.01,0.01,0.005,chrono.ChVectorD(0,0,0),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('Test_Cylinder-2')
body_4.SetPos(chrono.ChVectorD(0.0628090624328941,0.0706402242767326,0.161030551806379))
body_4.SetRot(chrono.ChQuaternionD(-3.49148133884313e-15,0,1,0))
body_4.SetMass(0.00314504840550874)
body_4.SetInertiaXX(chrono.ChVectorD(1.04834946850291e-07,1.04834946850291e-07,1.57252420275437e-07))
body_4.SetInertiaXY(chrono.ChVectorD(-2.12201961412841e-54,-3.66029260586404e-22,3.03885286528772e-40))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.31932292919015e-19,0,0),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_4.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_4_1 =chrono.ChMarker()
marker_4_1.SetName('Marker_Connector')
body_4.AddMarker(marker_4_1)
marker_4_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.0628090624328941,0.0706402242767326,0.161030551806379),chrono.ChQuaternionD(-3.49148133884313E-15,0,1,0)))

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.01,0.01,0.005,chrono.ChVectorD(0,0,0),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)




# Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Mem_In-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.085309062432893,0.0706402242767325,0.10703055180638)
dA = chrono.ChVectorD(0,0,-1)
cB = chrono.ChVectorD(0.085309062432893,0.0706402242767326,0.0970305518063806)
dB = chrono.ChVectorD(6.3215212584122e-17,-1.04887724374991e-16,1)
link_1.SetFlipped(True)
link_1.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_1.SetName("Concentric1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.085309062432893,0.0706402242767325,0.10703055180638)
cB = chrono.ChVectorD(0.085309062432893,0.0706402242767326,0.0970305518063806)
dA = chrono.ChVectorD(0,0,-1)
dB = chrono.ChVectorD(6.3215212584122e-17,-1.04887724374991e-16,1)
link_2.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_2.SetName("Concentric1")
exported_items.append(link_2)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Mem_In-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.040309062432893,0.0706402242767325,0.10703055180638)
dA = chrono.ChVectorD(0,0,-1)
cB = chrono.ChVectorD(0.040309062432893,0.0706402242767326,0.0970305518063806)
dB = chrono.ChVectorD(6.3215212584122e-17,-1.04887724374991e-16,1)
link_3.SetFlipped(True)
link_3.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_3.SetName("Concentric2")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateGeneric()
link_4.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.040309062432893,0.0706402242767325,0.10703055180638)
cB = chrono.ChVectorD(0.040309062432893,0.0706402242767326,0.0970305518063806)
dA = chrono.ChVectorD(0,0,-1)
dB = chrono.ChVectorD(6.3215212584122e-17,-1.04887724374991e-16,1)
link_4.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_4.SetName("Concentric2")
exported_items.append(link_4)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Test_Cylinder-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0120305518063804)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0480305518063804)
dB = chrono.ChVectorD(0,0,-1)
link_5.SetFlipped(True)
link_5.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_5.SetName("Concentric4")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0120305518063804)
cB = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0480305518063804)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(0,0,-1)
link_6.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_6.SetName("Concentric4")
exported_items.append(link_6)


# Mate constraint: Parallel3 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Test_Cylinder-1 ,  SW ref.type:4 (4)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.062809062432893,0.0531402242767325,0.10703055180638)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.0430305518063804)
dB = chrono.ChVectorD(0,1,0)
link_7.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_7.SetName("Parallel3")
exported_items.append(link_7)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Test_Cylinder-2 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.192030551806381)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
cB = chrono.ChVectorD(0.0628090624328941,0.0706402242767326,0.156030551806379)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_8.SetFlipped(True)
link_8.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_8.SetName("Concentric5")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.062809062432893,0.0706402242767325,0.192030551806381)
cB = chrono.ChVectorD(0.0628090624328941,0.0706402242767326,0.156030551806379)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_9.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_9.SetName("Concentric5")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Test_Cylinder-2 ,  SW ref.type:4 (4)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.062809062432893,0.0546402242767326,0.0970305518063806)
dA = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
cB = chrono.ChVectorD(0.0628090624328941,0.0706402242767326,0.161030551806379)
dB = chrono.ChVectorD(0,1,0)
link_10.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)

