# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\Koki\Google Drive (67k.tanaka@gmail.com)\PycharmProjects\chrono_solidworks_learning\Origami_robot\CAD\Env.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'Env_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('Unit-1/Mem_Out-1')
body_1.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,1.01335459153996))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(2.3526168627949e-05)
body_1.SetInertiaXX(chrono.ChVectorD(2.22747106420029e-08,3.23369321590324e-08,1.94410474675924e-08))
body_1.SetInertiaXY(chrono.ChVectorD(7.05091446269127e-25,8.22680875349612e-24,-2.90234994386466e-25))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.82984114745205e-17,2.3740741553963e-18,0.0337648725525006),chrono.ChQuaternionD(1,0,0,0)))

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
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539962),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(0.015,0.015,0.0005,chrono.ChVectorD(0,0,0.0795),mr)
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
body_2.SetName('Unit-1/Mem_In-1')
body_2.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,1.16335459153996))
body_2.SetRot(chrono.ChQuaternionD(-3.1607606292061e-17,1.65762494845746e-33,1,5.24438621874953e-17))
body_2.SetMass(2.52176464906947e-05)
body_2.SetInertiaXX(chrono.ChVectorD(2.17840275474413e-08,3.01649429102467e-08,1.78564800210354e-08))
body_2.SetInertiaXY(chrono.ChVectorD(-5.201409469241e-25,1.20143892725788e-23,-2.00625589757305e-16))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-1.94136927445332e-17,2.16476683692275e-10,0.0328142315589825),chrono.ChQuaternionD(1,0,0,0)))

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
marker_2_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,1.17835459153996),chrono.ChQuaternionD(-3.1607606292061E-17,1.65762494845746E-33,1,5.24438621874953E-17)))

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=7.70988211545248E-17; mr[2,0]=0 
mr[0,1]=-1.0842021724855E-16; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_2.GetCollisionModel().AddBox(0.0225,0.016,0.0005,chrono.ChVectorD(-5.20417042793042E-18,0,0.0005),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(0.015,0.015,0.0005,chrono.ChVectorD(0,0,0.0795),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=1; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-3.46944695195361E-18,-0.01775,0.0225),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-1.73472347597681E-18,0.01775,0.0225),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('Unit-1/Test_Cylinder-1')
body_3.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,1.02935459153996))
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
marker_3_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,1.02935459153996),chrono.ChQuaternionD(1,0,0,0)))

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
body_4.SetName('Unit-1/Test_Cylinder-2')
body_4.SetPos(chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14735459153995))
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
marker_4_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14735459153995),chrono.ChQuaternionD(-3.49148133884313E-15,0,1,0)))

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



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('Floor-1')
body_5.SetPos(chrono.ChVectorD(0.465865329198189,0.393833988458082,0.804002872779893))
body_5.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_5.SetMass(400.44)
body_5.SetInertiaXX(chrono.ChVectorD(133.8137,266.96,133.8137))
body_5.SetInertiaXY(chrono.ChVectorD(0,0,0))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,-0.045,0),chrono.ChQuaternionD(1,0,0,0)))
body_5.SetBodyFixed(True)

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_5.GetAssets().push_back(body_5_1_level) 

# Collision shapes 
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_5.GetCollisionModel().AddBox(1,1,0.05,chrono.ChVectorD(0,-0.045,0),mr)
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(True)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('Unit-2/Test_Cylinder-2')
body_6.SetPos(chrono.ChVectorD(0.472399212849674,0.417333988458083,0.967354591539953))
body_6.SetRot(chrono.ChQuaternionD(-3.49148133884313e-15,0,1,0))
body_6.SetMass(0.00314504840550874)
body_6.SetInertiaXX(chrono.ChVectorD(1.04834946850291e-07,1.04834946850291e-07,1.57252420275437e-07))
body_6.SetInertiaXY(chrono.ChVectorD(-2.12201961412841e-54,-3.66029260586404e-22,3.03885286528772e-40))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.31932292919015e-19,0,0),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_6.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_6_1 =chrono.ChMarker()
marker_6_1.SetName('Marker_Connector')
body_6.AddMarker(marker_6_1)
marker_6_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849674,0.417333988458083,0.967354591539953),chrono.ChQuaternionD(-3.49148133884313E-15,0,1,0)))

# Collision shapes 
body_6.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_6.GetCollisionModel().AddCylinder(0.01,0.01,0.005,chrono.ChVectorD(0,0,0),mr)
body_6.GetCollisionModel().BuildModel()
body_6.SetCollide(True)

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRef()
body_7.SetName('Unit-2/Mem_In-1')
body_7.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.983354591539962))
body_7.SetRot(chrono.ChQuaternionD(-3.1607606292061e-17,1.65762494845746e-33,1,5.24438621874953e-17))
body_7.SetMass(2.52176464906947e-05)
body_7.SetInertiaXX(chrono.ChVectorD(2.17840275474413e-08,3.01649429102467e-08,1.78564800210354e-08))
body_7.SetInertiaXY(chrono.ChVectorD(-5.201409469241e-25,1.20143892725788e-23,-2.00625589757305e-16))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-1.94136927445332e-17,2.16476683692275e-10,0.0328142315589825),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_7.GetAssets().push_back(body_2_1_level) 

# Auxiliary marker (coordinate system feature)
marker_7_1 =chrono.ChMarker()
marker_7_1.SetName('Marker_Joint_Female')
body_7.AddMarker(marker_7_1)
marker_7_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539962),chrono.ChQuaternionD(-3.1607606292061E-17,1.65762494845746E-33,1,5.24438621874953E-17)))

# Collision shapes 
body_7.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=7.70988211545248E-17; mr[2,0]=0 
mr[0,1]=-1.0842021724855E-16; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_7.GetCollisionModel().AddBox(0.0225,0.016,0.0005,chrono.ChVectorD(-5.20417042793042E-18,0,0.0005),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_7.GetCollisionModel().AddBox(0.015,0.015,0.0005,chrono.ChVectorD(0,0,0.0795),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=1; mr[2,2]=0 
body_7.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-3.46944695195361E-18,-0.01775,0.0225),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_7.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-1.73472347597681E-18,0.01775,0.0225),mr)
body_7.GetCollisionModel().BuildModel()
body_7.SetCollide(True)

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRef()
body_8.SetName('Unit-2/Test_Cylinder-1')
body_8.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.849354591539961))
body_8.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_8.SetMass(0.00314504840550874)
body_8.SetInertiaXX(chrono.ChVectorD(1.04834946850291e-07,1.04834946850291e-07,1.57252420275437e-07))
body_8.SetInertiaXY(chrono.ChVectorD(0,2.87301610011766e-40,-3.03885286528772e-40))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.31932292919015e-19,0,0),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_8.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_8_1 =chrono.ChMarker()
marker_8_1.SetName('Marker_Connector')
body_8.AddMarker(marker_8_1)
marker_8_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.849354591539961),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_8.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.01,0.01,0.005,chrono.ChVectorD(0,0,0),mr)
body_8.GetCollisionModel().BuildModel()
body_8.SetCollide(True)

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRef()
body_9.SetName('Unit-2/Mem_Out-1')
body_9.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.833354591539961))
body_9.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_9.SetMass(2.3526168627949e-05)
body_9.SetInertiaXX(chrono.ChVectorD(2.22747106420029e-08,3.23369321590324e-08,1.94410474675924e-08))
body_9.SetInertiaXY(chrono.ChVectorD(7.05091446269127e-25,8.22680875349612e-24,-2.90234994386466e-25))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.82984114745205e-17,2.3740741553963e-18,0.0337648725525006),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_9.GetAssets().push_back(body_1_1_level) 

# Auxiliary marker (coordinate system feature)
marker_9_1 =chrono.ChMarker()
marker_9_1.SetName('Marker_Joint_Male')
body_9.AddMarker(marker_9_1)
marker_9_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_9.GetCollisionModel().AddBox(0.015,0.015,0.0005,chrono.ChVectorD(0,0,0.0795),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_9.GetCollisionModel().AddBox(0.0225,0.0175,0.0005,chrono.ChVectorD(0,0,0.0005),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(True)

exported_items.append(body_9)



# Rigid body part
body_10= chrono.ChBodyAuxRef()
body_10.SetName('Unit-3/Mem_In-1')
body_10.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.803354591539961))
body_10.SetRot(chrono.ChQuaternionD(-3.1607606292061e-17,1.65762494845746e-33,1,5.24438621874953e-17))
body_10.SetMass(2.52176464906947e-05)
body_10.SetInertiaXX(chrono.ChVectorD(2.17840275474413e-08,3.01649429102467e-08,1.78564800210354e-08))
body_10.SetInertiaXY(chrono.ChVectorD(-5.201409469241e-25,1.20143892725788e-23,-2.00625589757305e-16))
body_10.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-1.94136927445332e-17,2.16476683692275e-10,0.0328142315589825),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_10.GetAssets().push_back(body_2_1_level) 

# Auxiliary marker (coordinate system feature)
marker_10_1 =chrono.ChMarker()
marker_10_1.SetName('Marker_Joint_Female')
body_10.AddMarker(marker_10_1)
marker_10_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961),chrono.ChQuaternionD(-3.1607606292061E-17,1.65762494845746E-33,1,5.24438621874953E-17)))

# Collision shapes 
body_10.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=7.70988211545248E-17; mr[2,0]=0 
mr[0,1]=-1.0842021724855E-16; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_10.GetCollisionModel().AddBox(0.0225,0.016,0.0005,chrono.ChVectorD(-5.20417042793042E-18,0,0.0005),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_10.GetCollisionModel().AddBox(0.015,0.015,0.0005,chrono.ChVectorD(0,0,0.0795),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=0; mr[1,2]=1; mr[2,2]=0 
body_10.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-3.46944695195361E-18,-0.01775,0.0225),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_10.GetCollisionModel().AddBox(0.015,0.0075,0.000750000000000001,chrono.ChVectorD(-1.73472347597681E-18,0.01775,0.0225),mr)
body_10.GetCollisionModel().BuildModel()
body_10.SetCollide(True)

exported_items.append(body_10)



# Rigid body part
body_11= chrono.ChBodyAuxRef()
body_11.SetName('Unit-3/Test_Cylinder-2')
body_11.SetPos(chrono.ChVectorD(0.472399212849673,0.417333988458083,0.787354591539953))
body_11.SetRot(chrono.ChQuaternionD(-3.49148133884313e-15,0,1,0))
body_11.SetMass(0.00314504840550874)
body_11.SetInertiaXX(chrono.ChVectorD(1.04834946850291e-07,1.04834946850291e-07,1.57252420275437e-07))
body_11.SetInertiaXY(chrono.ChVectorD(-2.12201961412841e-54,-3.66029260586404e-22,3.03885286528772e-40))
body_11.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.31932292919015e-19,0,0),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_11.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_11_1 =chrono.ChMarker()
marker_11_1.SetName('Marker_Connector')
body_11.AddMarker(marker_11_1)
marker_11_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849673,0.417333988458083,0.787354591539953),chrono.ChQuaternionD(-3.49148133884313E-15,0,1,0)))

# Collision shapes 
body_11.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.01,0.01,0.005,chrono.ChVectorD(0,0,0),mr)
body_11.GetCollisionModel().BuildModel()
body_11.SetCollide(True)

exported_items.append(body_11)



# Rigid body part
body_12= chrono.ChBodyAuxRef()
body_12.SetName('Unit-3/Mem_Out-1')
body_12.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.653354591539961))
body_12.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_12.SetMass(2.3526168627949e-05)
body_12.SetInertiaXX(chrono.ChVectorD(2.22747106420029e-08,3.23369321590324e-08,1.94410474675924e-08))
body_12.SetInertiaXY(chrono.ChVectorD(7.05091446269127e-25,8.22680875349612e-24,-2.90234994386466e-25))
body_12.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.82984114745205e-17,2.3740741553963e-18,0.0337648725525006),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_12.GetAssets().push_back(body_1_1_level) 

# Auxiliary marker (coordinate system feature)
marker_12_1 =chrono.ChMarker()
marker_12_1.SetName('Marker_Joint_Male')
body_12.AddMarker(marker_12_1)
marker_12_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.638354591539961),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_12.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_12.GetCollisionModel().AddBox(0.015,0.015,0.0005,chrono.ChVectorD(0,0,0.0795),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_12.GetCollisionModel().AddBox(0.0225,0.0175,0.0005,chrono.ChVectorD(0,0,0.0005),mr)
body_12.GetCollisionModel().BuildModel()
body_12.SetCollide(True)

exported_items.append(body_12)



# Rigid body part
body_13= chrono.ChBodyAuxRef()
body_13.SetName('Unit-3/Test_Cylinder-1')
body_13.SetPos(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.669354591539961))
body_13.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_13.SetMass(0.00314504840550874)
body_13.SetInertiaXX(chrono.ChVectorD(1.04834946850291e-07,1.04834946850291e-07,1.57252420275437e-07))
body_13.SetInertiaXY(chrono.ChVectorD(0,2.87301610011766e-40,-3.03885286528772e-40))
body_13.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.31932292919015e-19,0,0),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_13.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_13_1 =chrono.ChMarker()
marker_13_1.SetName('Marker_Connector')
body_13.AddMarker(marker_13_1)
marker_13_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.472399212849667,0.417333988458083,0.669354591539961),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_13.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.01,0.01,0.005,chrono.ChVectorD(0,0,0),mr)
body_13.GetCollisionModel().BuildModel()
body_13.SetCollide(True)

exported_items.append(body_13)




# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Unit-1/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Unit-1/Test_Cylinder-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539962)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.03435459153996)
dB = chrono.ChVectorD(0,0,-1)
link_1.SetFlipped(True)
link_1.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_1.SetName("Concentric4")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539962)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.03435459153996)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(0,0,-1)
link_2.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_2.SetName("Concentric4")
exported_items.append(link_2)


# Mate constraint: Parallel3 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Unit-1/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Unit-1/Test_Cylinder-1 ,  SW ref.type:4 (4)

link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.399833988458083,1.09335459153996)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.02935459153996)
dB = chrono.ChVectorD(0,1,0)
link_3.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_3.SetName("Parallel3")
exported_items.append(link_3)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Unit-1/Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Unit-1/Test_Cylinder-2 ,  SW ref.type:2 (2)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.17835459153996)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
cB = chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14235459153995)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_4.SetFlipped(True)
link_4.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_4.SetName("Concentric5")
exported_items.append(link_4)

link_5 = chrono.ChLinkMateGeneric()
link_5.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.17835459153996)
cB = chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14235459153995)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_5.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_5.SetName("Concentric5")
exported_items.append(link_5)


# Mate constraint: Parallel4 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Unit-1/Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Unit-1/Test_Cylinder-2 ,  SW ref.type:4 (4)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.401333988458083,1.08335459153996)
dA = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
cB = chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14735459153995)
dB = chrono.ChVectorD(0,1,0)
link_6.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_6.SetName("Parallel4")
exported_items.append(link_6)


# Mate constraint: Concentric9 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: Unit-1/Test_Cylinder-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Unit-1/Test_Cylinder-2 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.03435459153996)
dA = chrono.ChVectorD(0,0,-1)
cB = chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14235459153995)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_7.SetFlipped(True)
link_7.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_7.SetName("Concentric9")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateGeneric()
link_8.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.03435459153996)
cB = chrono.ChVectorD(0.472399212849675,0.417333988458083,1.14235459153995)
dA = chrono.ChVectorD(0,0,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_8.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_8.SetName("Concentric9")
exported_items.append(link_8)


# Mate constraint: Concentric10 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Unit-1/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Unit-1/Mem_In-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539962)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.17835459153996)
dB = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
link_9.SetFlipped(True)
link_9.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_9.SetName("Concentric10")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539962)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.17835459153996)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
link_10.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_10.SetName("Concentric10")
exported_items.append(link_10)


# Mate constraint: Parallel5 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Unit-1/Mem_Out-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_2 , SW name: Unit-1/Mem_In-1 ,  SW ref.type:4 (4)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.01335459153996)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.16335459153996)
dB = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
link_11.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_11.SetName("Parallel5")
exported_items.append(link_11)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Unit-1/Mem_Out-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_2 , SW name: Unit-1/Mem_In-1 ,  SW ref.type:4 (4)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.01335459153996)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,1.16335459153996)
dB = chrono.ChVectorD(-1,7.79187653482018e-49,6.3215212584122e-17)
link_12.SetFlipped(True)
link_12.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_12.SetName("Parallel6")
exported_items.append(link_12)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Unit-2/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Unit-2/Test_Cylinder-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.854354591539961)
dB = chrono.ChVectorD(0,0,-1)
link_1.SetFlipped(True)
link_1.Initialize(body_9,body_8,False,cA,cB,dA,dB)
link_1.SetName("Concentric4")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.854354591539961)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(0,0,-1)
link_2.Initialize(body_9,body_8,False,cA,cB,dA,dB)
link_2.SetName("Concentric4")
exported_items.append(link_2)


# Mate constraint: Parallel3 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_9 , SW name: Unit-2/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Unit-2/Test_Cylinder-1 ,  SW ref.type:4 (4)

link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.399833988458083,0.913354591539961)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.849354591539961)
dB = chrono.ChVectorD(0,1,0)
link_3.Initialize(body_9,body_8,False,cA,cB,dA,dB)
link_3.SetName("Parallel3")
exported_items.append(link_3)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: Unit-2/Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Unit-2/Test_Cylinder-2 ,  SW ref.type:2 (2)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539961)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
cB = chrono.ChVectorD(0.472399212849674,0.417333988458083,0.962354591539953)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_4.SetFlipped(True)
link_4.Initialize(body_7,body_6,False,cA,cB,dA,dB)
link_4.SetName("Concentric5")
exported_items.append(link_4)

link_5 = chrono.ChLinkMateGeneric()
link_5.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539961)
cB = chrono.ChVectorD(0.472399212849674,0.417333988458083,0.962354591539953)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_5.Initialize(body_7,body_6,False,cA,cB,dA,dB)
link_5.SetName("Concentric5")
exported_items.append(link_5)


# Mate constraint: Parallel4 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: Unit-2/Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Unit-2/Test_Cylinder-2 ,  SW ref.type:4 (4)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.401333988458083,0.903354591539961)
dA = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
cB = chrono.ChVectorD(0.472399212849674,0.417333988458083,0.967354591539953)
dB = chrono.ChVectorD(0,1,0)
link_6.Initialize(body_7,body_6,False,cA,cB,dA,dB)
link_6.SetName("Parallel4")
exported_items.append(link_6)


# Mate constraint: Concentric9 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: Unit-2/Test_Cylinder-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Unit-2/Test_Cylinder-2 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.854354591539961)
dA = chrono.ChVectorD(0,0,-1)
cB = chrono.ChVectorD(0.472399212849674,0.417333988458083,0.962354591539953)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_7.SetFlipped(True)
link_7.Initialize(body_8,body_6,False,cA,cB,dA,dB)
link_7.SetName("Concentric9")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateGeneric()
link_8.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.854354591539961)
cB = chrono.ChVectorD(0.472399212849674,0.417333988458083,0.962354591539953)
dA = chrono.ChVectorD(0,0,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_8.Initialize(body_8,body_6,False,cA,cB,dA,dB)
link_8.SetName("Concentric9")
exported_items.append(link_8)


# Mate constraint: Concentric10 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Unit-2/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: Unit-2/Mem_In-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539961)
dB = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
link_9.SetFlipped(True)
link_9.Initialize(body_9,body_7,False,cA,cB,dA,dB)
link_9.SetName("Concentric10")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.998354591539961)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
link_10.Initialize(body_9,body_7,False,cA,cB,dA,dB)
link_10.SetName("Concentric10")
exported_items.append(link_10)


# Mate constraint: Parallel5 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_9 , SW name: Unit-2/Mem_Out-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_7 , SW name: Unit-2/Mem_In-1 ,  SW ref.type:4 (4)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.833354591539961)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.983354591539962)
dB = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
link_11.Initialize(body_9,body_7,False,cA,cB,dA,dB)
link_11.SetName("Parallel5")
exported_items.append(link_11)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Unit-2/Mem_Out-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_7 , SW name: Unit-2/Mem_In-1 ,  SW ref.type:4 (4)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.833354591539961)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.983354591539962)
dB = chrono.ChVectorD(-1,7.79187653482018e-49,6.3215212584122e-17)
link_12.SetFlipped(True)
link_12.Initialize(body_9,body_7,False,cA,cB,dA,dB)
link_12.SetName("Parallel6")
exported_items.append(link_12)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Unit-3/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: Unit-3/Test_Cylinder-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.638354591539961)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.674354591539961)
dB = chrono.ChVectorD(0,0,-1)
link_1.SetFlipped(True)
link_1.Initialize(body_12,body_13,False,cA,cB,dA,dB)
link_1.SetName("Concentric4")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.638354591539961)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.674354591539961)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(0,0,-1)
link_2.Initialize(body_12,body_13,False,cA,cB,dA,dB)
link_2.SetName("Concentric4")
exported_items.append(link_2)


# Mate constraint: Parallel3 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_12 , SW name: Unit-3/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: Unit-3/Test_Cylinder-1 ,  SW ref.type:4 (4)

link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.399833988458083,0.733354591539961)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.669354591539961)
dB = chrono.ChVectorD(0,1,0)
link_3.Initialize(body_12,body_13,False,cA,cB,dA,dB)
link_3.SetName("Parallel3")
exported_items.append(link_3)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: Unit-3/Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: Unit-3/Test_Cylinder-2 ,  SW ref.type:2 (2)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
cB = chrono.ChVectorD(0.472399212849673,0.417333988458083,0.782354591539953)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_4.SetFlipped(True)
link_4.Initialize(body_10,body_11,False,cA,cB,dA,dB)
link_4.SetName("Concentric5")
exported_items.append(link_4)

link_5 = chrono.ChLinkMateGeneric()
link_5.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
cB = chrono.ChVectorD(0.472399212849673,0.417333988458083,0.782354591539953)
dA = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_5.Initialize(body_10,body_11,False,cA,cB,dA,dB)
link_5.SetName("Concentric5")
exported_items.append(link_5)


# Mate constraint: Parallel4 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_10 , SW name: Unit-3/Mem_In-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: Unit-3/Test_Cylinder-2 ,  SW ref.type:4 (4)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.401333988458083,0.723354591539961)
dA = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
cB = chrono.ChVectorD(0.472399212849673,0.417333988458083,0.787354591539953)
dB = chrono.ChVectorD(0,1,0)
link_6.Initialize(body_10,body_11,False,cA,cB,dA,dB)
link_6.SetName("Parallel4")
exported_items.append(link_6)


# Mate constraint: Concentric9 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: Unit-3/Test_Cylinder-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: Unit-3/Test_Cylinder-2 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.674354591539961)
dA = chrono.ChVectorD(0,0,-1)
cB = chrono.ChVectorD(0.472399212849673,0.417333988458083,0.782354591539953)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_7.SetFlipped(True)
link_7.Initialize(body_13,body_11,False,cA,cB,dA,dB)
link_7.SetName("Concentric9")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateGeneric()
link_8.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.674354591539961)
cB = chrono.ChVectorD(0.472399212849673,0.417333988458083,0.782354591539953)
dA = chrono.ChVectorD(0,0,-1)
dB = chrono.ChVectorD(6.98296267768627e-15,0,1)
link_8.Initialize(body_13,body_11,False,cA,cB,dA,dB)
link_8.SetName("Concentric9")
exported_items.append(link_8)


# Mate constraint: Concentric10 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Unit-3/Mem_Out-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: Unit-3/Mem_In-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.638354591539961)
dA = chrono.ChVectorD(0,0,1)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
dB = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
link_9.SetFlipped(True)
link_9.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_9.SetName("Concentric10")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.638354591539961)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.818354591539961)
dA = chrono.ChVectorD(0,0,1)
dB = chrono.ChVectorD(-6.3215212584122e-17,1.04887724374991e-16,-1)
link_10.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_10.SetName("Concentric10")
exported_items.append(link_10)


# Mate constraint: Parallel5 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_12 , SW name: Unit-3/Mem_Out-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_10 , SW name: Unit-3/Mem_In-1 ,  SW ref.type:4 (4)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.653354591539961)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.803354591539961)
dB = chrono.ChVectorD(6.63049979382982e-33,1,1.04887724374991e-16)
link_11.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_11.SetName("Parallel5")
exported_items.append(link_11)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Unit-3/Mem_Out-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_10 , SW name: Unit-3/Mem_In-1 ,  SW ref.type:4 (4)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.653354591539961)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(0.472399212849667,0.417333988458083,0.803354591539961)
dB = chrono.ChVectorD(-1,7.79187653482018e-49,6.3215212584122e-17)
link_12.SetFlipped(True)
link_12.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_12.SetName("Parallel6")
exported_items.append(link_12)

