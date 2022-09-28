# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: D:\MMAE\540\final project\Simulations\CAD\_diff_bot.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'diff_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('wheel-1')
body_1.SetPos(chrono.ChVectorD(-0.00940122225075207,0.0386491141711684,-0.0399695080863005))
body_1.SetRot(chrono.ChQuaternionD(0.000431869692200167,-1.48736528557315e-17,0.99999990674428,5.26282345722356e-15))
body_1.SetMass(0.00321821538001793)
body_1.SetInertiaXX(chrono.ChVectorD(2.68703262088169e-07,2.68682178962215e-07,5.13491465574613e-07))
body_1.SetInertiaXY(chrono.ChVectorD(1.26232328027905e-15,2.1143709881174e-10,-1.10744177632733e-11))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009105686e-11,2.84596975179139e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('wheel-2')
body_2.SetPos(chrono.ChVectorD(-0.0094587818379601,0.0386491141711684,0.0266704670554561))
body_2.SetRot(chrono.ChQuaternionD(0.99999990674428,-5.20364044905919e-15,-0.000431869692196586,3.23299024502035e-17))
body_2.SetMass(0.00321821538001793)
body_2.SetInertiaXX(chrono.ChVectorD(2.68703262088169e-07,2.68682178962215e-07,5.13491465574612e-07))
body_2.SetInertiaXY(chrono.ChVectorD(-1.26232328478527e-15,2.11437098809986e-10,1.10744177581487e-11))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009105686e-11,2.84596975179139e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_2.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_2.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('ground-1')
body_3.SetPos(chrono.ChVectorD(0.453690077961251,-0.0792241855626804,0.802454612447068))
body_3.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3.SetMass(45.99)
body_3.SetInertiaXX(chrono.ChVectorD(34.530825,68.985,34.530825))
body_3.SetInertiaXY(chrono.ChVectorD(1.16573417585641e-16,-2.47024622979097e-15,9.16956645744367e-33))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(2.41405310855655e-17,0.05,4.472878302189e-16),chrono.ChQuaternionD(1,0,0,0)))
body_3.SetBodyFixed(True)

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(15,15,0.05,chrono.ChVectorD(0,0.05,4.44089209850063E-16),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('_bot-1')
body_4.SetPos(chrono.ChVectorD(-0.00943000204435618,0.0283553863145698,-0.00664952051542213))
body_4.SetRot(chrono.ChQuaternionD(0.99999990674428,2.5678575438006e-17,-0.00043186969220017,-1.64844618757732e-16))
body_4.SetMass(0.106126015508697)
body_4.SetInertiaXX(chrono.ChVectorD(8.56757288367862e-05,6.87885247291666e-05,8.43999614173628e-05))
body_4.SetInertiaXY(chrono.ChVectorD(-8.31860111190725e-07,3.39193932124887e-08,-5.81579231423884e-07))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000277814138212316,0.0315902190492457,0.000221563221437957),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_4.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_4_1 =chrono.ChMarker()
marker_4_1.SetName('left_marker')
body_4.AddMarker(marker_4_1)
marker_4_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.00939837191104916,0.0386491141711684,-0.043269506855325),chrono.ChQuaternionD(0.000305377987943705,-0.707106715244796,0.707106715244796,-0.000305377987943669)))

# Auxiliary marker (coordinate system feature)
marker_4_2 =chrono.ChMarker()
marker_4_2.SetName('right_marker')
body_4.AddMarker(marker_4_2)
marker_4_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.00946163217766279,0.0386491141711684,0.0299704658244806),chrono.ChQuaternionD(0.707106715244796,-0.000305377987943669,-0.000305377987943705,0.707106715244796)))

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-3.94187020738272E-17; mr[2,0]=-1 
mr[0,1]=1.85615411929519E-16; mr[1,1]=-1; mr[2,1]=3.94187020738272E-17 
mr[0,2]=-1; mr[1,2]=-1.85615411929519E-16; mr[2,2]=7.3167186231604E-33 
body_4.GetCollisionModel().AddCylinder(0.0396875,0.0396875,0.0192759,chrono.ChVectorD(5.25344026960917E-17,0.0495696278565986,6.77407103726728E-17),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)




# Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: wheel-2 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0094622627073546,0.0386491141711684,0.0307004655521739)
dA = chrono.ChVectorD(0.000863739303844567,6.71920135596089e-17,-0.999999626977138)
cB = chrono.ChVectorD(-0.00945420401964973,0.0386491141711684,0.0213704690324772)
dB = chrono.ChVectorD(-0.000863739303844535,-6.71920135595383e-17,0.999999626977138)
link_1.SetFlipped(True)
link_1.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_1.SetName("Concentric1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.0094622627073546,0.0386491141711684,0.0307004655521739)
cB = chrono.ChVectorD(-0.00945420401964973,0.0386491141711684,0.0213704690324772)
dA = chrono.ChVectorD(0.000863739303844567,6.71920135596089e-17,-0.999999626977138)
dB = chrono.ChVectorD(-0.000863739303844535,-6.71920135595383e-17,0.999999626977138)
link_2.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_2.SetName("Concentric1")
exported_items.append(link_2)


# Mate constraint: Coincident1 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: wheel-2 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.00946163217766279,0.0386491141711684,0.0299704658244806)
cB = chrono.ChVectorD(-0.00946163217766279,0.0386491141711684,0.0299704658244806)
dA = chrono.ChVectorD(-0.000863739303844567,-6.71920135596089e-17,0.999999626977138)
dB = chrono.ChVectorD(-0.000863739303844535,-6.71920135595383e-17,0.999999626977138)
link_3.Initialize(body_4,body_2,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident1")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.00946163217766279,0.0386491141711684,0.0299704658244806)
dA = chrono.ChVectorD(-0.000863739303844567,-6.71920135596089e-17,0.999999626977138)
cB = chrono.ChVectorD(-0.00946163217766279,0.0386491141711684,0.0299704658244806)
dB = chrono.ChVectorD(-0.000863739303844535,-6.71920135595383e-17,0.999999626977138)
link_4.Initialize(body_4,body_2,False,cA,cB,dA,dB)
link_4.SetName("Coincident1")
exported_items.append(link_4)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: wheel-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.00939774138135735,0.0386491141711684,-0.0439995065830183)
dA = chrono.ChVectorD(-0.000863739303851701,-5.12147632971365e-17,0.999999626977138)
cB = chrono.ChVectorD(-0.00940580006906249,0.0386491141711684,-0.0346695100633217)
dB = chrono.ChVectorD(0.000863739303851696,5.12147633007023e-17,-0.999999626977138)
link_5.SetFlipped(True)
link_5.Initialize(body_4,body_1,False,cA,cB,dA,dB)
link_5.SetName("Concentric2")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.00939774138135735,0.0386491141711684,-0.0439995065830183)
cB = chrono.ChVectorD(-0.00940580006906249,0.0386491141711684,-0.0346695100633217)
dA = chrono.ChVectorD(-0.000863739303851701,-5.12147632971365e-17,0.999999626977138)
dB = chrono.ChVectorD(0.000863739303851696,5.12147633007023e-17,-0.999999626977138)
link_6.Initialize(body_4,body_1,False,cA,cB,dA,dB)
link_6.SetName("Concentric2")
exported_items.append(link_6)


# Mate constraint: Coincident2 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: wheel-1 ,  SW ref.type:1 (1)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.00939837191104916,0.0386491141711684,-0.043269506855325)
cB = chrono.ChVectorD(-0.00939837191104936,0.0386491141711684,-0.043269506855325)
dA = chrono.ChVectorD(0.000863739303851701,5.12147632971365e-17,-0.999999626977138)
dB = chrono.ChVectorD(0.000863739303851696,5.12147633007023e-17,-0.999999626977138)
link_7.Initialize(body_1,body_4,False,cB,cA,dA)
link_7.SetDistance(0)
link_7.SetName("Coincident2")
exported_items.append(link_7)


# ChLinkMateOrthogonal skipped because directions not orthogonal! 

# Mate constraint: Parallel1 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: ground-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.00943000204435611,0.0296491141711684,-0.00664952051542214)
dA = chrono.ChVectorD(1.26515655117922e-16,-1,6.11282847499835e-18)
cB = chrono.ChVectorD(0.453690077961251,0.0207758144373196,0.802454612447068)
dB = chrono.ChVectorD(0,1,0)
link_8.SetFlipped(True)
link_8.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_8.SetName("Parallel1")
exported_items.append(link_8)

