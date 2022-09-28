# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: D:\WaveLab\Soft Robotics\Chrono_BitBucket\QZ\diff_drive\CAD\_JAMEBA.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'jameba2_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('wheel-8')
body_1.SetPos(chrono.ChVectorD(-0.238590793645048,0.0436452837141895,-0.126252875154447))
body_1.SetRot(chrono.ChQuaternionD(-0.00254992960373263,0.692850498771854,0.721061472220276,-0.00469441052077965))
body_1.SetMass(0.00321821538001793)
body_1.SetInertiaXX(chrono.ChVectorD(2.68684108045608e-07,2.68729022744133e-07,5.13463775835256e-07))
body_1.SetInertiaXY(chrono.ChVectorD(6.17321474066562e-12,6.81161299513498e-10,2.52155798285621e-09))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

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
body_2.SetName('ground-1')
body_2.SetPos(chrono.ChVectorD(0.701104484536109,-0.0499779810241818,0.000466362524235695))
body_2.SetRot(chrono.ChQuaternionD(0.999610878982654,-0.00117045627699325,-0.0247396885944354,0.0128323209077701))
body_2.SetMass(4599)
body_2.SetInertiaXX(chrono.ChVectorD(345156.872364671,689620.957362761,344929.835272568))
body_2.SetInertiaXY(chrono.ChVectorD(8865.86405867706,15.1219086317615,587.918197682824))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-1.79460708090094e-15,0.05,3.39898677684762e-17),chrono.ChQuaternionD(1,0,0,0)))
body_2.SetBodyFixed(True)

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_2.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=-1.1842378929335E-16 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(15,15,0.05,chrono.ChVectorD(-1.77635683940025E-15,0.05,0),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('_bot-1')
body_3.SetPos(chrono.ChVectorD(-0.164200012972729,0.0360609675714462,0.145271771363821))
body_3.SetRot(chrono.ChQuaternionD(0.999916458025835,-0.00147454634429635,-0.00101908846147551,0.0128009429674092))
body_3.SetMass(0.106126015508697)
body_3.SetInertiaXX(chrono.ChVectorD(8.56757288367862e-05,6.87885247291666e-05,8.43999614173628e-05))
body_3.SetInertiaXY(chrono.ChVectorD(-8.31860111190725e-07,3.39193932124887e-08,-5.81579231423884e-07))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000277814138212316,0.0315902190492457,0.000221563221437957),chrono.ChQuaternionD(1,0,0,0)))

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
marker_3_1.SetName('left_marker')
body_3.AddMarker(marker_3_1)
marker_3_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.164387484614717,0.0462442458044696,0.108621383480498),chrono.ChQuaternionD(0.000322057357487487,0.716099341667939,-0.697996074512265,0.00176326608096408)))

# Auxiliary marker (coordinate system feature)
marker_3_2 =chrono.ChMarker()
marker_3_2.SetName('right_marker')
body_3.AddMarker(marker_3_2)
marker_3_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.164539513117229,0.046458308434063,0.181860912865422),chrono.ChQuaternionD(0.697996074512264,-0.00176326608096408,0.000322057357487486,0.716099341667939)))

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-3.94187020738272E-17; mr[2,0]=-1 
mr[0,1]=1.85615411929519E-16; mr[1,1]=-1; mr[2,1]=3.94187020738272E-17 
mr[0,2]=-1; mr[1,2]=-1.85615411929519E-16; mr[2,2]=7.3167186231604E-33 
body_3.GetCollisionModel().AddCylinder(0.0396875,0.0396875,0.0192759,chrono.ChVectorD(5.25344026960917E-17,0.0495696278565986,6.77407103726728E-17),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('_bot-4')
body_4.SetPos(chrono.ChVectorD(-0.126846686783404,0.0365850967310838,2.02331146420862e-06))
body_4.SetRot(chrono.ChQuaternionD(0.999913656361861,-0.00152057504608341,0.00257709007955511,0.0127955570142753))
body_4.SetMass(0.106126015508697)
body_4.SetInertiaXX(chrono.ChVectorD(8.56757288367862e-05,6.87885247291666e-05,8.43999614173628e-05))
body_4.SetInertiaXY(chrono.ChVectorD(-8.31860111190725e-07,3.39193932124887e-08,-5.81579231423884e-07))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000277814138212316,0.0315902190492457,0.000221563221437957),chrono.ChQuaternionD(1,0,0,0)))

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
marker_4_1.SetName('left_marker')
body_4.AddMarker(marker_4_1)
marker_4_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.127297477455324,0.0467616338671585,-0.0366479441219991),chrono.ChQuaternionD(0.00289748679737063,0.71609355214836,-0.697997901880654,-0.000747068944593359)))

# Auxiliary marker (coordinate system feature)
marker_4_2 =chrono.ChMarker()
marker_4_2.SetName('right_marker')
body_4.AddMarker(marker_4_2)
marker_4_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.126922867897972,0.0469891786906492,0.0365907443632055),chrono.ChQuaternionD(0.697997901880654,0.000747068944593382,0.00289748679737061,0.71609355214836)))

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



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('_bot-7')
body_5.SetPos(chrono.ChVectorD(-0.339753338132426,0.0311784617804261,0.0150870598806283))
body_5.SetRot(chrono.ChQuaternionD(0.997716538805136,-0.000635647381436107,-0.0662998474125141,0.0128699021046323))
body_5.SetMass(0.106126015508697)
body_5.SetInertiaXX(chrono.ChVectorD(8.56757288367862e-05,6.87885247291666e-05,8.43999614173628e-05))
body_5.SetInertiaXY(chrono.ChVectorD(-8.31860111190725e-07,3.39193932124887e-08,-5.81579231423884e-07))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000277814138212316,0.0315902190492457,0.000221563221437957),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_5.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_5_1 =chrono.ChMarker()
marker_5_1.SetName('left_marker')
body_5.AddMarker(marker_5_1)
marker_5_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.335171512078029,0.0414848165013923,-0.0212415948640576),chrono.ChQuaternionD(-0.0464316011231652,0.714592525342476,-0.696391735239691,0.047330542270879)))

# Auxiliary marker (coordinate system feature)
marker_5_2 =chrono.ChMarker()
marker_5_2.SetName('right_marker')
body_5.AddMarker(marker_5_2)
marker_5_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.344862135973311,0.0414527261551001,0.0513544682435922),chrono.ChQuaternionD(0.696391735239691,-0.047330542270879,-0.0464316011231651,0.714592525342476)))

# Collision shapes 
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-3.94187020738272E-17; mr[2,0]=-1 
mr[0,1]=1.85615411929519E-16; mr[1,1]=-1; mr[2,1]=3.94187020738272E-17 
mr[0,2]=-1; mr[1,2]=-1.85615411929519E-16; mr[2,2]=7.3167186231604E-33 
body_5.GetCollisionModel().AddCylinder(0.0396875,0.0396875,0.0192759,chrono.ChVectorD(5.25344026960917E-17,0.0495696278565986,6.77407103726728E-17),mr)
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(True)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('_bot-5')
body_6.SetPos(chrono.ChVectorD(-0.237988031472295,0.0334628135842438,-0.0929041538446036))
body_6.SetRot(chrono.ChQuaternionD(0.999903912262867,-0.00155300273181763,0.0051115373065855,0.0127916617486279))
body_6.SetMass(0.106126015508697)
body_6.SetInertiaXX(chrono.ChVectorD(8.56757288367862e-05,6.87885247291666e-05,8.43999614173628e-05))
body_6.SetInertiaXY(chrono.ChVectorD(-8.31860111190725e-07,3.39193932124887e-08,-5.81579231423884e-07))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000277814138212316,0.0315902190492457,0.000221563221437957),chrono.ChQuaternionD(1,0,0,0)))

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
marker_6_1.SetName('left_marker')
body_6.AddMarker(marker_6_1)
marker_6_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.238624395437436,0.0436346033396702,-0.129552686792881),chrono.ChQuaternionD(0.0047125414546441,0.716083907661131,-0.697993766130933,-0.00251626392890516)))

# Auxiliary marker (coordinate system feature)
marker_6_2 =chrono.ChMarker()
marker_6_2.SetName('right_marker')
body_6.AddMarker(marker_6_2)
marker_6_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.237878639293642,0.0438716429244576,-0.0563168672780479),chrono.ChQuaternionD(0.697993766130933,0.00251626392890513,0.00471254145464412,0.716083907661131)))

# Collision shapes 
body_6.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-3.94187020738272E-17; mr[2,0]=-1 
mr[0,1]=1.85615411929519E-16; mr[1,1]=-1; mr[2,1]=3.94187020738272E-17 
mr[0,2]=-1; mr[1,2]=-1.85615411929519E-16; mr[2,2]=7.3167186231604E-33 
body_6.GetCollisionModel().AddCylinder(0.0396875,0.0396875,0.0192759,chrono.ChVectorD(5.25344026960917E-17,0.0495696278565986,6.77407103726728E-17),mr)
body_6.GetCollisionModel().BuildModel()
body_6.SetCollide(True)

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRef()
body_7.SetName('_bot-8')
body_7.SetPos(chrono.ChVectorD(-0.2987226486004,0.0326473456283557,0.155642680709384))
body_7.SetRot(chrono.ChQuaternionD(0.995412982082187,-0.000267409874228151,-0.0947995605183469,0.0128828148995285))
body_7.SetMass(0.106126015508697)
body_7.SetInertiaXX(chrono.ChVectorD(8.56757288367862e-05,6.87885247291666e-05,8.43999614173628e-05))
body_7.SetInertiaXY(chrono.ChVectorD(-8.31860111190725e-07,3.39193932124887e-08,-5.81579231423884e-07))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000277814138212316,0.0315902190492457,0.000221563221437957),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_7.GetAssets().push_back(body_3_1_level) 

# Auxiliary marker (coordinate system feature)
marker_7_1 =chrono.ChMarker()
marker_7_1.SetName('left_marker')
body_7.AddMarker(marker_7_1)
marker_7_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.292074610585057,0.0430076068415561,0.119650267462611),chrono.ChQuaternionD(-0.0668443247606046,0.712972795487665,-0.69475374393521,0.0672224994314506)))

# Auxiliary marker (coordinate system feature)
marker_7_2 =chrono.ChMarker()
marker_7_2.SetName('right_marker')
body_7.AddMarker(marker_7_2)
marker_7_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.30589765840223,0.0428677035107954,0.191573847574436),chrono.ChQuaternionD(0.69475374393521,-0.0672224994314506,-0.0668443247606046,0.712972795487665)))

# Collision shapes 
body_7.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-3.94187020738272E-17; mr[2,0]=-1 
mr[0,1]=1.85615411929519E-16; mr[1,1]=-1; mr[2,1]=3.94187020738272E-17 
mr[0,2]=-1; mr[1,2]=-1.85615411929519E-16; mr[2,2]=7.3167186231604E-33 
body_7.GetCollisionModel().AddCylinder(0.0396875,0.0396875,0.0192759,chrono.ChVectorD(5.25344026960917E-17,0.0495696278565986,6.77407103726728E-17),mr)
body_7.GetCollisionModel().BuildModel()
body_7.SetCollide(True)

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRef()
body_8.SetName('wheel-1')
body_8.SetPos(chrono.ChVectorD(-0.164394334615513,0.0462538908983753,0.111921362275832))
body_8.SetRot(chrono.ChQuaternionD(0.00176091574310442,0.692852615336913,0.721077000460135,-0.000334669926176492))
body_8.SetMass(0.00321821538001793)
body_8.SetInertiaXX(chrono.ChVectorD(2.68684354726469e-07,2.68703984996975e-07,5.13488566901553e-07))
body_8.SetInertiaXY(chrono.ChVectorD(-2.26300761636312e-12,7.24156139252968e-10,-4.79605116175069e-10))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_8.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_8.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_8.GetCollisionModel().BuildModel()
body_8.SetCollide(True)

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRef()
body_9.SetName('wheel-2')
body_9.SetPos(chrono.ChVectorD(-0.164532663116433,0.0464486633401572,0.178560934070088))
body_9.SetRot(chrono.ChQuaternionD(0.692852615338352,-0.00176091574310257,0.00033466992617551,0.721077000458753))
body_9.SetMass(0.00321821538001793)
body_9.SetInertiaXX(chrono.ChVectorD(2.68684354726469e-07,2.68703984996975e-07,5.13488566901553e-07))
body_9.SetInertiaXY(chrono.ChVectorD(2.26300761627898e-12,-7.24156139251025e-10,-4.79605116176211e-10))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_9.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_9.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(True)

exported_items.append(body_9)



# Rigid body part
body_10= chrono.ChBodyAuxRef()
body_10.SetName('wheel-5')
body_10.SetPos(chrono.ChVectorD(-0.292697440429359,0.0430013031688157,0.122890953076606))
body_10.SetRot(chrono.ChQuaternionD(0.0680875379483738,0.685407313642324,0.721962454856968,0.0659629847614728))
body_10.SetMass(0.00321821538001793)
body_10.SetInertiaXX(chrono.ChVectorD(2.68697301191244e-07,2.7740860425074e-07,5.04771001183013e-07))
body_10.SetInertiaXY(chrono.ChVectorD(-3.63243549791712e-10,1.88582872015579e-09,-4.53333079339588e-08))
body_10.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_10.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_10.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_10.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_10.GetCollisionModel().BuildModel()
body_10.SetCollide(True)

exported_items.append(body_10)



# Rigid body part
body_11= chrono.ChBodyAuxRef()
body_11.SetName('wheel-7')
body_11.SetPos(chrono.ChVectorD(-0.33560814586905,0.0414833705955654,-0.0179706089512334))
body_11.SetRot(chrono.ChQuaternionD(0.0433775966480851,0.752462301208508,0.655289563052456,0.0501443708062319))
body_11.SetMass(0.00321821538001793)
body_11.SetInertiaXX(chrono.ChVectorD(2.68767791237808e-07,2.72902989626874e-07,5.09206125760314e-07))
body_11.SetInertiaXY(chrono.ChVectorD(6.01127133734535e-10,-4.52725000011814e-09,-3.17834128477075e-08))
body_11.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_11.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_11.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_11.GetCollisionModel().BuildModel()
body_11.SetCollide(True)

exported_items.append(body_11)



# Rigid body part
body_12= chrono.ChBodyAuxRef()
body_12.SetName('wheel-6')
body_12.SetPos(chrono.ChVectorD(-0.127280598542991,0.0467718864329357,-0.0333480032153746))
body_12.SetRot(chrono.ChQuaternionD(-0.000773547714005649,0.69141691629981,0.722449786706808,-0.00289053034632785))
body_12.SetMass(0.00321821538001793)
body_12.SetInertiaXX(chrono.ChVectorD(2.68684186184943e-07,2.6870977040882e-07,5.13482950031234e-07))
body_12.SetInertiaXY(chrono.ChVectorD(2.71156980260926e-12,6.93839240623432e-10,1.28370381243878e-09))
body_12.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_12.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_12.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_12.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_12.GetCollisionModel().BuildModel()
body_12.SetCollide(True)

exported_items.append(body_12)



# Rigid body part
body_13= chrono.ChBodyAuxRef()
body_13.SetName('wheel-12')
body_13.SetPos(chrono.ChVectorD(-0.344425502182289,0.041454172060927,0.048083482330768))
body_13.SetRot(chrono.ChQuaternionD(0.752462301208509,-0.0433775966480827,-0.0501443708062291,0.655289563052456))
body_13.SetMass(0.00321821538001793)
body_13.SetInertiaXX(chrono.ChVectorD(2.68767791237808e-07,2.72902989626874e-07,5.09206125760315e-07))
body_13.SetInertiaXY(chrono.ChVectorD(-6.01127133734451e-10,4.52725000011787e-09,-3.17834128477058e-08))
body_13.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_13.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_13.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_13.GetCollisionModel().BuildModel()
body_13.SetCollide(True)

exported_items.append(body_13)



# Rigid body part
body_14= chrono.ChBodyAuxRef()
body_14.SetName('interior-1')
body_14.SetPos(chrono.ChVectorD(-0.250891876373902,0.106085234979652,0.127997130214379))
body_14.SetRot(chrono.ChQuaternionD(-0.0123688750218452,-0.389253341842043,-0.00361238957897945,0.921040605748208))
body_14.SetMass(0.0452035912943401)
body_14.SetInertiaXX(chrono.ChVectorD(2.58654350072856e-05,3.54636717066892e-06,2.58684230090823e-05))
body_14.SetInertiaXY(chrono.ChVectorD(-4.46087083825743e-07,-7.26630181936415e-09,-3.63699933446688e-07))
body_14.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_14.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_14.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_14.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_14.GetCollisionModel().BuildModel()
body_14.SetCollide(True)

exported_items.append(body_14)



# Rigid body part
body_15= chrono.ChBodyAuxRef()
body_15.SetName('interior-9')
body_15.SetPos(chrono.ChVectorD(-0.25761007984627,0.105778246563745,0.0826441168504304))
body_15.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_15.SetMass(0.0452035912943401)
body_15.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_15.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_15.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_15.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_15.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_15.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_15.GetCollisionModel().BuildModel()
body_15.SetCollide(True)

exported_items.append(body_15)



# Rigid body part
body_16= chrono.ChBodyAuxRef()
body_16.SetName('wheel-13')
body_16.SetPos(chrono.ChVectorD(-0.305274828557928,0.0428740071835359,0.188333161960441))
body_16.SetRot(chrono.ChQuaternionD(0.685407313642335,-0.0680875379483702,-0.0659629847614715,0.721962454856958))
body_16.SetMass(0.00321821538001793)
body_16.SetInertiaXX(chrono.ChVectorD(2.68697301191244e-07,2.7740860425074e-07,5.04771001183013e-07))
body_16.SetInertiaXY(chrono.ChVectorD(3.6324354979141e-10,-1.88582872015426e-09,-4.53333079339573e-08))
body_16.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_16.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_16.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_16.GetCollisionModel().BuildModel()
body_16.SetCollide(True)

exported_items.append(body_16)



# Rigid body part
body_17= chrono.ChBodyAuxRef()
body_17.SetName('interior-6')
body_17.SetPos(chrono.ChVectorD(-0.220320326146825,0.0269230694514383,0.155423029467483))
body_17.SetRot(chrono.ChQuaternionD(0.999916977339731,-0.00148759195489957,-1.38789400743411e-17,0.0127994335012165))
body_17.SetMass(0.0452035912943401)
body_17.SetInertiaXX(chrono.ChVectorD(2.58597084472889e-05,3.54636717066892e-06,2.5874149569079e-05))
body_17.SetInertiaXY(chrono.ChVectorD(-5.71713393424948e-07,-1.70137625921201e-09,-6.64463973727003e-08))
body_17.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_17.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_17.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_17.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_17.GetCollisionModel().BuildModel()
body_17.SetCollide(True)

exported_items.append(body_17)



# Rigid body part
body_18= chrono.ChBodyAuxRef()
body_18.SetName('interior-5')
body_18.SetPos(chrono.ChVectorD(-0.247759172017366,0.106246717241675,0.155305814696337))
body_18.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_18.SetMass(0.0452035912943401)
body_18.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_18.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_18.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_18.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_18.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_18.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_18.GetCollisionModel().BuildModel()
body_18.SetCollide(True)

exported_items.append(body_18)



# Rigid body part
body_19= chrono.ChBodyAuxRef()
body_19.SetName('linkage_bot-3')
body_19.SetPos(chrono.ChVectorD(-0.237958480747021,0.0323087248143216,-0.0929007193628217))
body_19.SetRot(chrono.ChQuaternionD(0.886700306660836,-0.0072350996649258,0.462165043831534,0.0106626338498019))
body_19.SetMass(0.013460753961169)
body_19.SetInertiaXX(chrono.ChVectorD(2.31757838104214e-05,2.57991620780839e-05,1.9356651664764e-05))
body_19.SetInertiaXY(chrono.ChVectorD(1.50484382329531e-07,-5.21419828394657e-06,2.10103173274342e-07))
body_19.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.020195479145351,0.035,-6.09160166932366e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_19_1_shape = chrono.ChObjShapeFile() 
body_19_1_shape.SetFilename(shapes_dir +'body_19_1.obj') 
body_19_1_level = chrono.ChAssetLevel() 
body_19_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_19_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_19_1_level.GetAssets().push_back(body_19_1_shape) 
body_19.GetAssets().push_back(body_19_1_level) 

# Collision shapes 
body_19.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_19.GetCollisionModel().AddBox(0.00839757122983233,0.00349999999999999,0.035,chrono.ChVectorD(-0.0586024287701677,0.035,7.80625564189563E-18),mr)
body_19.GetCollisionModel().BuildModel()
body_19.SetCollide(True)

exported_items.append(body_19)



# Rigid body part
body_20= chrono.ChBodyAuxRef()
body_20.SetName('linkage_bot_female-2')
body_20.SetPos(chrono.ChVectorD(-0.238406423727058,0.0498029134693494,-0.0929527807586087))
body_20.SetRot(chrono.ChQuaternionD(0.460779607532031,-0.0120449387200738,0.887421047110587,0.00457797761091263))
body_20.SetMass(0.0114044896980996)
body_20.SetInertiaXX(chrono.ChVectorD(1.36628000073663e-05,2.2369655556564e-05,1.1053578035157e-05))
body_20.SetInertiaXY(chrono.ChVectorD(-2.19408518703647e-07,3.62947478646609e-06,2.80102210099171e-07))
body_20.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0135858641227269,0.0175,-4.38245796283384e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_20.GetAssets().push_back(body_20_1_level) 

# Collision shapes 
body_20.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_20.GetCollisionModel().AddBox(0.00639757122983233,0.00349999999999999,0.0175,chrono.ChVectorD(-0.0606024287701677,0.0175,7.80625564189563E-18),mr)
body_20.GetCollisionModel().BuildModel()
body_20.SetCollide(True)

exported_items.append(body_20)



# Rigid body part
body_21= chrono.ChBodyAuxRef()
body_21.SetName('interior-8')
body_21.SetPos(chrono.ChVectorD(-0.197135385583981,0.107132948758413,0.0175325510054126))
body_21.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_21.SetMass(0.0452035912943401)
body_21.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_21.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_21.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_21.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_21.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_21.GetCollisionModel().BuildModel()
body_21.SetCollide(True)

exported_items.append(body_21)



# Rigid body part
body_22= chrono.ChBodyAuxRef()
body_22.SetName('linkage_bot-5')
body_22.SetPos(chrono.ChVectorD(-0.339723787407152,0.0300243730105039,0.0150904943624101))
body_22.SetRot(chrono.ChQuaternionD(0.508948051455745,-0.0117745670536035,0.860700785692463,0.00523430974212918))
body_22.SetMass(0.013460753961169)
body_22.SetInertiaXX(chrono.ChVectorD(2.42407699228365e-05,2.57991620780839e-05,1.8291665552349e-05))
body_22.SetInertiaXY(chrono.ChVectorD(-1.21870474715962e-07,4.68895997296534e-06,2.27895765949163e-07))
body_22.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.020195479145351,0.035,-6.09160166932366e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_19_1_shape = chrono.ChObjShapeFile() 
body_19_1_shape.SetFilename(shapes_dir +'body_19_1.obj') 
body_19_1_level = chrono.ChAssetLevel() 
body_19_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_19_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_19_1_level.GetAssets().push_back(body_19_1_shape) 
body_22.GetAssets().push_back(body_19_1_level) 

# Collision shapes 
body_22.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_22.GetCollisionModel().AddBox(0.00839757122983233,0.00349999999999999,0.035,chrono.ChVectorD(-0.0586024287701677,0.035,7.80625564189563E-18),mr)
body_22.GetCollisionModel().BuildModel()
body_22.SetCollide(True)

exported_items.append(body_22)



# Rigid body part
body_23= chrono.ChBodyAuxRef()
body_23.SetName('linkage_bot-1')
body_23.SetPos(chrono.ChVectorD(-0.164170462247456,0.0349068788015239,0.145275205845603))
body_23.SetRot(chrono.ChQuaternionD(0.609865477906061,0.00923580933570999,-0.79240018957004,0.00898544677190616))
body_23.SetMass(0.013460753961169)
body_23.SetInertiaXX(chrono.ChVectorD(2.60910596271266e-05,2.57991620780839e-05,1.64413758480588e-05))
body_23.SetInertiaXY(chrono.ChVectorD(-6.9079767465312e-08,-2.74867369322158e-06,-2.49031882440754e-07))
body_23.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.020195479145351,0.035,-6.09160166932366e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_19_1_shape = chrono.ChObjShapeFile() 
body_19_1_shape.SetFilename(shapes_dir +'body_19_1.obj') 
body_19_1_level = chrono.ChAssetLevel() 
body_19_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_19_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_19_1_level.GetAssets().push_back(body_19_1_shape) 
body_23.GetAssets().push_back(body_19_1_level) 

# Collision shapes 
body_23.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_23.GetCollisionModel().AddBox(0.00839757122983233,0.00349999999999999,0.035,chrono.ChVectorD(-0.0586024287701677,0.035,7.80625564189563E-18),mr)
body_23.GetCollisionModel().BuildModel()
body_23.SetCollide(True)

exported_items.append(body_23)



# Rigid body part
body_24= chrono.ChBodyAuxRef()
body_24.SetName('interior-4')
body_24.SetPos(chrono.ChVectorD(-0.221963940767549,0.106534936377621,0.0302105743534659))
body_24.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_24.SetMass(0.0452035912943401)
body_24.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_24.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_24.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_24.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_24.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_24.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_24.GetCollisionModel().BuildModel()
body_24.SetCollide(True)

exported_items.append(body_24)



# Rigid body part
body_25= chrono.ChBodyAuxRef()
body_25.SetName('interior-10')
body_25.SetPos(chrono.ChVectorD(-0.214656985146502,0.106588475752714,-0.0146685824997791))
body_25.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_25.SetMass(0.0452035912943401)
body_25.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_25.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_25.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_25.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_25.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_25.GetCollisionModel().BuildModel()
body_25.SetCollide(True)

exported_items.append(body_25)



# Rigid body part
body_26= chrono.ChBodyAuxRef()
body_26.SetName('linkage_bot_female-1')
body_26.SetPos(chrono.ChVectorD(-0.127265079038168,0.0529251966161894,-4.6603602540994e-05))
body_26.SetRot(chrono.ChQuaternionD(0.788011071294361,-0.00905137717068271,0.615526208288271,0.00917120488760286))
body_26.SetMass(0.0114044896980996)
body_26.SetInertiaXX(chrono.ChVectorD(1.57624306070905e-05,2.2369655556564e-05,8.95394743543271e-06))
body_26.SetInertiaXY(chrono.ChVectorD(6.82582783107521e-08,-1.81279791371547e-06,3.49196439879072e-07))
body_26.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0135858641227269,0.0175,-4.38245796283384e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_26.GetAssets().push_back(body_20_1_level) 

# Collision shapes 
body_26.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_26.GetCollisionModel().AddBox(0.00639757122983233,0.00349999999999999,0.0175,chrono.ChVectorD(-0.0606024287701677,0.0175,7.80625564189563E-18),mr)
body_26.GetCollisionModel().BuildModel()
body_26.SetCollide(True)

exported_items.append(body_26)



# Rigid body part
body_27= chrono.ChBodyAuxRef()
body_27.SetName('linkage_bot_female-4')
body_27.SetPos(chrono.ChVectorD(-0.340171730387189,0.0475185616655317,0.0150384329666231))
body_27.SetRot(chrono.ChQuaternionD(-0.328206940660883,-0.0116020199706875,0.944517954130173,-0.00560638567850256))
body_27.SetMass(0.0114044896980996)
body_27.SetInertiaXX(chrono.ChVectorD(1.14666825885261e-05,2.23696555565641e-05,1.32496954539971e-05))
body_27.SetInertiaXY(chrono.ChVectorD(-2.67405227360321e-07,-3.75237435912907e-06,-2.34716404545027e-07))
body_27.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0135858641227269,0.0175,-4.38245796283384e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_27.GetAssets().push_back(body_20_1_level) 

# Collision shapes 
body_27.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_27.GetCollisionModel().AddBox(0.00639757122983233,0.00349999999999999,0.0175,chrono.ChVectorD(-0.0606024287701677,0.0175,7.80625564189563E-18),mr)
body_27.GetCollisionModel().BuildModel()
body_27.SetCollide(True)

exported_items.append(body_27)



# Rigid body part
body_28= chrono.ChBodyAuxRef()
body_28.SetName('linkage_bot-6')
body_28.SetPos(chrono.ChVectorD(-0.298693097875126,0.0314932568584335,0.155646115191166))
body_28.SetRot(chrono.ChQuaternionD(0.186231833201087,-0.0128525404879099,0.982421328084232,0.000922296472393718))
body_28.SetMass(0.013460753961169)
body_28.SetInertiaXX(chrono.ChVectorD(1.72010808899897e-05,2.57991620780839e-05,2.53313545851958e-05))
body_28.SetInertiaXY(chrono.ChVectorD(-2.39382893494986e-07,3.78271975449804e-06,9.73895428011559e-08))
body_28.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.020195479145351,0.035,-6.09160166932366e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_19_1_shape = chrono.ChObjShapeFile() 
body_19_1_shape.SetFilename(shapes_dir +'body_19_1.obj') 
body_19_1_level = chrono.ChAssetLevel() 
body_19_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_19_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_19_1_level.GetAssets().push_back(body_19_1_shape) 
body_28.GetAssets().push_back(body_19_1_level) 

# Collision shapes 
body_28.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_28.GetCollisionModel().AddBox(0.00839757122983233,0.00349999999999999,0.035,chrono.ChVectorD(-0.0586024287701677,0.035,7.80625564189563E-18),mr)
body_28.GetCollisionModel().BuildModel()
body_28.SetCollide(True)

exported_items.append(body_28)



# Rigid body part
body_29= chrono.ChBodyAuxRef()
body_29.SetName('interior-3')
body_29.SetPos(chrono.ChVectorD(-0.22916394247305,0.106429652339522,0.0567817289387203))
body_29.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_29.SetMass(0.0452035912943401)
body_29.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_29.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_29.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_29.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_29.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_29.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_29.GetCollisionModel().BuildModel()
body_29.SetCollide(True)

exported_items.append(body_29)



# Rigid body part
body_30= chrono.ChBodyAuxRef()
body_30.SetName('linkage_bot-2')
body_30.SetPos(chrono.ChVectorD(-0.126817136058131,0.0354310079611616,5.45779324606589e-06))
body_30.SetRot(chrono.ChQuaternionD(0.976423842737708,0.00130557474210702,-0.215477239878256,0.0128192785432379))
body_30.SetMass(0.013460753961169)
body_30.SetInertiaXX(chrono.ChVectorD(1.76807371999658e-05,2.57991620780839e-05,2.48516982752196e-05))
body_30.SetInertiaXY(chrono.ChVectorD(2.33144025689311e-07,4.24012212620206e-06,-1.11502269171662e-07))
body_30.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.020195479145351,0.035,-6.09160166932366e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_19_1_shape = chrono.ChObjShapeFile() 
body_19_1_shape.SetFilename(shapes_dir +'body_19_1.obj') 
body_19_1_level = chrono.ChAssetLevel() 
body_19_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_19_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_19_1_level.GetAssets().push_back(body_19_1_shape) 
body_30.GetAssets().push_back(body_19_1_level) 

# Collision shapes 
body_30.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_30.GetCollisionModel().AddBox(0.00839757122983233,0.00349999999999999,0.035,chrono.ChVectorD(-0.0586024287701677,0.035,7.80625564189563E-18),mr)
body_30.GetCollisionModel().BuildModel()
body_30.SetCollide(True)

exported_items.append(body_30)



# Rigid body part
body_31= chrono.ChBodyAuxRef()
body_31.SetName('interior-7')
body_31.SetPos(chrono.ChVectorD(-0.240003635041282,0.105956603125442,-0.00891086248805028))
body_31.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_31.SetMass(0.0452035912943401)
body_31.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_31.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_31.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_31.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_31.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_31.GetCollisionModel().BuildModel()
body_31.SetCollide(True)

exported_items.append(body_31)



# Rigid body part
body_32= chrono.ChBodyAuxRef()
body_32.SetName('linkage_bot_female-6')
body_32.SetPos(chrono.ChVectorD(-0.164618405227493,0.0524010674565518,0.145223144449816))
body_32.SetRot(chrono.ChQuaternionD(0.965216151456807,-0.00477863081617888,0.261135486939509,0.0119667504068151))
body_32.SetMass(0.0114044896980996)
body_32.SetInertiaXX(chrono.ChVectorD(1.04615527245157e-05,2.2369655556564e-05,1.42548253180076e-05))
body_32.SetInertiaXY(chrono.ChVectorD(2.97625004822866e-07,-3.35825368436094e-06,1.94978723676084e-07))
body_32.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0135858641227269,0.0175,-4.38245796283384e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_32.GetAssets().push_back(body_20_1_level) 

# Collision shapes 
body_32.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_32.GetCollisionModel().AddBox(0.00639757122983233,0.00349999999999999,0.0175,chrono.ChVectorD(-0.0606024287701677,0.0175,7.80625564189563E-18),mr)
body_32.GetCollisionModel().BuildModel()
body_32.SetCollide(True)

exported_items.append(body_32)



# Rigid body part
body_33= chrono.ChBodyAuxRef()
body_33.SetName('wheel-14')
body_33.SetPos(chrono.ChVectorD(-0.23791224108603,0.0438609625499383,-0.0596166789164825))
body_33.SetRot(chrono.ChQuaternionD(0.692850498771854,0.00254992960373515,0.00469441052078218,0.721061472220276))
body_33.SetMass(0.00321821538001793)
body_33.SetInertiaXX(chrono.ChVectorD(2.68684108045608e-07,2.68729022744133e-07,5.13463775835256e-07))
body_33.SetInertiaXY(chrono.ChVectorD(-6.17321474068024e-12,-6.81161299513469e-10,2.52155798285795e-09))
body_33.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_33.GetAssets().push_back(body_1_1_level) 

# Collision parameters 
body_33.GetMaterialSurfaceNSC().SetFriction(0.8);
body_33.GetCollisionModel().SetEnvelope(0.0005);
body_33.GetCollisionModel().SetSafeMargin(0.001);

# Collision shapes 
body_33.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_33.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_33.GetCollisionModel().BuildModel()
body_33.SetCollide(True)

exported_items.append(body_33)



# Rigid body part
body_34= chrono.ChBodyAuxRef()
body_34.SetName('wheel-15')
body_34.SetPos(chrono.ChVectorD(-0.126939746810305,0.0469789261248721,0.0332908034565811))
body_34.SetRot(chrono.ChQuaternionD(0.69141691629981,0.000773547714008166,0.00289053034633038,0.722449786706808))
body_34.SetMass(0.00321821538001793)
body_34.SetInertiaXX(chrono.ChVectorD(2.68684186184943e-07,2.6870977040882e-07,5.13482950031234e-07))
body_34.SetInertiaXY(chrono.ChVectorD(-2.71156980261524e-12,-6.93839240623397e-10,1.28370381244053e-09))
body_34.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(4.32090009589031e-11,2.84596975179143e-06,-8.20361165922148e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_34.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_34.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.016,0.016,0.00329999999999999,chrono.ChVectorD(0,0,1.40078920685127E-16),mr)
body_34.GetCollisionModel().BuildModel()
body_34.SetCollide(True)

exported_items.append(body_34)



# Rigid body part
body_35= chrono.ChBodyAuxRef()
body_35.SetName('interior-2')
body_35.SetPos(chrono.ChVectorD(-0.259768324171782,0.105624981741297,0.0497123581576915))
body_35.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_35.SetMass(0.0452035912943401)
body_35.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_35.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_35.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_35.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_35.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_35.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_35.GetCollisionModel().BuildModel()
body_35.SetCollide(True)

exported_items.append(body_35)



# Rigid body part
body_36= chrono.ChBodyAuxRef()
body_36.SetName('linkage_bot_female-5')
body_36.SetPos(chrono.ChVectorD(-0.299141040855163,0.0489874455134613,0.155594053795379))
body_36.SetRot(chrono.ChQuaternionD(0.729825979455235,0.00766351326830739,-0.683511522422579,0.010359005365518))
body_36.SetMass(0.0114044896980996)
body_36.SetInertiaXX(chrono.ChVectorD(1.61820514265027e-05,2.2369655556564e-05,8.53432661602049e-06))
body_36.SetInertiaXY(chrono.ChVectorD(4.15473758080777e-08,5.03163349660579e-07,-3.53371138812487e-07))
body_36.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0135858641227269,0.0175,-4.38245796283384e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_36.GetAssets().push_back(body_20_1_level) 

# Collision shapes 
body_36.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_36.GetCollisionModel().AddBox(0.00639757122983233,0.00349999999999999,0.0175,chrono.ChVectorD(-0.0606024287701677,0.0175,7.80625564189563E-18),mr)
body_36.GetCollisionModel().BuildModel()
body_36.SetCollide(True)

exported_items.append(body_36)



# Rigid body part
body_37= chrono.ChBodyAuxRef()
body_37.SetName('interior-23')
body_37.SetPos(chrono.ChVectorD(-0.225421648298487,0.0262189393872032,-0.0372933313405059))
body_37.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_37.SetMass(0.0452035912943401)
body_37.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_37.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_37.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_37.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_37.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_37.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_37.GetCollisionModel().BuildModel()
body_37.SetCollide(True)

exported_items.append(body_37)



# Rigid body part
body_38= chrono.ChBodyAuxRef()
body_38.SetName('interior-13')
body_38.SetPos(chrono.ChVectorD(-0.202956618277607,0.0271416604485571,0.0794765657630851))
body_38.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_38.SetMass(0.0452035912943401)
body_38.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_38.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_38.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_38.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_38.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_38.GetCollisionModel().BuildModel()
body_38.SetCollide(True)

exported_items.append(body_38)



# Rigid body part
body_39= chrono.ChBodyAuxRef()
body_39.SetName('interior-17')
body_39.SetPos(chrono.ChVectorD(-0.203479670402174,0.0272080390965196,0.106282188171357))
body_39.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_39.SetMass(0.0452035912943401)
body_39.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_39.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986143e-09,-1.91011896593707e-07))
body_39.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_39.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_39.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_39.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_39.GetCollisionModel().BuildModel()
body_39.SetCollide(True)

exported_items.append(body_39)



# Rigid body part
body_40= chrono.ChBodyAuxRef()
body_40.SetName('interior-14')
body_40.SetPos(chrono.ChVectorD(-0.252665726164505,0.0255377908265845,-0.0317680855850225))
body_40.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_40.SetMass(0.0452035912943401)
body_40.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_40.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_40.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_40.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_40.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_40.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_40.GetCollisionModel().BuildModel()
body_40.SetCollide(True)

exported_items.append(body_40)



# Rigid body part
body_41= chrono.ChBodyAuxRef()
body_41.SetName('interior-22')
body_41.SetPos(chrono.ChVectorD(-0.234354466849048,0.0263986849727824,0.0999656959612354))
body_41.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_41.SetMass(0.0452035912943401)
body_41.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_41.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_41.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_41.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_41.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_41.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_41.GetCollisionModel().BuildModel()
body_41.SetCollide(True)

exported_items.append(body_41)



# Rigid body part
body_42= chrono.ChBodyAuxRef()
body_42.SetName('interior-24')
body_42.SetPos(chrono.ChVectorD(-0.163862031570576,0.0280953931632819,0.0635844517987302))
body_42.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_42.SetMass(0.0452035912943401)
body_42.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_42.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_42.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_42.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_42.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_42.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_42.GetCollisionModel().BuildModel()
body_42.SetCollide(True)

exported_items.append(body_42)



# Rigid body part
body_43= chrono.ChBodyAuxRef()
body_43.SetName('interior-19')
body_43.SetPos(chrono.ChVectorD(-0.29042120119694,0.0248139921420405,0.0498668609779022))
body_43.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_43.SetMass(0.0452035912943401)
body_43.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_43.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_43.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_43.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_43.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_43.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_43.GetCollisionModel().BuildModel()
body_43.SetCollide(True)

exported_items.append(body_43)



# Rigid body part
body_44= chrono.ChBodyAuxRef()
body_44.SetName('interior-21')
body_44.SetPos(chrono.ChVectorD(-0.184795244705956,0.0273487701127791,-0.00719118858463377))
body_44.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_44.SetMass(0.0452035912943401)
body_44.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_44.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_44.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_44.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_44.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_44.GetCollisionModel().BuildModel()
body_44.SetCollide(True)

exported_items.append(body_44)



# Rigid body part
body_45= chrono.ChBodyAuxRef()
body_45.SetName('interior-25')
body_45.SetPos(chrono.ChVectorD(-0.283700486233702,0.0248610895934517,0.00786711518934503))
body_45.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_45.SetMass(0.0452035912943401)
body_45.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_45.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_45.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_45.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_45.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_45.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_45.GetCollisionModel().BuildModel()
body_45.SetCollide(True)

exported_items.append(body_45)



# Rigid body part
body_46= chrono.ChBodyAuxRef()
body_46.SetName('interior-20')
body_46.SetPos(chrono.ChVectorD(-0.282632616106883,0.0251582223908389,0.0985245440737853))
body_46.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_46.SetMass(0.0452035912943401)
body_46.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_46.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_46.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_46.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_46.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_46.GetCollisionModel().BuildModel()
body_46.SetCollide(True)

exported_items.append(body_46)



# Rigid body part
body_47= chrono.ChBodyAuxRef()
body_47.SetName('interior-12')
body_47.SetPos(chrono.ChVectorD(-0.218264387848056,0.0268946845439996,0.128195291282985))
body_47.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_47.SetMass(0.0452035912943401)
body_47.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_47.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_47.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_47.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_47.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_47.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_47.GetCollisionModel().BuildModel()
body_47.SetCollide(True)

exported_items.append(body_47)



# Rigid body part
body_48= chrono.ChBodyAuxRef()
body_48.SetName('interior-18')
body_48.SetPos(chrono.ChVectorD(-0.195348366876243,0.0272543975061777,0.0518972158539279))
body_48.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_48.SetMass(0.0452035912943401)
body_48.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_48.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_48.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_48.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_48.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_48.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_48.GetCollisionModel().BuildModel()
body_48.SetCollide(True)

exported_items.append(body_48)



# Rigid body part
body_49= chrono.ChBodyAuxRef()
body_49.SetName('interior-11')
body_49.SetPos(chrono.ChVectorD(-0.248078666950377,0.105840102298109,0.0214199878772216))
body_49.SetRot(chrono.ChQuaternionD(-0.012693552035433,-0.284553427956906,-0.00221634033941092,0.958573579966711))
body_49.SetMass(0.0452035912943401)
body_49.SetInertiaXX(chrono.ChVectorD(2.5862451706218e-05,3.54636717066892e-06,2.58714063101499e-05))
body_49.SetInertiaXY(chrono.ChVectorD(-5.15368580084981e-07,-5.914807513381e-09,-2.56254862600636e-07))
body_49.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_49.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_49.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_49.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_49.GetCollisionModel().BuildModel()
body_49.SetCollide(True)

exported_items.append(body_49)



# Rigid body part
body_50= chrono.ChBodyAuxRef()
body_50.SetName('interior-15')
body_50.SetPos(chrono.ChVectorD(-0.177558772258079,0.0278223215747112,0.0896727449194911))
body_50.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_50.SetMass(0.0452035912943401)
body_50.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_50.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_50.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_50.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_50.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_50.GetCollisionModel().BuildModel()
body_50.SetCollide(True)

exported_items.append(body_50)



# Rigid body part
body_51= chrono.ChBodyAuxRef()
body_51.SetName('interior-16')
body_51.SetPos(chrono.ChVectorD(-0.17312152956095,0.0277695863183373,0.033773509090706))
body_51.SetRot(chrono.ChQuaternionD(0.99373106570827,-0.00289990885903097,0.111051927576831,0.0125550370921658))
body_51.SetMass(0.0452035912943401)
body_51.SetInertiaXX(chrono.ChVectorD(2.58611447805675e-05,3.54636717066892e-06,2.58727132358004e-05))
body_51.SetInertiaXY(chrono.ChVectorD(-5.4294178629477e-07,-4.64477035986101e-09,-1.91011896593707e-07))
body_51.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0,0.04,-2.43170099165011e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_14_1_shape = chrono.ChObjShapeFile() 
body_14_1_shape.SetFilename(shapes_dir +'body_14_1.obj') 
body_14_1_level = chrono.ChAssetLevel() 
body_14_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_14_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_14_1_level.GetAssets().push_back(body_14_1_shape) 
body_51.GetAssets().push_back(body_14_1_level) 

# Collision shapes 
body_51.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_51.GetCollisionModel().AddCylinder(0.0125,0.0125,0.04,chrono.ChVectorD(0,0.04,0),mr)
body_51.GetCollisionModel().BuildModel()
body_51.SetCollide(True)

exported_items.append(body_51)




# Mate constraint: wheel7_l [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: wheel-7 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.335074923390924,0.0414851363532874,-0.0219651765962884)
dA = chrono.ChVectorD(-0.132313270006589,-0.000438153280888271,0.991207852370968)
cB = chrono.ChVectorD(-0.336309406200085,0.0414810483831767,-0.0127172073336672)
dB = chrono.ChVectorD(0.132313270006588,0.000438153280888396,-0.991207852370968)
link_1.SetFlipped(True)
link_1.Initialize(body_5,body_11,False,cA,cB,dA,dB)
link_1.SetName("wheel7_l")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.335074923390924,0.0414851363532874,-0.0219651765962884)
cB = chrono.ChVectorD(-0.336309406200085,0.0414810483831767,-0.0127172073336672)
dA = chrono.ChVectorD(-0.132313270006589,-0.000438153280888271,0.991207852370968)
dB = chrono.ChVectorD(0.132313270006588,0.000438153280888396,-0.991207852370968)
link_2.Initialize(body_5,body_11,False,cA,cB,dA,dB)
link_2.SetName("wheel7_l")
exported_items.append(link_2)


# Mate constraint: wheel7_lc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_11 , SW name: wheel-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.335171512078029,0.0414848165013923,-0.0212415948640575)
cB = chrono.ChVectorD(-0.335171512078029,0.0414848165013923,-0.0212415948640576)
dA = chrono.ChVectorD(0.132313270006588,0.000438153280888396,-0.991207852370968)
dB = chrono.ChVectorD(0.132313270006589,0.000438153280888271,-0.991207852370968)
link_3.Initialize(body_11,body_5,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("wheel7_lc")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.335171512078029,0.0414848165013923,-0.0212415948640575)
dA = chrono.ChVectorD(0.132313270006588,0.000438153280888396,-0.991207852370968)
cB = chrono.ChVectorD(-0.335171512078029,0.0414848165013923,-0.0212415948640576)
dB = chrono.ChVectorD(0.132313270006589,0.000438153280888271,-0.991207852370968)
link_4.Initialize(body_11,body_5,False,cA,cB,dA,dB)
link_4.SetName("wheel7_lc")
exported_items.append(link_4)


# Mate constraint: wheel7_r [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: wheel-12 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.344958724660415,0.041452406303205,0.052078049975823)
dA = chrono.ChVectorD(0.132313270006582,0.000438153280888103,-0.991207852370969)
cB = chrono.ChVectorD(-0.343724241851254,0.0414564942733157,0.0428300807132019)
dB = chrono.ChVectorD(-0.132313270006581,-0.000438153280888247,0.991207852370969)
link_5.SetFlipped(True)
link_5.Initialize(body_5,body_13,False,cA,cB,dA,dB)
link_5.SetName("wheel7_r")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.344958724660415,0.041452406303205,0.052078049975823)
cB = chrono.ChVectorD(-0.343724241851254,0.0414564942733157,0.0428300807132019)
dA = chrono.ChVectorD(0.132313270006582,0.000438153280888103,-0.991207852370969)
dB = chrono.ChVectorD(-0.132313270006581,-0.000438153280888247,0.991207852370969)
link_6.Initialize(body_5,body_13,False,cA,cB,dA,dB)
link_6.SetName("wheel7_r")
exported_items.append(link_6)


# Mate constraint: wheel7_rc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: wheel-12 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.344862135973311,0.0414527261551001,0.0513544682435922)
cB = chrono.ChVectorD(-0.344862135973311,0.0414527261551001,0.0513544682435922)
dA = chrono.ChVectorD(-0.132313270006582,-0.000438153280888103,0.991207852370969)
dB = chrono.ChVectorD(-0.132313270006581,-0.000438153280888247,0.991207852370969)
link_7.Initialize(body_5,body_13,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("wheel7_rc")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.344862135973311,0.0414527261551001,0.0513544682435922)
dA = chrono.ChVectorD(-0.132313270006582,-0.000438153280888103,0.991207852370969)
cB = chrono.ChVectorD(-0.344862135973311,0.0414527261551001,0.0513544682435922)
dB = chrono.ChVectorD(-0.132313270006581,-0.000438153280888247,0.991207852370969)
link_8.Initialize(body_5,body_13,False,cA,cB,dA,dB)
link_8.SetName("wheel7_rc")
exported_items.append(link_8)


# Mate constraint: wheel8_lc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: wheel-5 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.292074610585057,0.0430076068415561,0.119650267462611)
cB = chrono.ChVectorD(-0.292074610585057,0.0430076068415561,0.119650267462611)
dA = chrono.ChVectorD(0.188736316455121,0.0019102038607417,-0.982025943634975)
dB = chrono.ChVectorD(0.188736316455121,0.00191020386074165,-0.982025943634975)
link_9.Initialize(body_7,body_10,False,cA,cB,dB)
link_9.SetDistance(0)
link_9.SetName("wheel8_lc")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.292074610585057,0.0430076068415561,0.119650267462611)
dA = chrono.ChVectorD(0.188736316455121,0.0019102038607417,-0.982025943634975)
cB = chrono.ChVectorD(-0.292074610585057,0.0430076068415561,0.119650267462611)
dB = chrono.ChVectorD(0.188736316455121,0.00191020386074165,-0.982025943634975)
link_10.Initialize(body_7,body_10,False,cA,cB,dA,dB)
link_10.SetName("wheel8_lc")
exported_items.append(link_10)


# Mate constraint: wheel8_l [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: wheel-5 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.291936833074045,0.0430090012903745,0.118933388523757)
dA = chrono.ChVectorD(-0.188736316455121,-0.0019102038607417,0.982025943634975)
cB = chrono.ChVectorD(-0.293697742906571,0.0429911790883538,0.128095690577871)
dB = chrono.ChVectorD(0.188736316455121,0.00191020386074165,-0.982025943634975)
link_11.SetFlipped(True)
link_11.Initialize(body_7,body_10,False,cA,cB,dA,dB)
link_11.SetName("wheel8_l")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.291936833074045,0.0430090012903745,0.118933388523757)
cB = chrono.ChVectorD(-0.293697742906571,0.0429911790883538,0.128095690577871)
dA = chrono.ChVectorD(-0.188736316455121,-0.0019102038607417,0.982025943634975)
dB = chrono.ChVectorD(0.188736316455121,0.00191020386074165,-0.982025943634975)
link_12.Initialize(body_7,body_10,False,cA,cB,dA,dB)
link_12.SetName("wheel8_l")
exported_items.append(link_12)


# Mate constraint: wheel8_rc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: wheel-13 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.30589765840223,0.0428677035107954,0.191573847574436)
cB = chrono.ChVectorD(-0.30589765840223,0.0428677035107954,0.191573847574436)
dA = chrono.ChVectorD(-0.188736316455114,-0.00191020386074153,0.982025943634976)
dB = chrono.ChVectorD(-0.188736316455114,-0.00191020386074149,0.982025943634976)
link_13.Initialize(body_7,body_16,False,cA,cB,dB)
link_13.SetDistance(0)
link_13.SetName("wheel8_rc")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.30589765840223,0.0428677035107954,0.191573847574436)
dA = chrono.ChVectorD(-0.188736316455114,-0.00191020386074153,0.982025943634976)
cB = chrono.ChVectorD(-0.30589765840223,0.0428677035107954,0.191573847574436)
dB = chrono.ChVectorD(-0.188736316455114,-0.00191020386074149,0.982025943634976)
link_14.Initialize(body_7,body_16,False,cA,cB,dA,dB)
link_14.SetName("wheel8_rc")
exported_items.append(link_14)


# Mate constraint: wheel8_r [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: wheel-13 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.306035435913242,0.0428663090619771,0.19229072651329)
dA = chrono.ChVectorD(0.188736316455114,0.00191020386074153,-0.982025943634976)
cB = chrono.ChVectorD(-0.304274526080716,0.0428841312639978,0.183128424459176)
dB = chrono.ChVectorD(-0.188736316455114,-0.00191020386074149,0.982025943634976)
link_15.SetFlipped(True)
link_15.Initialize(body_7,body_16,False,cA,cB,dA,dB)
link_15.SetName("wheel8_r")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.306035435913242,0.0428663090619771,0.19229072651329)
cB = chrono.ChVectorD(-0.304274526080716,0.0428841312639978,0.183128424459176)
dA = chrono.ChVectorD(0.188736316455114,0.00191020386074153,-0.982025943634976)
dB = chrono.ChVectorD(-0.188736316455114,-0.00191020386074149,0.982025943634976)
link_16.Initialize(body_7,body_16,False,cA,cB,dA,dB)
link_16.SetName("wheel8_r")
exported_items.append(link_16)


# Mate constraint: wheel1_rc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: wheel-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.164539513117229,0.046458308434063,0.181860912865422)
cB = chrono.ChVectorD(-0.164539513117229,0.046458308434063,0.181860912865422)
dA = chrono.ChVectorD(-0.00207575781693233,0.00292275572901949,0.999993574343573)
dB = chrono.ChVectorD(-0.00207575781693194,0.00292275572901938,0.999993574343572)
link_17.Initialize(body_3,body_9,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("wheel1_rc")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.164539513117229,0.046458308434063,0.181860912865422)
dA = chrono.ChVectorD(-0.00207575781693233,0.00292275572901949,0.999993574343573)
cB = chrono.ChVectorD(-0.164539513117229,0.046458308434063,0.181860912865422)
dB = chrono.ChVectorD(-0.00207575781693194,0.00292275572901938,0.999993574343572)
link_18.Initialize(body_3,body_9,False,cA,cB,dA,dB)
link_18.SetName("wheel1_rc")
exported_items.append(link_18)


# Mate constraint: wheel1_r [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: wheel-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.164541028420436,0.0464604420457451,0.182590908174692)
dA = chrono.ChVectorD(0.00207575781693233,-0.00292275572901949,-0.999993574343573)
cB = chrono.ChVectorD(-0.164521661600004,0.0464331727347934,0.173260968126067)
dB = chrono.ChVectorD(-0.00207575781693194,0.00292275572901938,0.999993574343572)
link_19.SetFlipped(True)
link_19.Initialize(body_3,body_9,False,cA,cB,dA,dB)
link_19.SetName("wheel1_r")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateGeneric()
link_20.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.164541028420436,0.0464604420457451,0.182590908174692)
cB = chrono.ChVectorD(-0.164521661600004,0.0464331727347934,0.173260968126067)
dA = chrono.ChVectorD(0.00207575781693233,-0.00292275572901949,-0.999993574343573)
dB = chrono.ChVectorD(-0.00207575781693194,0.00292275572901938,0.999993574343572)
link_20.Initialize(body_3,body_9,False,cA,cB,dA,dB)
link_20.SetName("wheel1_r")
exported_items.append(link_20)


# Mate constraint: wheel1_lc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: wheel-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.164387484614717,0.0462442458044696,0.108621383480498)
cB = chrono.ChVectorD(-0.164387484614717,0.0462442458044696,0.108621383480498)
dA = chrono.ChVectorD(0.00207575781693946,-0.00292275572901932,-0.999993574343572)
dB = chrono.ChVectorD(0.00207575781693908,-0.00292275572901922,-0.999993574343572)
link_21.Initialize(body_3,body_8,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("wheel1_lc")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.164387484614717,0.0462442458044696,0.108621383480498)
dA = chrono.ChVectorD(0.00207575781693946,-0.00292275572901932,-0.999993574343572)
cB = chrono.ChVectorD(-0.164387484614717,0.0462442458044696,0.108621383480498)
dB = chrono.ChVectorD(0.00207575781693908,-0.00292275572901922,-0.999993574343572)
link_22.Initialize(body_3,body_8,False,cA,cB,dA,dB)
link_22.SetName("wheel1_lc")
exported_items.append(link_22)


# Mate constraint: wheel1_l [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: wheel-1 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.164385969311511,0.0462421121927874,0.107891388171228)
dA = chrono.ChVectorD(-0.00207575781693946,0.00292275572901932,0.999993574343572)
cB = chrono.ChVectorD(-0.164405336131943,0.0462693815037391,0.117221328219853)
dB = chrono.ChVectorD(0.00207575781693908,-0.00292275572901922,-0.999993574343572)
link_23.SetFlipped(True)
link_23.Initialize(body_3,body_8,False,cA,cB,dA,dB)
link_23.SetName("wheel1_l")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.164385969311511,0.0462421121927874,0.107891388171228)
cB = chrono.ChVectorD(-0.164405336131943,0.0462693815037391,0.117221328219853)
dA = chrono.ChVectorD(-0.00207575781693946,0.00292275572901932,0.999993574343572)
dB = chrono.ChVectorD(0.00207575781693908,-0.00292275572901922,-0.999993574343572)
link_24.Initialize(body_3,body_8,False,cA,cB,dA,dB)
link_24.SetName("wheel1_l")
exported_items.append(link_24)


# Mate constraint: wheel4_rc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_34 , SW name: wheel-15 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.126922867897972,0.0469891786906492,0.0365907443632055)
cB = chrono.ChVectorD(-0.126922867897972,0.0469891786906492,0.0365907443632055)
dA = chrono.ChVectorD(0.00511482191905746,0.00310683811429163,0.999982092916502)
dB = chrono.ChVectorD(0.00511482191905748,0.00310683811429175,0.999982092916502)
link_25.Initialize(body_4,body_34,False,cA,cB,dB)
link_25.SetDistance(0)
link_25.SetName("wheel4_rc")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.126922867897972,0.0469891786906492,0.0365907443632055)
dA = chrono.ChVectorD(0.00511482191905746,0.00310683811429163,0.999982092916502)
cB = chrono.ChVectorD(-0.126922867897972,0.0469891786906492,0.0365907443632055)
dB = chrono.ChVectorD(0.00511482191905748,0.00310683811429175,0.999982092916502)
link_26.Initialize(body_4,body_34,False,cA,cB,dA,dB)
link_26.SetName("wheel4_rc")
exported_items.append(link_26)


# Mate constraint: wheel4_r [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_34 , SW name: wheel-15 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.126919134077972,0.0469914466824727,0.0373207312910346)
dA = chrono.ChVectorD(-0.00511482191905746,-0.00310683811429163,-0.999982092916502)
cB = chrono.ChVectorD(-0.126966855366476,0.0469624598828663,0.0279908983641236)
dB = chrono.ChVectorD(0.00511482191905748,0.00310683811429175,0.999982092916502)
link_27.SetFlipped(True)
link_27.Initialize(body_4,body_34,False,cA,cB,dA,dB)
link_27.SetName("wheel4_r")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.126919134077972,0.0469914466824727,0.0373207312910346)
cB = chrono.ChVectorD(-0.126966855366476,0.0469624598828663,0.0279908983641236)
dA = chrono.ChVectorD(-0.00511482191905746,-0.00310683811429163,-0.999982092916502)
dB = chrono.ChVectorD(0.00511482191905748,0.00310683811429175,0.999982092916502)
link_28.Initialize(body_4,body_34,False,cA,cB,dA,dB)
link_28.SetName("wheel4_r")
exported_items.append(link_28)


# Mate constraint: wheel4_lc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: wheel-6 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.127297477455324,0.0467616338671585,-0.0366479441219991)
cB = chrono.ChVectorD(-0.127297477455324,0.0467616338671585,-0.0366479441219991)
dA = chrono.ChVectorD(-0.00511482191905033,-0.00310683811429147,-0.999982092916502)
dB = chrono.ChVectorD(-0.00511482191905035,-0.00310683811429158,-0.999982092916502)
link_29.Initialize(body_4,body_12,False,cA,cB,dB)
link_29.SetDistance(0)
link_29.SetName("wheel4_lc")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.127297477455324,0.0467616338671585,-0.0366479441219991)
dA = chrono.ChVectorD(-0.00511482191905033,-0.00310683811429147,-0.999982092916502)
cB = chrono.ChVectorD(-0.127297477455324,0.0467616338671585,-0.0366479441219991)
dB = chrono.ChVectorD(-0.00511482191905035,-0.00310683811429158,-0.999982092916502)
link_30.Initialize(body_4,body_12,False,cA,cB,dA,dB)
link_30.SetName("wheel4_lc")
exported_items.append(link_30)


# Mate constraint: wheel4_l [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: wheel-6 ,  SW ref.type:2 (2)

link_31 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.127301211275325,0.0467593658753351,-0.0373779310498281)
dA = chrono.ChVectorD(0.00511482191905033,0.00310683811429147,0.999982092916502)
cB = chrono.ChVectorD(-0.12725348998682,0.0467883526749414,-0.0280480981229172)
dB = chrono.ChVectorD(-0.00511482191905035,-0.00310683811429158,-0.999982092916502)
link_31.SetFlipped(True)
link_31.Initialize(body_4,body_12,False,cA,cB,dA,dB)
link_31.SetName("wheel4_l")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateGeneric()
link_32.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.127301211275325,0.0467593658753351,-0.0373779310498281)
cB = chrono.ChVectorD(-0.12725348998682,0.0467883526749414,-0.0280480981229172)
dA = chrono.ChVectorD(0.00511482191905033,0.00310683811429147,0.999982092916502)
dB = chrono.ChVectorD(-0.00511482191905035,-0.00310683811429158,-0.999982092916502)
link_32.Initialize(body_4,body_12,False,cA,cB,dA,dB)
link_32.SetName("wheel4_l")
exported_items.append(link_32)


# Mate constraint: wheel5_lc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: wheel-8 ,  SW ref.type:2 (2)

link_33 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.238624395437436,0.0436346033396702,-0.129552686792881)
cB = chrono.ChVectorD(-0.238624395437436,0.0436346033396702,-0.129552686792881)
dA = chrono.ChVectorD(-0.0101823613297847,-0.00323647712708141,-0.999942920737757)
dB = chrono.ChVectorD(-0.0101823613297855,-0.00323647712708134,-0.999942920737757)
link_33.Initialize(body_6,body_1,False,cA,cB,dB)
link_33.SetDistance(0)
link_33.SetName("wheel5_lc")
exported_items.append(link_33)

link_34 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.238624395437436,0.0436346033396702,-0.129552686792881)
dA = chrono.ChVectorD(-0.0101823613297847,-0.00323647712708141,-0.999942920737757)
cB = chrono.ChVectorD(-0.238624395437436,0.0436346033396702,-0.129552686792881)
dB = chrono.ChVectorD(-0.0101823613297855,-0.00323647712708134,-0.999942920737757)
link_34.Initialize(body_6,body_1,False,cA,cB,dA,dB)
link_34.SetName("wheel5_lc")
exported_items.append(link_34)


# Mate constraint: wheel5_l [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: wheel-8 ,  SW ref.type:2 (2)

link_35 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.238631828561207,0.0436322407113674,-0.13028264512502)
dA = chrono.ChVectorD(0.0101823613297847,0.00323647712708141,0.999942920737757)
cB = chrono.ChVectorD(-0.23853682713,0.0436624370429631,-0.120953177674537)
dB = chrono.ChVectorD(-0.0101823613297855,-0.00323647712708134,-0.999942920737757)
link_35.SetFlipped(True)
link_35.Initialize(body_6,body_1,False,cA,cB,dA,dB)
link_35.SetName("wheel5_l")
exported_items.append(link_35)

link_36 = chrono.ChLinkMateGeneric()
link_36.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.238631828561207,0.0436322407113674,-0.13028264512502)
cB = chrono.ChVectorD(-0.23853682713,0.0436624370429631,-0.120953177674537)
dA = chrono.ChVectorD(0.0101823613297847,0.00323647712708141,0.999942920737757)
dB = chrono.ChVectorD(-0.0101823613297855,-0.00323647712708134,-0.999942920737757)
link_36.Initialize(body_6,body_1,False,cA,cB,dA,dB)
link_36.SetName("wheel5_l")
exported_items.append(link_36)


# Mate constraint: wheel5_rc [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: wheel-14 ,  SW ref.type:2 (2)

link_37 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.237878639293642,0.0438716429244576,-0.0563168672780479)
cB = chrono.ChVectorD(-0.237878639293642,0.0438716429244576,-0.0563168672780479)
dA = chrono.ChVectorD(0.0101823613297918,0.00323647712708158,0.999942920737757)
dB = chrono.ChVectorD(0.0101823613297927,0.00323647712708149,0.999942920737757)
link_37.Initialize(body_6,body_33,False,cA,cB,dB)
link_37.SetDistance(0)
link_37.SetName("wheel5_rc")
exported_items.append(link_37)

link_38 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.237878639293642,0.0438716429244576,-0.0563168672780479)
dA = chrono.ChVectorD(0.0101823613297918,0.00323647712708158,0.999942920737757)
cB = chrono.ChVectorD(-0.237878639293642,0.0438716429244576,-0.0563168672780479)
dB = chrono.ChVectorD(0.0101823613297927,0.00323647712708149,0.999942920737757)
link_38.Initialize(body_6,body_33,False,cA,cB,dA,dB)
link_38.SetName("wheel5_rc")
exported_items.append(link_38)


# Mate constraint: wheel5_r [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: wheel-14 ,  SW ref.type:2 (2)

link_39 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.237871206169871,0.0438740055527604,-0.0555869089459094)
dA = chrono.ChVectorD(-0.0101823613297918,-0.00323647712708158,-0.999942920737757)
cB = chrono.ChVectorD(-0.237966207601078,0.0438438092211647,-0.0649163763963926)
dB = chrono.ChVectorD(0.0101823613297927,0.00323647712708149,0.999942920737757)
link_39.SetFlipped(True)
link_39.Initialize(body_6,body_33,False,cA,cB,dA,dB)
link_39.SetName("wheel5_r")
exported_items.append(link_39)

link_40 = chrono.ChLinkMateGeneric()
link_40.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.237871206169871,0.0438740055527604,-0.0555869089459094)
cB = chrono.ChVectorD(-0.237966207601078,0.0438438092211647,-0.0649163763963926)
dA = chrono.ChVectorD(-0.0101823613297918,-0.00323647712708158,-0.999942920737757)
dB = chrono.ChVectorD(0.0101823613297927,0.00323647712708149,0.999942920737757)
link_40.Initialize(body_6,body_33,False,cA,cB,dA,dB)
link_40.SetName("wheel5_r")
exported_items.append(link_40)


# Mate constraint: Concentric33 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: linkage_bot_female-4 ,  SW ref.type:2 (2)

link_41 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.339888840317088,0.0364704317125239,0.0150713113742779)
dA = chrono.ChVectorD(-0.0255967417163951,0.999667923144447,-0.00297493690211631)
cB = chrono.ChVectorD(-0.341067616347263,0.0825069389755874,0.014934310175049)
dB = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211624)
link_41.SetFlipped(True)
link_41.Initialize(body_5,body_27,False,cA,cB,dA,dB)
link_41.SetName("Concentric33")
exported_items.append(link_41)

link_42 = chrono.ChLinkMateGeneric()
link_42.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.339888840317088,0.0364704317125239,0.0150713113742779)
cB = chrono.ChVectorD(-0.341067616347263,0.0825069389755874,0.014934310175049)
dA = chrono.ChVectorD(-0.0255967417163951,0.999667923144447,-0.00297493690211631)
dB = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211624)
link_42.Initialize(body_5,body_27,False,cA,cB,dA,dB)
link_42.SetName("Concentric33")
exported_items.append(link_42)


# Mate constraint: Concentric32 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: linkage_bot-5 ,  SW ref.type:2 (2)

link_43 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.339888840317088,0.0364704317125239,0.0150713113742779)
dA = chrono.ChVectorD(-0.0255967417163951,0.999667923144447,-0.00297493690211631)
cB = chrono.ChVectorD(-0.3415155593273,0.100001127630615,0.014882248779262)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
link_43.SetFlipped(True)
link_43.Initialize(body_5,body_22,False,cA,cB,dA,dB)
link_43.SetName("Concentric32")
exported_items.append(link_43)

link_44 = chrono.ChLinkMateGeneric()
link_44.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.339888840317088,0.0364704317125239,0.0150713113742779)
cB = chrono.ChVectorD(-0.3415155593273,0.100001127630615,0.014882248779262)
dA = chrono.ChVectorD(-0.0255967417163951,0.999667923144447,-0.00297493690211631)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
link_44.Initialize(body_5,body_22,False,cA,cB,dA,dB)
link_44.SetName("Concentric32")
exported_items.append(link_44)


# Mate constraint: Coincident77 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_22 , SW name: linkage_bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: linkage_bot_female-4 ,  SW ref.type:2 (2)

link_45 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.356654160330292,0.0822766912128181,0.0716728035691536)
cB = chrono.ChVectorD(-0.282245776762069,0.0838746781968051,-0.0315750488533928)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
dB = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211624)
link_45.Initialize(body_22,body_27,False,cA,cB,dB)
link_45.SetDistance(0)
link_45.SetName("Coincident77")
exported_items.append(link_45)

link_46 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.356654160330292,0.0822766912128181,0.0716728035691536)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
cB = chrono.ChVectorD(-0.282245776762069,0.0838746781968051,-0.0315750488533928)
dB = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211624)
link_46.SetFlipped(True)
link_46.Initialize(body_22,body_27,False,cA,cB,dA,dB)
link_46.SetName("Coincident77")
exported_items.append(link_46)


# Mate constraint: Coincident96 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: _bot-7 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: linkage_bot-5 ,  SW ref.type:2 (2)

link_47 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.343417350074839,0.0999541348307827,0.0154545136736419)
cB = chrono.ChVectorD(-0.305390573253785,0.101121685736153,0.0805992917036089)
dA = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211627)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211641)
link_47.Initialize(body_5,body_22,False,cA,cB,dB)
link_47.SetDistance(0)
link_47.SetName("Coincident96")
exported_items.append(link_47)

link_48 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.343417350074839,0.0999541348307827,0.0154545136736419)
dA = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211627)
cB = chrono.ChVectorD(-0.305390573253785,0.101121685736153,0.0805992917036089)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211641)
link_48.Initialize(body_5,body_22,False,cA,cB,dA,dB)
link_48.SetName("Coincident96")
exported_items.append(link_48)


# Mate constraint: Coincident95 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: linkage_bot-3 ,  SW ref.type:2 (2)

link_49 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.241551180305375,0.102241858319724,-0.0922715520727096)
cB = chrono.ChVectorD(-0.282693719742106,0.101368866851833,-0.0316271102491798)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211612)
dB = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211625)
link_49.Initialize(body_6,body_19,False,cA,cB,dB)
link_49.SetDistance(0)
link_49.SetName("Coincident95")
exported_items.append(link_49)

link_50 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.241551180305375,0.102241858319724,-0.0922715520727096)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211612)
cB = chrono.ChVectorD(-0.282693719742106,0.101368866851833,-0.0316271102491798)
dB = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211625)
link_50.Initialize(body_6,body_19,False,cA,cB,dA,dB)
link_50.SetName("Coincident95")
exported_items.append(link_50)


# Mate constraint: Coincident94 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_19 , SW name: linkage_bot-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: linkage_bot_female-2 ,  SW ref.type:2 (2)

link_51 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.297065377241339,0.0833453366044568,-0.0819399235982301)
cB = chrono.ChVectorD(-0.196171748777054,0.0860782147900807,-0.031712804060723)
dA = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211625)
dB = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211608)
link_51.Initialize(body_19,body_20,False,cA,cB,dB)
link_51.SetDistance(0)
link_51.SetName("Coincident94")
exported_items.append(link_51)

link_52 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.297065377241339,0.0833453366044568,-0.0819399235982301)
dA = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211625)
cB = chrono.ChVectorD(-0.196171748777054,0.0860782147900807,-0.031712804060723)
dB = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211608)
link_52.SetFlipped(True)
link_52.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_52.SetName("Coincident94")
exported_items.append(link_52)


# Mate constraint: Concentric50 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: linkage_bot-3 ,  SW ref.type:2 (2)

link_53 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.238123533656957,0.0387547835163417,-0.0929199023509539)
dA = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211612)
cB = chrono.ChVectorD(-0.239750252667169,0.102285479434433,-0.0931089649459698)
dB = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211625)
link_53.SetFlipped(True)
link_53.Initialize(body_6,body_19,False,cA,cB,dA,dB)
link_53.SetName("Concentric50")
exported_items.append(link_53)

link_54 = chrono.ChLinkMateGeneric()
link_54.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.238123533656957,0.0387547835163417,-0.0929199023509539)
cB = chrono.ChVectorD(-0.239750252667169,0.102285479434433,-0.0931089649459698)
dA = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211612)
dB = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211625)
link_54.Initialize(body_6,body_19,False,cA,cB,dA,dB)
link_54.SetName("Concentric50")
exported_items.append(link_54)


# Mate constraint: Concentric49 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: _bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: linkage_bot_female-2 ,  SW ref.type:2 (2)

link_55 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.238123533656957,0.0387547835163417,-0.0929199023509539)
dA = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211612)
cB = chrono.ChVectorD(-0.239302309687132,0.0847912907794051,-0.0930569035501828)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211608)
link_55.SetFlipped(True)
link_55.Initialize(body_6,body_20,False,cA,cB,dA,dB)
link_55.SetName("Concentric49")
exported_items.append(link_55)

link_56 = chrono.ChLinkMateGeneric()
link_56.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.238123533656957,0.0387547835163417,-0.0929199023509539)
cB = chrono.ChVectorD(-0.239302309687132,0.0847912907794051,-0.0930569035501828)
dA = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211612)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211608)
link_56.Initialize(body_6,body_20,False,cA,cB,dA,dB)
link_56.SetName("Concentric49")
exported_items.append(link_56)


# Mate constraint: Coincident93 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: linkage_bot-2 ,  SW ref.type:2 (2)

link_57 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.130414055532781,0.105364006206021,0.000625482193921228)
cB = chrono.ChVectorD(-0.19661969175709,0.103572403445108,-0.03176486545651)
dA = chrono.ChVectorD(-0.0255967417163945,0.999667923144447,-0.00297493690211604)
dB = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211606)
link_57.Initialize(body_4,body_30,False,cA,cB,dB)
link_57.SetDistance(0)
link_57.SetName("Coincident93")
exported_items.append(link_57)

link_58 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.130414055532781,0.105364006206021,0.000625482193921228)
dA = chrono.ChVectorD(-0.0255967417163945,0.999667923144447,-0.00297493690211604)
cB = chrono.ChVectorD(-0.19661969175709,0.103572403445108,-0.03176486545651)
dB = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211606)
link_58.Initialize(body_4,body_30,False,cA,cB,dA,dB)
link_58.SetName("Coincident93")
exported_items.append(link_58)


# Mate constraint: Coincident92 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: linkage_bot-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: linkage_bot_female-1 ,  SW ref.type:2 (2)

link_59 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.149181579872023,0.0872117983271633,-0.0551042636900589)
cB = chrono.ChVectorD(-0.146317471384918,0.0876652262584127,0.0726179454315166)
dA = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211606)
dB = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211592)
link_59.Initialize(body_30,body_26,False,cA,cB,dB)
link_59.SetDistance(0)
link_59.SetName("Coincident92")
exported_items.append(link_59)

link_60 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.149181579872023,0.0872117983271633,-0.0551042636900589)
dA = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211606)
cB = chrono.ChVectorD(-0.146317471384918,0.0876652262584127,0.0726179454315166)
dB = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211592)
link_60.SetFlipped(True)
link_60.Initialize(body_30,body_26,False,cA,cB,dA,dB)
link_60.SetName("Coincident92")
exported_items.append(link_60)


# Mate constraint: Concentric48 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: linkage_bot-2 ,  SW ref.type:2 (2)

link_61 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.126982188968066,0.0418770666631816,-1.3725194886162e-05)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211605)
cB = chrono.ChVectorD(-0.128608907978279,0.105407762581273,-0.000202787789902059)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211606)
link_61.SetFlipped(True)
link_61.Initialize(body_4,body_30,False,cA,cB,dA,dB)
link_61.SetName("Concentric48")
exported_items.append(link_61)

link_62 = chrono.ChLinkMateGeneric()
link_62.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.126982188968066,0.0418770666631816,-1.3725194886162e-05)
cB = chrono.ChVectorD(-0.128608907978279,0.105407762581273,-0.000202787789902059)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211605)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211606)
link_62.Initialize(body_4,body_30,False,cA,cB,dA,dB)
link_62.SetName("Concentric48")
exported_items.append(link_62)


# Mate constraint: Concentric47 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: linkage_bot_female-1 ,  SW ref.type:2 (2)

link_63 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.126982188968066,0.0418770666631816,-1.3725194886162e-05)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211605)
cB = chrono.ChVectorD(-0.128160964998242,0.0879135739262451,-0.000150726394115051)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211592)
link_63.SetFlipped(True)
link_63.Initialize(body_4,body_26,False,cA,cB,dA,dB)
link_63.SetName("Concentric47")
exported_items.append(link_63)

link_64 = chrono.ChLinkMateGeneric()
link_64.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.126982188968066,0.0418770666631816,-1.3725194886162e-05)
cB = chrono.ChVectorD(-0.128160964998242,0.0879135739262451,-0.000150726394115051)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211605)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211592)
link_64.Initialize(body_4,body_26,False,cA,cB,dA,dB)
link_64.SetName("Concentric47")
exported_items.append(link_64)


# Mate constraint: Coincident91 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: linkage_bot-1 ,  SW ref.type:2 (2)

link_65 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.167773289771368,0.104839687054295,0.14588222083238)
cB = chrono.ChVectorD(-0.146765414364954,0.10515941491344,0.0725658840357296)
dA = chrono.ChVectorD(-0.0255967417163944,0.999667923144448,-0.00297493690211574)
dB = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211582)
link_65.Initialize(body_3,body_23,False,cA,cB,dB)
link_65.SetDistance(0)
link_65.SetName("Coincident91")
exported_items.append(link_65)

link_66 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.167773289771368,0.104839687054295,0.14588222083238)
dA = chrono.ChVectorD(-0.0255967417163944,0.999667923144448,-0.00297493690211574)
cB = chrono.ChVectorD(-0.146765414364954,0.10515941491344,0.0725658840357296)
dB = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211582)
link_66.Initialize(body_3,body_23,False,cA,cB,dA,dB)
link_66.SetName("Coincident91")
exported_items.append(link_66)


# Mate constraint: Coincident90 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: linkage_bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: linkage_bot_female-6 ,  SW ref.type:2 (2)

link_67 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.115091325594848,0.0885903547029099,0.114815022645563)
cB = chrono.ChVectorD(-0.230264049339401,0.085844050163404,0.182935427817464)
dA = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211582)
dB = chrono.ChVectorD(-0.0255967417163946,0.999667923144448,-0.00297493690211566)
link_67.Initialize(body_23,body_32,False,cA,cB,dB)
link_67.SetDistance(0)
link_67.SetName("Coincident90")
exported_items.append(link_67)

link_68 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.115091325594848,0.0885903547029099,0.114815022645563)
dA = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211582)
cB = chrono.ChVectorD(-0.230264049339401,0.085844050163404,0.182935427817464)
dB = chrono.ChVectorD(-0.0255967417163946,0.999667923144448,-0.00297493690211566)
link_68.SetFlipped(True)
link_68.Initialize(body_23,body_32,False,cA,cB,dA,dB)
link_68.SetName("Coincident90")
exported_items.append(link_68)


# Mate constraint: Concentric46 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: linkage_bot-1 ,  SW ref.type:2 (2)

link_69 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.164335515157391,0.041352937503544,0.145256022857471)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211575)
cB = chrono.ChVectorD(-0.165962234167603,0.104883633421635,0.145066960262455)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211582)
link_69.SetFlipped(True)
link_69.Initialize(body_3,body_23,False,cA,cB,dA,dB)
link_69.SetName("Concentric46")
exported_items.append(link_69)

link_70 = chrono.ChLinkMateGeneric()
link_70.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.164335515157391,0.041352937503544,0.145256022857471)
cB = chrono.ChVectorD(-0.165962234167603,0.104883633421635,0.145066960262455)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211575)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211582)
link_70.Initialize(body_3,body_23,False,cA,cB,dA,dB)
link_70.SetName("Concentric46")
exported_items.append(link_70)


# Mate constraint: Concentric45 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: _bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: linkage_bot_female-6 ,  SW ref.type:2 (2)

link_71 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.164335515157391,0.041352937503544,0.145256022857471)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211575)
cB = chrono.ChVectorD(-0.165514291187566,0.0873894447666075,0.145119021658242)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144448,0.00297493690211566)
link_71.SetFlipped(True)
link_71.Initialize(body_3,body_32,False,cA,cB,dA,dB)
link_71.SetName("Concentric45")
exported_items.append(link_71)

link_72 = chrono.ChLinkMateGeneric()
link_72.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.164335515157391,0.041352937503544,0.145256022857471)
cB = chrono.ChVectorD(-0.165514291187566,0.0873894447666075,0.145119021658242)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211575)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144448,0.00297493690211566)
link_72.Initialize(body_3,body_32,False,cA,cB,dA,dB)
link_72.SetName("Concentric45")
exported_items.append(link_72)


# Mate constraint: Coincident89 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_28 , SW name: linkage_bot-6 ,  SW ref.type:2 (2)

link_73 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.302416243368619,0.101421934814849,0.155900458231521)
cB = chrono.ChVectorD(-0.230711992319438,0.103338238818432,0.182883366421677)
dA = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211645)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
link_73.Initialize(body_7,body_28,False,cA,cB,dB)
link_73.SetDistance(0)
link_73.SetName("Coincident89")
exported_items.append(link_73)

link_74 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.302416243368619,0.101421934814849,0.155900458231521)
dA = chrono.ChVectorD(-0.025596741716395,0.999667923144447,-0.00297493690211645)
cB = chrono.ChVectorD(-0.230711992319438,0.103338238818432,0.182883366421677)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
link_74.Initialize(body_7,body_28,False,cA,cB,dA,dB)
link_74.SetName("Coincident89")
exported_items.append(link_74)


# Mate constraint: Coincident88 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: linkage_bot-6 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: linkage_bot_female-5 ,  SW ref.type:2 (2)

link_75 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.275775129654227,0.0847565629293535,0.209090356780147)
cB = chrono.ChVectorD(-0.304942630273748,0.0836274970811253,0.0806513530993959)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
link_75.Initialize(body_28,body_36,False,cA,cB,dB)
link_75.SetDistance(0)
link_75.SetName("Coincident88")
exported_items.append(link_75)

link_76 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.275775129654227,0.0847565629293535,0.209090356780147)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
cB = chrono.ChVectorD(-0.304942630273748,0.0836274970811253,0.0806513530993959)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
link_76.SetFlipped(True)
link_76.Initialize(body_28,body_36,False,cA,cB,dA,dB)
link_76.SetName("Coincident88")
exported_items.append(link_76)


# Mate constraint: Concentric44 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_28 , SW name: linkage_bot-6 ,  SW ref.type:2 (2)

link_77 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.298858150785062,0.0379393155604535,0.155626932203034)
dA = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211651)
cB = chrono.ChVectorD(-0.300484869795274,0.101470011478545,0.155437869608018)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
link_77.SetFlipped(True)
link_77.Initialize(body_7,body_28,False,cA,cB,dA,dB)
link_77.SetName("Concentric44")
exported_items.append(link_77)

link_78 = chrono.ChLinkMateGeneric()
link_78.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.298858150785062,0.0379393155604535,0.155626932203034)
cB = chrono.ChVectorD(-0.300484869795274,0.101470011478545,0.155437869608018)
dA = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211651)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
link_78.Initialize(body_7,body_28,False,cA,cB,dA,dB)
link_78.SetName("Concentric44")
exported_items.append(link_78)


# Mate constraint: Concentric43 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: _bot-8 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: linkage_bot_female-5 ,  SW ref.type:2 (2)

link_79 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.298858150785062,0.0379393155604535,0.155626932203034)
dA = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211651)
cB = chrono.ChVectorD(-0.300036926815237,0.083975822823517,0.155489931003805)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
link_79.SetFlipped(True)
link_79.Initialize(body_7,body_36,False,cA,cB,dA,dB)
link_79.SetName("Concentric43")
exported_items.append(link_79)

link_80 = chrono.ChLinkMateGeneric()
link_80.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.298858150785062,0.0379393155604535,0.155626932203034)
cB = chrono.ChVectorD(-0.300036926815237,0.083975822823517,0.155489931003805)
dA = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211651)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
link_80.Initialize(body_7,body_36,False,cA,cB,dA,dB)
link_80.SetName("Concentric43")
exported_items.append(link_80)


# Mate constraint: Coincident50 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: interior-2 ,  SW ref.type:2 (2)

link_81 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.257720584834471,0.025651547889741,0.0499503531098608)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_81.Initialize(body_2,body_35,False,cA,cB,dB)
link_81.SetDistance(0)
link_81.SetName("Coincident50")
exported_items.append(link_81)

link_82 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.257720584834471,0.025651547889741,0.0499503531098608)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_82.SetFlipped(True)
link_82.Initialize(body_2,body_35,False,cA,cB,dA,dB)
link_82.SetName("Coincident50")
exported_items.append(link_82)


# Mate constraint: Coincident51 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: interior-9 ,  SW ref.type:2 (2)

link_83 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.255562340508959,0.0258048127121889,0.0828821118025997)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_83.Initialize(body_2,body_15,False,cA,cB,dB)
link_83.SetDistance(0)
link_83.SetName("Coincident51")
exported_items.append(link_83)

link_84 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.255562340508959,0.0258048127121889,0.0828821118025997)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_84.SetFlipped(True)
link_84.Initialize(body_2,body_15,False,cA,cB,dA,dB)
link_84.SetName("Coincident51")
exported_items.append(link_84)


# Mate constraint: Coincident52 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_29 , SW name: interior-3 ,  SW ref.type:2 (2)

link_85 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.227116203135738,0.0264562184879667,0.0570197238908896)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_85.Initialize(body_2,body_29,False,cA,cB,dB)
link_85.SetDistance(0)
link_85.SetName("Coincident52")
exported_items.append(link_85)

link_86 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.227116203135738,0.0264562184879667,0.0570197238908896)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_86.SetFlipped(True)
link_86.Initialize(body_2,body_29,False,cA,cB,dA,dB)
link_86.SetName("Coincident52")
exported_items.append(link_86)


# Mate constraint: Coincident53 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: interior-4 ,  SW ref.type:2 (2)

link_87 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.219916201430237,0.0265615025260648,0.0304485693056352)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_87.Initialize(body_2,body_24,False,cA,cB,dB)
link_87.SetDistance(0)
link_87.SetName("Coincident53")
exported_items.append(link_87)

link_88 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.219916201430237,0.0265615025260648,0.0304485693056352)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_88.SetFlipped(True)
link_88.Initialize(body_2,body_24,False,cA,cB,dA,dB)
link_88.SetName("Coincident53")
exported_items.append(link_88)


# Mate constraint: Coincident54 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: interior-6 ,  SW ref.type:2 (2)

link_89 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.220320326146825,0.0269230694514383,0.155423029467483)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_89.Initialize(body_2,body_17,False,cA,cB,dB)
link_89.SetDistance(0)
link_89.SetName("Coincident54")
exported_items.append(link_89)

link_90 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.220320326146825,0.0269230694514383,0.155423029467483)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_90.SetFlipped(True)
link_90.Initialize(body_2,body_17,False,cA,cB,dA,dB)
link_90.SetName("Coincident54")
exported_items.append(link_90)


# Mate constraint: Coincident55 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: interior-12 ,  SW ref.type:2 (2)

link_91 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.218264387848056,0.0268946845439996,0.128195291282985)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_91.Initialize(body_2,body_47,False,cA,cB,dB)
link_91.SetDistance(0)
link_91.SetName("Coincident55")
exported_items.append(link_91)

link_92 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.218264387848056,0.0268946845439996,0.128195291282985)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_92.SetFlipped(True)
link_92.Initialize(body_2,body_47,False,cA,cB,dA,dB)
link_92.SetName("Coincident55")
exported_items.append(link_92)


# Mate constraint: Coincident56 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: interior-1 ,  SW ref.type:2 (2)

link_93 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.24884413703659,0.0261118011280958,0.128235125166548)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_93.Initialize(body_2,body_14,False,cA,cB,dB)
link_93.SetDistance(0)
link_93.SetName("Coincident56")
exported_items.append(link_93)

link_94 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.24884413703659,0.0261118011280958,0.128235125166548)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_94.SetFlipped(True)
link_94.Initialize(body_2,body_14,False,cA,cB,dA,dB)
link_94.SetName("Coincident56")
exported_items.append(link_94)


# Mate constraint: Coincident58 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: interior-16 ,  SW ref.type:2 (2)

link_95 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.17312152956095,0.0277695863183373,0.033773509090706)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_95.Initialize(body_2,body_51,False,cA,cB,dB)
link_95.SetDistance(0)
link_95.SetName("Coincident58")
exported_items.append(link_95)

link_96 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.17312152956095,0.0277695863183373,0.033773509090706)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_96.SetFlipped(True)
link_96.Initialize(body_2,body_51,False,cA,cB,dA,dB)
link_96.SetName("Coincident58")
exported_items.append(link_96)


# Mate constraint: Coincident59 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: interior-17 ,  SW ref.type:2 (2)

link_97 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.203479670402174,0.0272080390965196,0.106282188171357)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_97.Initialize(body_2,body_39,False,cA,cB,dB)
link_97.SetDistance(0)
link_97.SetName("Coincident59")
exported_items.append(link_97)

link_98 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.203479670402174,0.0272080390965196,0.106282188171357)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_98.SetFlipped(True)
link_98.Initialize(body_2,body_39,False,cA,cB,dA,dB)
link_98.SetName("Coincident59")
exported_items.append(link_98)


# Mate constraint: Coincident60 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_41 , SW name: interior-22 ,  SW ref.type:2 (2)

link_99 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.234354466849048,0.0263986849727824,0.0999656959612354)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_99.Initialize(body_2,body_41,False,cA,cB,dB)
link_99.SetDistance(0)
link_99.SetName("Coincident60")
exported_items.append(link_99)

link_100 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.234354466849048,0.0263986849727824,0.0999656959612354)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_100.SetFlipped(True)
link_100.Initialize(body_2,body_41,False,cA,cB,dA,dB)
link_100.SetName("Coincident60")
exported_items.append(link_100)


# Mate constraint: Coincident61 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: interior-18 ,  SW ref.type:2 (2)

link_101 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.195348366876243,0.0272543975061777,0.0518972158539279)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_101.Initialize(body_2,body_48,False,cA,cB,dB)
link_101.SetDistance(0)
link_101.SetName("Coincident61")
exported_items.append(link_101)

link_102 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.195348366876243,0.0272543975061777,0.0518972158539279)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_102.SetFlipped(True)
link_102.Initialize(body_2,body_48,False,cA,cB,dA,dB)
link_102.SetName("Coincident61")
exported_items.append(link_102)


# Mate constraint: Coincident62 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_38 , SW name: interior-13 ,  SW ref.type:2 (2)

link_103 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.202956618277607,0.0271416604485571,0.0794765657630851)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_103.Initialize(body_2,body_38,False,cA,cB,dB)
link_103.SetDistance(0)
link_103.SetName("Coincident62")
exported_items.append(link_103)

link_104 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.202956618277607,0.0271416604485571,0.0794765657630851)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_104.SetFlipped(True)
link_104.Initialize(body_2,body_38,False,cA,cB,dA,dB)
link_104.SetName("Coincident62")
exported_items.append(link_104)


# Mate constraint: Coincident63 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_31 , SW name: interior-7 ,  SW ref.type:2 (2)

link_105 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.237955895703971,0.0259831692738858,-0.00867286753588099)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_105.Initialize(body_2,body_31,False,cA,cB,dB)
link_105.SetDistance(0)
link_105.SetName("Coincident63")
exported_items.append(link_105)

link_106 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.237955895703971,0.0259831692738858,-0.00867286753588099)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_106.SetFlipped(True)
link_106.Initialize(body_2,body_31,False,cA,cB,dA,dB)
link_106.SetName("Coincident63")
exported_items.append(link_106)


# Mate constraint: Coincident64 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_49 , SW name: interior-11 ,  SW ref.type:2 (2)

link_107 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.246030927613065,0.0258666684465529,0.0216579828293909)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_107.Initialize(body_2,body_49,False,cA,cB,dB)
link_107.SetDistance(0)
link_107.SetName("Coincident64")
exported_items.append(link_107)

link_108 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.246030927613065,0.0258666684465529,0.0216579828293909)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_108.SetFlipped(True)
link_108.Initialize(body_2,body_49,False,cA,cB,dA,dB)
link_108.SetName("Coincident64")
exported_items.append(link_108)


# Mate constraint: Coincident65 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: interior-10 ,  SW ref.type:2 (2)

link_109 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.21260924580919,0.0266150419011587,-0.0144305875476098)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_109.Initialize(body_2,body_25,False,cA,cB,dB)
link_109.SetDistance(0)
link_109.SetName("Coincident65")
exported_items.append(link_109)

link_110 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.21260924580919,0.0266150419011587,-0.0144305875476098)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_110.SetFlipped(True)
link_110.Initialize(body_2,body_25,False,cA,cB,dA,dB)
link_110.SetName("Coincident65")
exported_items.append(link_110)


# Mate constraint: Coincident67 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: interior-8 ,  SW ref.type:2 (2)

link_111 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.195087646246669,0.0271595149068569,0.0177705459575819)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_111.Initialize(body_2,body_21,False,cA,cB,dB)
link_111.SetDistance(0)
link_111.SetName("Coincident67")
exported_items.append(link_111)

link_112 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.195087646246669,0.0271595149068569,0.0177705459575819)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_112.SetFlipped(True)
link_112.Initialize(body_2,body_21,False,cA,cB,dA,dB)
link_112.SetName("Coincident67")
exported_items.append(link_112)


# Mate constraint: Coincident68 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: interior-5 ,  SW ref.type:2 (2)

link_113 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.245711432680054,0.0262732833901193,0.155543809648506)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_113.Initialize(body_2,body_18,False,cA,cB,dB)
link_113.SetDistance(0)
link_113.SetName("Coincident68")
exported_items.append(link_113)

link_114 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.245711432680054,0.0262732833901193,0.155543809648506)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211615)
link_114.SetFlipped(True)
link_114.Initialize(body_2,body_18,False,cA,cB,dA,dB)
link_114.SetName("Coincident68")
exported_items.append(link_114)


# Mate constraint: Coincident69 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: interior-19 ,  SW ref.type:2 (2)

link_115 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.29042120119694,0.0248139921420405,0.0498668609779022)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_115.Initialize(body_2,body_43,False,cA,cB,dB)
link_115.SetDistance(0)
link_115.SetName("Coincident69")
exported_items.append(link_115)

link_116 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.29042120119694,0.0248139921420405,0.0498668609779022)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_116.SetFlipped(True)
link_116.Initialize(body_2,body_43,False,cA,cB,dA,dB)
link_116.SetName("Coincident69")
exported_items.append(link_116)


# Mate constraint: Coincident70 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_44 , SW name: interior-21 ,  SW ref.type:2 (2)

link_117 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.184795244705956,0.0273487701127791,-0.00719118858463377)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_117.Initialize(body_2,body_44,False,cA,cB,dB)
link_117.SetDistance(0)
link_117.SetName("Coincident70")
exported_items.append(link_117)

link_118 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.184795244705956,0.0273487701127791,-0.00719118858463377)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_118.SetFlipped(True)
link_118.Initialize(body_2,body_44,False,cA,cB,dA,dB)
link_118.SetName("Coincident70")
exported_items.append(link_118)


# Mate constraint: Coincident71 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_46 , SW name: interior-20 ,  SW ref.type:2 (2)

link_119 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.282632616106883,0.0251582223908389,0.0985245440737853)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_119.Initialize(body_2,body_46,False,cA,cB,dB)
link_119.SetDistance(0)
link_119.SetName("Coincident71")
exported_items.append(link_119)

link_120 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.282632616106883,0.0251582223908389,0.0985245440737853)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_120.SetFlipped(True)
link_120.Initialize(body_2,body_46,False,cA,cB,dA,dB)
link_120.SetName("Coincident71")
exported_items.append(link_120)


# Mate constraint: Coincident72 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_42 , SW name: interior-24 ,  SW ref.type:2 (2)

link_121 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.163862031570576,0.0280953931632819,0.0635844517987302)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_121.Initialize(body_2,body_42,False,cA,cB,dB)
link_121.SetDistance(0)
link_121.SetName("Coincident72")
exported_items.append(link_121)

link_122 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.163862031570576,0.0280953931632819,0.0635844517987302)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_122.SetFlipped(True)
link_122.Initialize(body_2,body_42,False,cA,cB,dA,dB)
link_122.SetName("Coincident72")
exported_items.append(link_122)


# Mate constraint: Coincident73 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_45 , SW name: interior-25 ,  SW ref.type:2 (2)

link_123 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.283700486233702,0.0248610895934517,0.00786711518934503)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_123.Initialize(body_2,body_45,False,cA,cB,dB)
link_123.SetDistance(0)
link_123.SetName("Coincident73")
exported_items.append(link_123)

link_124 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.283700486233702,0.0248610895934517,0.00786711518934503)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_124.SetFlipped(True)
link_124.Initialize(body_2,body_45,False,cA,cB,dA,dB)
link_124.SetName("Coincident73")
exported_items.append(link_124)


# Mate constraint: Coincident74 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: interior-23 ,  SW ref.type:2 (2)

link_125 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.225421648298487,0.0262189393872032,-0.0372933313405059)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_125.Initialize(body_2,body_37,False,cA,cB,dB)
link_125.SetDistance(0)
link_125.SetName("Coincident74")
exported_items.append(link_125)

link_126 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.225421648298487,0.0262189393872032,-0.0372933313405059)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_126.SetFlipped(True)
link_126.Initialize(body_2,body_37,False,cA,cB,dA,dB)
link_126.SetName("Coincident74")
exported_items.append(link_126)


# Mate constraint: Coincident75 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: interior-14 ,  SW ref.type:2 (2)

link_127 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.252665726164505,0.0255377908265845,-0.0317680855850225)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_127.Initialize(body_2,body_40,False,cA,cB,dB)
link_127.SetDistance(0)
link_127.SetName("Coincident75")
exported_items.append(link_127)

link_128 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.252665726164505,0.0255377908265845,-0.0317680855850225)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_128.SetFlipped(True)
link_128.Initialize(body_2,body_40,False,cA,cB,dA,dB)
link_128.SetName("Coincident75")
exported_items.append(link_128)


# Mate constraint: Coincident76 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_50 , SW name: interior-15 ,  SW ref.type:2 (2)

link_129 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
cB = chrono.ChVectorD(-0.177558772258079,0.0278223215747112,0.0896727449194911)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_129.Initialize(body_2,body_50,False,cA,cB,dB)
link_129.SetDistance(0)
link_129.SetName("Coincident76")
exported_items.append(link_129)

link_130 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.177558772258079,0.0278223215747112,0.0896727449194911)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144447,0.00297493690211616)
link_130.SetFlipped(True)
link_130.Initialize(body_2,body_50,False,cA,cB,dA,dB)
link_130.SetName("Coincident76")
exported_items.append(link_130)


# Mate constraint: Concentric51 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_22 , SW name: linkage_bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: linkage_bot_female-5 ,  SW ref.type:2 (2)

link_131 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.305390573253785,0.101121685736153,0.0805992917036089)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
cB = chrono.ChVectorD(-0.304942630273748,0.0836274970811253,0.0806513530993959)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
link_131.Initialize(body_22,body_36,False,cA,cB,dA,dB)
link_131.SetName("Concentric51")
exported_items.append(link_131)

link_132 = chrono.ChLinkMateGeneric()
link_132.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.305390573253785,0.101121685736153,0.0805992917036089)
cB = chrono.ChVectorD(-0.304942630273748,0.0836274970811253,0.0806513530993959)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
dB = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
link_132.Initialize(body_22,body_36,False,cA,cB,dA,dB)
link_132.SetName("Concentric51")
exported_items.append(link_132)


# Mate constraint: Concentric52 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_28 , SW name: linkage_bot-6 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: linkage_bot_female-6 ,  SW ref.type:2 (2)

link_133 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.230711992319438,0.103338238818432,0.182883366421677)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
cB = chrono.ChVectorD(-0.230264049339401,0.085844050163404,0.182935427817464)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144448,0.00297493690211566)
link_133.Initialize(body_28,body_32,False,cA,cB,dA,dB)
link_133.SetName("Concentric52")
exported_items.append(link_133)

link_134 = chrono.ChLinkMateGeneric()
link_134.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.230711992319438,0.103338238818432,0.182883366421677)
cB = chrono.ChVectorD(-0.230264049339401,0.085844050163404,0.182935427817464)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211643)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144448,0.00297493690211566)
link_134.Initialize(body_28,body_32,False,cA,cB,dA,dB)
link_134.SetName("Concentric52")
exported_items.append(link_134)


# Mate constraint: Concentric53 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_23 , SW name: linkage_bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: linkage_bot_female-1 ,  SW ref.type:2 (2)

link_135 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.146765414364954,0.10515941491344,0.0725658840357296)
dA = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211582)
cB = chrono.ChVectorD(-0.146317471384918,0.0876652262584127,0.0726179454315166)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211592)
link_135.Initialize(body_23,body_26,False,cA,cB,dA,dB)
link_135.SetName("Concentric53")
exported_items.append(link_135)

link_136 = chrono.ChLinkMateGeneric()
link_136.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.146765414364954,0.10515941491344,0.0725658840357296)
cB = chrono.ChVectorD(-0.146317471384918,0.0876652262584127,0.0726179454315166)
dA = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211582)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211592)
link_136.Initialize(body_23,body_26,False,cA,cB,dA,dB)
link_136.SetName("Concentric53")
exported_items.append(link_136)


# Mate constraint: Concentric54 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_30 , SW name: linkage_bot-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: linkage_bot_female-2 ,  SW ref.type:2 (2)

link_137 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.19661969175709,0.103572403445108,-0.03176486545651)
dA = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211606)
cB = chrono.ChVectorD(-0.196171748777054,0.0860782147900807,-0.031712804060723)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211608)
link_137.Initialize(body_30,body_20,False,cA,cB,dA,dB)
link_137.SetName("Concentric54")
exported_items.append(link_137)

link_138 = chrono.ChLinkMateGeneric()
link_138.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.19661969175709,0.103572403445108,-0.03176486545651)
cB = chrono.ChVectorD(-0.196171748777054,0.0860782147900807,-0.031712804060723)
dA = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211606)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211608)
link_138.Initialize(body_30,body_20,False,cA,cB,dA,dB)
link_138.SetName("Concentric54")
exported_items.append(link_138)


# Mate constraint: Concentric55 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_19 , SW name: linkage_bot-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: linkage_bot_female-4 ,  SW ref.type:2 (2)

link_139 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.282693719742106,0.101368866851833,-0.0316271102491798)
dA = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211625)
cB = chrono.ChVectorD(-0.282245776762069,0.0838746781968051,-0.0315750488533928)
dB = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211624)
link_139.Initialize(body_19,body_27,False,cA,cB,dA,dB)
link_139.SetName("Concentric55")
exported_items.append(link_139)

link_140 = chrono.ChLinkMateGeneric()
link_140.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.282693719742106,0.101368866851833,-0.0316271102491798)
cB = chrono.ChVectorD(-0.282245776762069,0.0838746781968051,-0.0315750488533928)
dA = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211625)
dB = chrono.ChVectorD(0.025596741716395,-0.999667923144447,0.00297493690211624)
link_140.Initialize(body_19,body_27,False,cA,cB,dA,dB)
link_140.SetName("Concentric55")
exported_items.append(link_140)


# Mate constraint: Coincident97 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_22 , SW name: linkage_bot-5 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: linkage_bot_female-5 ,  SW ref.type:2 (2)

link_141 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.337589229293081,0.0828569050049037,0.1026048884978)
cB = chrono.ChVectorD(-0.304942630273748,0.0836274970811253,0.0806513530993959)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
link_141.Initialize(body_22,body_36,False,cA,cB,dB)
link_141.SetDistance(0)
link_141.SetName("Coincident97")
exported_items.append(link_141)

link_142 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.337589229293081,0.0828569050049037,0.1026048884978)
dA = chrono.ChVectorD(0.0255967417163952,-0.999667923144447,0.00297493690211641)
cB = chrono.ChVectorD(-0.304942630273748,0.0836274970811253,0.0806513530993959)
dB = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
link_142.SetFlipped(True)
link_142.Initialize(body_22,body_36,False,cA,cB,dA,dB)
link_142.SetName("Coincident97")
exported_items.append(link_142)


# Mate constraint: Coincident98 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: linkage_bot-6 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: linkage_bot_female-6 ,  SW ref.type:2 (2)

link_143 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.240452572145634,0.0506842106346611,0.220794794472309)
cB = chrono.ChVectorD(-0.229368163379328,0.0508556728533484,0.183039550609038)
dA = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144448,0.00297493690211566)
link_143.Initialize(body_28,body_32,False,cA,cB,dB)
link_143.SetDistance(0)
link_143.SetName("Coincident98")
exported_items.append(link_143)

link_144 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.240452572145634,0.0506842106346611,0.220794794472309)
dA = chrono.ChVectorD(-0.0255967417163952,0.999667923144447,-0.00297493690211643)
cB = chrono.ChVectorD(-0.229368163379328,0.0508556728533484,0.183039550609038)
dB = chrono.ChVectorD(0.0255967417163946,-0.999667923144448,0.00297493690211566)
link_144.SetFlipped(True)
link_144.Initialize(body_28,body_32,False,cA,cB,dA,dB)
link_144.SetName("Coincident98")
exported_items.append(link_144)


# Mate constraint: Coincident99 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: linkage_bot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: linkage_bot_female-1 ,  SW ref.type:2 (2)

link_145 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.106651482936259,0.0536893531042044,0.0793708233340177)
cB = chrono.ChVectorD(-0.145421585424844,0.052676848948357,0.0727220682230906)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211582)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211592)
link_145.Initialize(body_23,body_26,False,cA,cB,dB)
link_145.SetDistance(0)
link_145.SetName("Coincident99")
exported_items.append(link_145)

link_146 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.106651482936259,0.0536893531042044,0.0793708233340177)
dA = chrono.ChVectorD(-0.0255967417163947,0.999667923144447,-0.00297493690211582)
cB = chrono.ChVectorD(-0.145421585424844,0.052676848948357,0.0727220682230906)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211592)
link_146.SetFlipped(True)
link_146.Initialize(body_23,body_26,False,cA,cB,dA,dB)
link_146.SetName("Coincident99")
exported_items.append(link_146)


# Mate constraint: Coincident100 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: linkage_bot-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: linkage_bot_female-2 ,  SW ref.type:2 (2)

link_147 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.181958765604979,0.0513206372449386,-0.0686350719855106)
cB = chrono.ChVectorD(-0.19527586281698,0.051089837480025,-0.0316086812691489)
dA = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211606)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211608)
link_147.Initialize(body_30,body_20,False,cA,cB,dB)
link_147.SetDistance(0)
link_147.SetName("Coincident100")
exported_items.append(link_147)

link_148 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.181958765604979,0.0513206372449386,-0.0686350719855106)
dA = chrono.ChVectorD(-0.0255967417163949,0.999667923144447,-0.00297493690211606)
cB = chrono.ChVectorD(-0.19527586281698,0.051089837480025,-0.0316086812691489)
dB = chrono.ChVectorD(0.0255967417163949,-0.999667923144447,0.00297493690211608)
link_148.SetFlipped(True)
link_148.Initialize(body_30,body_20,False,cA,cB,dA,dB)
link_148.SetName("Coincident100")
exported_items.append(link_148)


# Mate constraint: Parallel1 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: ground-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: _bot-4 ,  SW ref.type:2 (2)

link_149 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.69854481036447,0.049988811290263,0.000168868834024079)
dA = chrono.ChVectorD(-0.0255967417163948,0.999667923144447,-0.00297493690211616)
cB = chrono.ChVectorD(-0.126879802001201,0.0378783949706038,-1.82544727769368e-06)
dB = chrono.ChVectorD(0.0255967417163947,-0.999667923144447,0.00297493690211605)
link_149.SetFlipped(True)
link_149.Initialize(body_2,body_4,False,cA,cB,dA,dB)
link_149.SetName("Parallel1")
exported_items.append(link_149)

