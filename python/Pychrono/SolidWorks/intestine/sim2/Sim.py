# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\Amin\Documents\SolidCAD\sim2\SimAssem2.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'Sim_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('graound-1')
body_1.SetPos(chrono.ChVectorD(0.25,-1.10171934447595e-17,-0.25))
body_1.SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,0,0))
body_1.SetMass(1603.10974449907)
body_1.SetInertiaXX(chrono.ChVectorD(2139.29220761236,4278.25165923698,2138.99655627765))
body_1.SetInertiaXY(chrono.ChVectorD(0.00941966655763467,0.0121521831061318,0.00370669095243679))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.75010692404897,1.75004207520538,0.00504630746441585),chrono.ChQuaternionD(1,0,0,0)))
body_1.SetBodyFixed(True)

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
marker_1_1.SetName('ground_marker')
body_1.AddMarker(marker_1_1)
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(4,0.00999999999999983,-4),chrono.ChQuaternionD(0.707106781186548,1.02176270012416E-16,-0.707106781186547,1.55806643888669E-17)))

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=5.89786600208644E-17; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(0.0305042045565396,1.88241479923824,0.05,chrono.ChVectorD(-0.219495795443461,1.75342339406911,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(0.0413523486386163,1.87726618574267,0.05,chrono.ChVectorD(3.70864765136138,1.73439272589085,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(2,0.0450620362723377,0.05,chrono.ChVectorD(1.75,3.70493796372766,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=6.93889390390723E-18; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(2,0.040792554490671,0.05,chrono.ChVectorD(1.75,-0.209207445509329,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=9.71445146547012E-17; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(2,2,0.00500000000005684,chrono.ChVectorD(1.75,1.75,0.00499999999994316),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('robo_leg_link-1/link2-2')
body_2.SetPos(chrono.ChVectorD(1.76653426799516,0.0952055737552142,-1.67212535415352))
body_2.SetRot(chrono.ChQuaternionD(3.40445207017784e-16,0.877055315062522,-1.25398554102702e-15,-0.480389398634671))
body_2.SetMass(0.2)
body_2.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_2.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_2.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('robo_leg_link-1/link2-1')
body_3.SetPos(chrono.ChVectorD(1.76653426799512,0.0282055737556763,-1.67212535415398))
body_3.SetRot(chrono.ChQuaternionD(0.877055315062522,-3.17204938710107e-16,0.480389398634671,-1.25398554102702e-15))
body_3.SetMass(0.2)
body_3.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_3.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_3.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_3.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('robo_leg_link-1/leg-1')
body_4.SetPos(chrono.ChVectorD(1.77146213137511,0.0342055711593255,-1.66660303514652))
body_4.SetRot(chrono.ChQuaternionD(-0.217440235101163,0.782067378299944,0.396984434806292,-0.428361667845912))
body_4.SetMass(0.1)
body_4.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_4.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_4.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('robo_leg_link-1/single_bot1-1')
body_5.SetPos(chrono.ChVectorD(1.83854355040639,-0.0974870430888756,-1.75460407733453))
body_5.SetRot(chrono.ChQuaternionD(0.70181812939663,-0.709305508406924,0.0061261896850091,0.0655704109622074))
body_5.SetMass(3.5)
body_5.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_5.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_5.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_5_1 =chrono.ChMarker()
marker_5_1.SetName('My_marker')
body_5.AddMarker(marker_5_1)
marker_5_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.77202671124361,0.0342055737557716,-1.66624226965994),chrono.ChQuaternionD(0.876877559351217,0.00967129645225651,0.480292036521019,0.0176570548415644)))

# Collision shapes 
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_5.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_5.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_5.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_5.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(True)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('robo_leg_link-1/link1-1')
body_6.SetPos(chrono.ChVectorD(1.76653426799513,0.0952055737557866,-1.67212535415382))
body_6.SetRot(chrono.ChQuaternionD(-1.34224911321681e-14,0.877055315062522,-4.16149144845558e-15,-0.480389398634671))
body_6.SetMass(0.2)
body_6.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_6.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_6.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRef()
body_7.SetName('robo_leg_link-3/link1-1')
body_7.SetPos(chrono.ChVectorD(1.5285366185935,0.0952055760792356,-1.86103838997309))
body_7.SetRot(chrono.ChQuaternionD(-6.34318070769032e-15,0.889215307282707,-1.25400478514634e-14,-0.457488947728927))
body_7.SetMass(0.2)
body_7.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_7.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_7.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRef()
body_8.SetName('robo_leg_link-3/link2-2')
body_8.SetPos(chrono.ChVectorD(1.52853661859348,0.0952055760792366,-1.86103838997306))
body_8.SetRot(chrono.ChQuaternionD(5.54654429698411e-15,0.889215307282707,-6.42461065503259e-15,-0.457488947728927))
body_8.SetMass(0.2)
body_8.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_8.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_8.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_8.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_8.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_8.GetCollisionModel().BuildModel()
body_8.SetCollide(True)

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRef()
body_9.SetName('robo_leg_link-3/link2-1')
body_9.SetPos(chrono.ChVectorD(1.5285366185935,0.0282055760792341,-1.8610383899731))
body_9.SetRot(chrono.ChQuaternionD(0.889215307282707,-5.57747305652364e-15,0.457488947728927,-6.41048494709223e-15))
body_9.SetMass(0.2)
body_9.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_9.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_9.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_9.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(True)

exported_items.append(body_9)



# Rigid body part
body_10= chrono.ChBodyAuxRef()
body_10.SetName('robo_leg_link-3/single_bot1-1')
body_10.SetPos(chrono.ChVectorD(1.60472442470348,-0.0974870407654132,-1.93967359030382))
body_10.SetRot(chrono.ChQuaternionD(0.701741045231396,-0.710767087592583,-0.0120715528156221,0.0471585649201193))
body_10.SetMass(3.5)
body_10.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_10.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_10.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_10.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_10_1 =chrono.ChMarker()
marker_10_1.SetName('My_marker')
body_10.AddMarker(marker_10_1)
marker_10_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.53371672502691,0.0342055760792365,-1.85487851064485),chrono.ChQuaternionD(0.889035087065424,0.00921025994680333,0.457396226925659,0.0179018622622703)))

# Collision shapes 
body_10.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_10.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_10.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_10.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_10.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_10.GetCollisionModel().BuildModel()
body_10.SetCollide(True)

exported_items.append(body_10)



# Rigid body part
body_11= chrono.ChBodyAuxRef()
body_11.SetName('robo_leg_link-3/leg-1')
body_11.SetPos(chrono.ChVectorD(1.5331716047521,0.0342055760792813,-1.85526805382088))
body_11.SetRot(chrono.ChQuaternionD(-0.207074723638517,0.792910408809177,0.402488442452628,-0.407941412612341))
body_11.SetMass(0.1)
body_11.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_11.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_11.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_11.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_11.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_11.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_11.GetCollisionModel().BuildModel()
body_11.SetCollide(True)

exported_items.append(body_11)



# Rigid body part
body_12= chrono.ChBodyAuxRef()
body_12.SetName('robo_leg_link-2/link2-1')
body_12.SetPos(chrono.ChVectorD(1.64642888243499,0.0282055756532229,-1.76494525711719))
body_12.SetRot(chrono.ChQuaternionD(0.916421559874536,-6.18075831774476e-15,0.400214348314902,-5.9401153579094e-15))
body_12.SetMass(0.2)
body_12.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_12.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_12.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_12.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_12.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_12.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_12.GetCollisionModel().BuildModel()
body_12.SetCollide(True)

exported_items.append(body_12)



# Rigid body part
body_13= chrono.ChBodyAuxRef()
body_13.SetName('robo_leg_link-2/leg-1')
body_13.SetPos(chrono.ChVectorD(1.65029670413096,0.0342055756533005,-1.75863496145621))
body_13.SetRot(chrono.ChQuaternionD(-0.181150321323803,0.81717015056541,0.41480286935819,-0.356869844174876))
body_13.SetMass(0.1)
body_13.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_13.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_13.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_13.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_13.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_13.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_13.GetCollisionModel().BuildModel()
body_13.SetCollide(True)

exported_items.append(body_13)



# Rigid body part
body_14= chrono.ChBodyAuxRef()
body_14.SetName('robo_leg_link-2/link2-2')
body_14.SetPos(chrono.ChVectorD(1.64642888243478,0.0952055756532232,-1.76494525711697))
body_14.SetRot(chrono.ChQuaternionD(6.15873035762544e-15,0.916421559874536,-5.9421635548115e-15,-0.400214348314902))
body_14.SetMass(0.2)
body_14.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_14.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_14.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_14.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_14.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_14.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_14.GetCollisionModel().BuildModel()
body_14.SetCollide(True)

exported_items.append(body_14)



# Rigid body part
body_15= chrono.ChBodyAuxRef()
body_15.SetName('robo_leg_link-2/single_bot1-1')
body_15.SetPos(chrono.ChVectorD(1.73195180279291,-0.0974870411914324,-1.83331123275087))
body_15.SetRot(chrono.ChQuaternionD(0.69956530241508,-0.712326968461062,-0.0565208333867177,0.00201818067592932))
body_15.SetMass(3.5)
body_15.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_15.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_15.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_15.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_15_1 =chrono.ChMarker()
marker_15_1.SetName('My_marker')
body_15.AddMarker(marker_15_1)
marker_15_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.65078816930782,0.0342055756532171,-1.75817959129917),chrono.ChQuaternionD(0.916235825675754,0.00805719613712778,0.400133235544767,0.0184495840373942)))

# Collision shapes 
body_15.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_15.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_15.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_15.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_15.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_15.GetCollisionModel().BuildModel()
body_15.SetCollide(True)

exported_items.append(body_15)



# Rigid body part
body_16= chrono.ChBodyAuxRef()
body_16.SetName('robo_leg_link-2/link1-1')
body_16.SetPos(chrono.ChVectorD(1.64642888243496,0.0952055756532315,-1.76494525711712))
body_16.SetRot(chrono.ChQuaternionD(-5.77154893538014e-15,0.916421559874536,-1.11493680647389e-14,-0.400214348314902))
body_16.SetMass(0.2)
body_16.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_16.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_16.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_16.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_16)



# Rigid body part
body_17= chrono.ChBodyAuxRef()
body_17.SetName('robo_leg_link-4/link2-1')
body_17.SetPos(chrono.ChVectorD(1.45804932379753,0.0282055756532208,-1.9810672085601))
body_17.SetRot(chrono.ChQuaternionD(0.998442200583955,-7.81474315683544e-15,0.0557958071280361,-3.54088242862032e-15))
body_17.SetMass(0.2)
body_17.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_17.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_17.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_17.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_17.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_17.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_17.GetCollisionModel().BuildModel()
body_17.SetCollide(True)

exported_items.append(body_17)



# Rigid body part
body_18= chrono.ChBodyAuxRef()
body_18.SetName('robo_leg_link-4/link2-2')
body_18.SetPos(chrono.ChVectorD(1.45804932379806,0.0952055756532212,-1.98106720856016))
body_18.SetRot(chrono.ChQuaternionD(7.81191983301478e-15,0.998442200583955,-3.50265896766367e-15,-0.0557958071280361))
body_18.SetMass(0.2)
body_18.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_18.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_18.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_18.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_18.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_18.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_18.GetCollisionModel().BuildModel()
body_18.SetCollide(True)

exported_items.append(body_18)



# Rigid body part
body_19= chrono.ChBodyAuxRef()
body_19.SetName('robo_leg_link-4/single_bot1-1')
body_19.SetPos(chrono.ChVectorD(1.56746248685881,-0.0974870411914289,-1.97696400495014))
body_19.SetRot(chrono.ChQuaternionD(0.636024356498631,-0.668384587444748,-0.296747763075536,-0.246324636032374))
body_19.SetMass(3.5)
body_19.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_19.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_19.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_19.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_19_1 =chrono.ChMarker()
marker_19_1.SetName('My_marker')
body_19.AddMarker(marker_19_1)
marker_19_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.4569303894748,0.0342055756532197,-1.97309691358562),chrono.ChQuaternionD(0.998239842989723,0.00112329246452391,0.055784498806636,0.0201008401512053)))

# Collision shapes 
body_19.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_19.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_19.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_19.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_19.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_19.GetCollisionModel().BuildModel()
body_19.SetCollide(True)

exported_items.append(body_19)



# Rigid body part
body_20= chrono.ChBodyAuxRef()
body_20.SetName('robo_leg_link-4/link1-1')
body_20.SetPos(chrono.ChVectorD(1.45804932379766,0.0952055756532568,-1.98106720856015))
body_20.SetRot(chrono.ChQuaternionD(-5.4974458363387e-15,0.998442200583955,-4.28450248723146e-15,-0.0557958071280361))
body_20.SetMass(0.2)
body_20.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_20.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_20.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_20.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_20)



# Rigid body part
body_21= chrono.ChBodyAuxRef()
body_21.SetName('robo_leg_link-4/leg-1')
body_21.SetPos(chrono.ChVectorD(1.45685573956354,0.034205575679288,-1.97376274193494))
body_21.SetRot(chrono.ChQuaternionD(-0.0252550374229336,0.890307692507711,0.451928136509032,-0.049752941398828))
body_21.SetMass(0.1)
body_21.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_21.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_21.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_21.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_21.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_21.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_21.GetCollisionModel().BuildModel()
body_21.SetCollide(True)

exported_items.append(body_21)



# Rigid body part
body_22= chrono.ChBodyAuxRef()
body_22.SetName('robo_leg_link-5/link2-1')
body_22.SetPos(chrono.ChVectorD(1.48120086828796,0.0282055807421748,-2.12611268218589))
body_22.SetRot(chrono.ChQuaternionD(0.97716687095753,-8.47952067274223e-15,-0.212473307271923,-1.09000851943133e-15))
body_22.SetMass(0.2)
body_22.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_22.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_22.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_22.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_22.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_22.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_22.GetCollisionModel().BuildModel()
body_22.SetCollide(True)

exported_items.append(body_22)



# Rigid body part
body_23= chrono.ChBodyAuxRef()
body_23.SetName('robo_leg_link-5/single_bot1-1')
body_23.SetPos(chrono.ChVectorD(1.57294423122754,-0.0974870411914339,-2.06635289055758))
body_23.SetRot(chrono.ChQuaternionD(0.533861625797504,-0.578496116632602,-0.455607266678969,-0.415639298059224))
body_23.SetMass(3.5)
body_23.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_23.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_23.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_23.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_23_1 =chrono.ChMarker()
marker_23_1.SetName('My_marker')
body_23.AddMarker(marker_23_1)
marker_23_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.47614420780503,0.0342055756532136,-2.11985106261169),chrono.ChQuaternionD(0.976968825304957,-0.00427755555943423,-0.212430244601629,0.0196725209157676)))

# Collision shapes 
body_23.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_23.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_23.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_23.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_23.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_23.GetCollisionModel().BuildModel()
body_23.SetCollide(True)

exported_items.append(body_23)



# Rigid body part
body_24= chrono.ChBodyAuxRef()
body_24.SetName('robo_leg_link-5/link1-1')
body_24.SetPos(chrono.ChVectorD(1.48120086848332,0.095205575653262,-2.12611267572529))
body_24.SetRot(chrono.ChQuaternionD(-4.49483409351241e-15,0.97716687095753,1.73005764884841e-15,0.212473307271923))
body_24.SetMass(0.2)
body_24.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_24.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_24.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_24.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_24)



# Rigid body part
body_25= chrono.ChBodyAuxRef()
body_25.SetName('robo_leg_link-5/leg-1')
body_25.SetPos(chrono.ChVectorD(1.47642242111997,0.0342055756535343,-2.12046056843724))
body_25.SetRot(chrono.ChQuaternionD(0.0961724862870948,0.871336542558961,0.442298228911587,0.189461761800178))
body_25.SetMass(0.1)
body_25.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_25.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_25.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_25.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_25.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_25.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_25.GetCollisionModel().BuildModel()
body_25.SetCollide(True)

exported_items.append(body_25)



# Rigid body part
body_26= chrono.ChBodyAuxRef()
body_26.SetName('robo_leg_link-5/link2-2')
body_26.SetPos(chrono.ChVectorD(1.48120086788773,0.0952055817647147,-2.12611268110726))
body_26.SetRot(chrono.ChQuaternionD(8.58367124738527e-15,0.97716687095753,-7.94161224700993e-16,0.212473307271923))
body_26.SetMass(0.2)
body_26.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_26.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_26.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_26.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_26.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_26.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_26.GetCollisionModel().BuildModel()
body_26.SetCollide(True)

exported_items.append(body_26)



# Rigid body part
body_27= chrono.ChBodyAuxRef()
body_27.SetName('robo_leg_link-8/link1-1')
body_27.SetPos(chrono.ChVectorD(1.85052480350311,0.0952055756532246,-2.05976949673849))
body_27.SetRot(chrono.ChQuaternionD(-4.79780492283816e-16,0.544387285767087,1.50612827808809e-14,0.838834002109562))
body_27.SetMass(0.2)
body_27.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_27.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_27.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_27.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_27)



# Rigid body part
body_28= chrono.ChBodyAuxRef()
body_28.SetName('robo_leg_link-8/leg-1')
body_28.SetPos(chrono.ChVectorD(1.84483118957432,0.0342055756533141,-2.06449841167006))
body_28.SetRot(chrono.ChQuaternionD(0.379684224660554,0.485428366708892,0.246407827999017,0.747985543068219))
body_28.SetMass(0.1)
body_28.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_28.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_28.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_28.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_28.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_28.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_28.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_28.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_28.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_28.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_28.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_28.GetCollisionModel().BuildModel()
body_28.SetCollide(True)

exported_items.append(body_28)



# Rigid body part
body_29= chrono.ChBodyAuxRef()
body_29.SetName('robo_leg_link-8/single_bot1-1')
body_29.SetPos(chrono.ChVectorD(1.79156833576289,-0.0974870411914101,-1.96750785008886))
body_29.SetRot(chrono.ChQuaternionD(-0.0583881636364392,0.118224971940484,0.699411923902511,0.702450453099594))
body_29.SetMass(3.5)
body_29.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_29.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_29.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_29.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_29_1 =chrono.ChMarker()
marker_29_1.SetName('My_marker')
body_29.AddMarker(marker_29_1)
marker_29_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.8442192778164,0.0342055756532361,-2.06477129259737),chrono.ChQuaternionD(0.544276952989266,-0.0168875756452888,-0.838663992838595,0.0109597148489501)))

# Collision shapes 
body_29.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_29.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_29.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_29.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_29.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_29.GetCollisionModel().BuildModel()
body_29.SetCollide(True)

exported_items.append(body_29)



# Rigid body part
body_30= chrono.ChBodyAuxRef()
body_30.SetName('robo_leg_link-8/link2-1')
body_30.SetPos(chrono.ChVectorD(1.85052480350314,0.0282055756532202,-2.05976949673873))
body_30.SetRot(chrono.ChQuaternionD(0.544387285767087,-6.28995882482298e-15,-0.838834002109562,4.65145454038649e-15))
body_30.SetMass(0.2)
body_30.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_30.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_30.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_30.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_30.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_30.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_30.GetCollisionModel().BuildModel()
body_30.SetCollide(True)

exported_items.append(body_30)



# Rigid body part
body_31= chrono.ChBodyAuxRef()
body_31.SetName('robo_leg_link-8/link2-2')
body_31.SetPos(chrono.ChVectorD(1.85052480350277,0.0952055756532214,-2.05976949673789))
body_31.SetRot(chrono.ChQuaternionD(5.97046029716898e-15,0.544387285767087,4.48892420948714e-15,0.838834002109562))
body_31.SetMass(0.2)
body_31.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_31.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_31.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_31.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_31.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_31.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_31.GetCollisionModel().BuildModel()
body_31.SetCollide(True)

exported_items.append(body_31)



# Rigid body part
body_32= chrono.ChBodyAuxRef()
body_32.SetName('robo_leg_link-7/link1-1')
body_32.SetPos(chrono.ChVectorD(1.72303650230103,0.0952055756532059,-2.14034889487404))
body_32.SetRot(chrono.ChQuaternionD(-3.32000901721579e-16,0.41821794295049,1.75947186205532e-14,0.908346713647526))
body_32.SetMass(0.2)
body_32.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_32.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_32.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_32.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_32)



# Rigid body part
body_33= chrono.ChBodyAuxRef()
body_33.SetName('robo_leg_link-7/link2-2')
body_33.SetPos(chrono.ChVectorD(1.72303650230074,0.0952055756532171,-2.14034889487377))
body_33.SetRot(chrono.ChQuaternionD(5.27475485809613e-15,0.41821794295049,5.47432469251722e-15,0.908346713647526))
body_33.SetMass(0.2)
body_33.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_33.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_33.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_33.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_33.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_33.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_33.GetCollisionModel().BuildModel()
body_33.SetCollide(True)

exported_items.append(body_33)



# Rigid body part
body_34= chrono.ChBodyAuxRef()
body_34.SetName('robo_leg_link-7/leg-1')
body_34.SetPos(chrono.ChVectorD(1.71892272931436,0.0342055742743635,-2.14650167862855))
body_34.SetRot(chrono.ChQuaternionD(0.4111479972839,0.37292357468605,0.18929937994911,0.809969799760274))
body_34.SetMass(0.1)
body_34.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_34.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_34.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_34.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_34.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_34.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_34.GetCollisionModel().BuildModel()
body_34.SetCollide(True)

exported_items.append(body_34)



# Rigid body part
body_35= chrono.ChBodyAuxRef()
body_35.SetName('robo_leg_link-7/link2-1')
body_35.SetPos(chrono.ChVectorD(1.72303650230108,0.0282055756532166,-2.14034889487416))
body_35.SetRot(chrono.ChQuaternionD(-0.41821794295049,5.6223310290496e-15,0.908346713647526,-5.65384205553714e-15))
body_35.SetMass(0.2)
body_35.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_35.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_35.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_35.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_35.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_35.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_35.GetCollisionModel().BuildModel()
body_35.SetCollide(True)

exported_items.append(body_35)



# Rigid body part
body_36= chrono.ChBodyAuxRef()
body_36.SetName('robo_leg_link-7/single_bot1-1')
body_36.SetPos(chrono.ChVectorD(1.64027744082636,-0.0974870411914093,-2.06866197643883))
body_36.SetRot(chrono.ChQuaternionD(0.0427070213895863,0.0160723918113438,0.700544307859165,0.712148482601498))
body_36.SetMass(3.5)
body_36.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_36.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_36.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_36.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_36_1 =chrono.ChMarker()
marker_36_1.SetName('My_marker')
body_36.AddMarker(marker_36_1)
marker_36_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.71841368085753,0.0342055756532371,-2.14693730423451),chrono.ChQuaternionD(-0.418133181332085,0.018287019601367,0.908162616004625,-0.00841964814257869)))

# Collision shapes 
body_36.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_36.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_36.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_36.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_36.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_36.GetCollisionModel().BuildModel()
body_36.SetCollide(True)

exported_items.append(body_36)



# Rigid body part
body_37= chrono.ChBodyAuxRef()
body_37.SetName('robo_leg_link-6/link1-1')
body_37.SetPos(chrono.ChVectorD(1.58899208334844,0.0952055760792379,-2.19266289213859))
body_37.SetRot(chrono.ChQuaternionD(-1.80023194904146e-15,0.694136937375286,1.34877543399091e-14,0.719842977441093))
body_37.SetMass(0.2)
body_37.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_37.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_37.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_37.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_37)



# Rigid body part
body_38= chrono.ChBodyAuxRef()
body_38.SetName('robo_leg_link-6/leg-1')
body_38.SetPos(chrono.ChVectorD(1.58194348786147,0.0342055760792723,-2.19492058513386))
body_38.SetRot(chrono.ChQuaternionD(0.325824906361366,0.618959649843522,0.314189496473588,0.641881642178576))
body_38.SetMass(0.1)
body_38.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_38.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_38.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_38.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_38.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_38.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_38.GetCollisionModel().BuildModel()
body_38.SetCollide(True)

exported_items.append(body_38)



# Rigid body part
body_39= chrono.ChBodyAuxRef()
body_39.SetName('robo_leg_link-6/link2-1')
body_39.SetPos(chrono.ChVectorD(1.58899208334844,0.0282055760792372,-2.1926628921386))
body_39.SetRot(chrono.ChQuaternionD(0.694136937375286,-7.49707509595302e-15,-0.719842977441093,3.8361427937933e-15))
body_39.SetMass(0.2)
body_39.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_39.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_39.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_39.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_39.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_39.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_39.GetCollisionModel().BuildModel()
body_39.SetCollide(True)

exported_items.append(body_39)



# Rigid body part
body_40= chrono.ChBodyAuxRef()
body_40.SetName('robo_leg_link-6/link2-2')
body_40.SetPos(chrono.ChVectorD(1.58899208334844,0.0952055760792394,-2.19266289213856))
body_40.SetRot(chrono.ChQuaternionD(7.25609966319856e-15,0.694136937375286,3.57864762532377e-15,0.719842977441093))
body_40.SetMass(0.2)
body_40.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_40.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_40.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_40.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_40.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_40.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_40.GetCollisionModel().BuildModel()
body_40.SetCollide(True)

exported_items.append(body_40)



# Rigid body part
body_41= chrono.ChBodyAuxRef()
body_41.SetName('robo_leg_link-6/single_bot1-1')
body_41.SetPos(chrono.ChVectorD(1.56879905479095,-0.0974870407654087,-2.08505100558054))
body_41.SetRot(chrono.ChQuaternionD(-0.190482777745549,0.249803569038372,0.675501686402177,0.667092167504496))
body_41.SetMass(3.5)
body_41.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_41.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_41.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_41.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_41_1 =chrono.ChMarker()
marker_41_1.SetName('My_marker')
body_41.AddMarker(marker_41_1)
marker_41_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.5812739305978,0.0342055760792374,-2.19494493817617),chrono.ChQuaternionD(0.693996254338615,-0.0144920242904995,-0.719697084475981,0.0139745050971125)))

# Collision shapes 
body_41.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_41.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_41.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_41.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_41.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_41.GetCollisionModel().BuildModel()
body_41.SetCollide(True)

exported_items.append(body_41)



# Rigid body part
body_42= chrono.ChBodyAuxRef()
body_42.SetName('robo_leg_link-9/link2-2')
body_42.SetPos(chrono.ChVectorD(1.96326777701361,0.0952055756532277,-1.96592858897912))
body_42.SetRot(chrono.ChQuaternionD(6.34892313667215e-15,0.296459971378337,1.31869061180102e-15,0.955045279225208))
body_42.SetMass(0.2)
body_42.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_42.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_42.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_42.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_42.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_42.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_42.GetCollisionModel().BuildModel()
body_42.SetCollide(True)

exported_items.append(body_42)



# Rigid body part
body_43= chrono.ChBodyAuxRef()
body_43.SetName('robo_leg_link-9/link2-1')
body_43.SetPos(chrono.ChVectorD(1.9632677770135,0.0282055756532279,-1.96592858897904))
body_43.SetRot(chrono.ChQuaternionD(-0.296459971378337,6.67382779670783e-15,0.955045279225208,-1.45673535904189e-15))
body_43.SetMass(0.2)
body_43.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_43.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_43.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_43.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_43.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_43.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_43.GetCollisionModel().BuildModel()
body_43.SetCollide(True)

exported_items.append(body_43)



# Rigid body part
body_44= chrono.ChBodyAuxRef()
body_44.SetName('robo_leg_link-9/leg-1')
body_44.SetPos(chrono.ChVectorD(1.96088101577675,0.034205575168286,-1.97293453265807))
body_44.SetRot(chrono.ChQuaternionD(0.43228532781602,0.26435238832494,0.134187664919489,0.851610756581522))
body_44.SetMass(0.1)
body_44.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_44.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_44.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_44.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_44.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_44.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_44.GetCollisionModel().BuildModel()
body_44.SetCollide(True)

exported_items.append(body_44)



# Rigid body part
body_45= chrono.ChBodyAuxRef()
body_45.SetName('robo_leg_link-9/single_bot1-1')
body_45.SetPos(chrono.ChVectorD(1.86481310758195,-0.0974870411913985,-1.9180249920575))
body_45.SetRot(chrono.ChQuaternionD(0.133504760716208,-0.0767351771798748,0.689030257547485,0.708184647979133))
body_45.SetMass(3.5)
body_45.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_45.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_45.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_45.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_45_1 =chrono.ChMarker()
marker_45_1.SetName('My_marker')
body_45.AddMarker(marker_45_1)
marker_45_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.96050161795674,0.0342055756532479,-1.97348676204352),chrono.ChQuaternionD(-0.296399886852098,0.0192271645606043,0.954851717029046,-0.00596839205356359)))

# Collision shapes 
body_45.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_45.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_45.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_45.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_45.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_45.GetCollisionModel().BuildModel()
body_45.SetCollide(True)

exported_items.append(body_45)



# Rigid body part
body_46= chrono.ChBodyAuxRef()
body_46.SetName('robo_leg_link-9/link1-1')
body_46.SetPos(chrono.ChVectorD(1.96326777701352,0.0952055756532122,-1.96592858897907))
body_46.SetRot(chrono.ChQuaternionD(2.88608927441743e-15,0.296459971378337,1.36518920817851e-14,0.955045279225208))
body_46.SetMass(0.2)
body_46.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_46.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_46.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_46.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_46)



# Rigid body part
body_47= chrono.ChBodyAuxRef()
body_47.SetName('robo_leg_link-10/single_bot1-1')
body_47.SetPos(chrono.ChVectorD(1.89816770557646,-0.0974870411838313,-1.84314542330169))
body_47.SetRot(chrono.ChQuaternionD(0.33164272090311,-0.283096312164373,0.618546136211219,0.653659132185182))
body_47.SetMass(3.5)
body_47.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_47.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_47.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_47.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_47_1 =chrono.ChMarker()
marker_47_1.SetName('My_marker')
body_47.AddMarker(marker_47_1)
marker_47_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(2.00844601359079,0.0342055756608154,-1.83471964563885),chrono.ChQuaternionD(-0.000210561427249927,0.0201322016373915,0.999797304508839,-4.23992452560725E-06)))

# Collision shapes 
body_47.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_47.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_47.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_47.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_47.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_47.GetCollisionModel().BuildModel()
body_47.SetCollide(True)

exported_items.append(body_47)



# Rigid body part
body_48= chrono.ChBodyAuxRef()
body_48.SetName('robo_leg_link-10/link2-1')
body_48.SetPos(chrono.ChVectorD(2.00644929750707,0.028205575660764,-1.82692280391384))
body_48.SetRot(chrono.ChQuaternionD(-0.000210604111090335,5.64132086898441e-15,0.999999977822954,-3.20056488415628e-15))
body_48.SetMass(0.2)
body_48.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_48.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_48.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_48.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_48.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_48.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_48.GetCollisionModel().BuildModel()
body_48.SetCollide(True)

exported_items.append(body_48)



# Rigid body part
body_49= chrono.ChBodyAuxRef()
body_49.SetName('robo_leg_link-10/link2-2')
body_49.SetPos(chrono.ChVectorD(2.00644929750848,0.095205575660701,-1.82692280391417))
body_49.SetRot(chrono.ChQuaternionD(5.30478450718153e-15,0.000210604111090367,3.20056488415628e-15,0.999999977822954))
body_49.SetMass(0.2)
body_49.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_49.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_49.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_49.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_49.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_49.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_49.GetCollisionModel().BuildModel()
body_49.SetCollide(True)

exported_items.append(body_49)



# Rigid body part
body_50= chrono.ChBodyAuxRef()
body_50.SetName('robo_leg_link-10/leg-1')
body_50.SetPos(chrono.ChVectorD(2.00844629582269,0.03420557561238,-1.83404964565574))
body_50.SetRot(chrono.ChQuaternionD(0.452634715165308,0.000187794849239428,9.53267339489991e-05,0.891696007770097))
body_50.SetMass(0.1)
body_50.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_50.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_50.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_50.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_50.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_50.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_50.GetCollisionModel().BuildModel()
body_50.SetCollide(True)

exported_items.append(body_50)



# Rigid body part
body_51= chrono.ChBodyAuxRef()
body_51.SetName('robo_leg_link-10/link1-1')
body_51.SetPos(chrono.ChVectorD(2.00644929750744,0.0952055756607767,-1.82692280391396))
body_51.SetRot(chrono.ChQuaternionD(5.67254589224447e-15,0.000210604111090272,1.63593101030005e-14,0.999999977822954))
body_51.SetMass(0.2)
body_51.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_51.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_51.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_51.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_51)



# Rigid body part
body_52= chrono.ChBodyAuxRef()
body_52.SetName('robo_leg_link-11/link2-1')
body_52.SetPos(chrono.ChVectorD(1.99327488858649,0.0282055756685825,-1.67567597333039))
body_52.SetRot(chrono.ChQuaternionD(0.0869865588227251,4.54137296637465e-15,0.996209485291212,-3.68899487318432e-15))
body_52.SetMass(0.2)
body_52.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_52.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_52.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_52.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_52.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_52.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_52.GetCollisionModel().BuildModel()
body_52.SetCollide(True)

exported_items.append(body_52)



# Rigid body part
body_53= chrono.ChBodyAuxRef()
body_53.SetName('robo_leg_link-11/single_bot1-1')
body_53.SetPos(chrono.ChVectorD(1.88945820222901,-0.0974870411762702,-1.71046350219809))
body_53.SetRot(chrono.ChQuaternionD(0.384314514604332,-0.339014737704131,0.587272143741113,0.626484469615611))
body_53.SetMass(3.5)
body_53.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_53.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_53.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_53.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_53_1 =chrono.ChMarker()
marker_53_1.SetName('My_marker')
body_53.AddMarker(marker_53_1)
marker_53_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.9965957753118,0.0342055756683768,-1.68300736660586),chrono.ChQuaternionD(0.0869689289681813,0.0200558906757441,0.996007580208796,0.00175123098079664)))

# Collision shapes 
body_53.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_53.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_53.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_53.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_53.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_53.GetCollisionModel().BuildModel()
body_53.SetCollide(True)

exported_items.append(body_53)



# Rigid body part
body_54= chrono.ChBodyAuxRef()
body_54.SetName('robo_leg_link-11/link2-2')
body_54.SetPos(chrono.ChVectorD(1.99327488858648,0.0952055756686292,-1.67567597333039))
body_54.SetRot(chrono.ChQuaternionD(4.49958119061047e-15,-0.0869865588227251,3.67898226024081e-15,0.996209485291212))
body_54.SetMass(0.2)
body_54.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_54.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_54.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_54.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_54.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_54.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_54.GetCollisionModel().BuildModel()
body_54.SetCollide(True)

exported_items.append(body_54)



# Rigid body part
body_55= chrono.ChBodyAuxRef()
body_55.SetName('robo_leg_link-11/leg-1')
body_55.SetPos(chrono.ChVectorD(1.9964796551536,0.0342055756576191,-1.68234750593182))
body_55.SetRot(chrono.ChQuaternionD(-0.0979330129755195,-0.0865652193761598,0.00855126950672966,0.991384115040037))
body_55.SetMass(0.1)
body_55.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_55.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_55.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_55.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_55.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_55.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_55.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_55.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_55.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_55.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_55.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_55.GetCollisionModel().BuildModel()
body_55.SetCollide(True)

exported_items.append(body_55)



# Rigid body part
body_56= chrono.ChBodyAuxRef()
body_56.SetName('robo_leg_link-11/link1-1')
body_56.SetPos(chrono.ChVectorD(1.99327488858649,0.095205575669298,-1.67567597333031))
body_56.SetRot(chrono.ChQuaternionD(5.6976120958504e-15,-0.086986558822725,1.69452590793839e-14,0.996209485291212))
body_56.SetMass(0.2)
body_56.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_56.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_56.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_56.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_56)



# Rigid body part
body_57= chrono.ChBodyAuxRef()
body_57.SetName('robo_leg_link-12/link2-2')
body_57.SetPos(chrono.ChVectorD(1.90540652890631,0.0952055756532793,-1.61586221971114))
body_57.SetRot(chrono.ChQuaternionD(-2.77990726478239e-16,0.774567390011396,2.29559310888374e-16,-0.632491389926324))
body_57.SetMass(0.2)
body_57.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_57.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_57.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_57.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_57.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_57.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_57.GetCollisionModel().BuildModel()
body_57.SetCollide(True)

exported_items.append(body_57)



# Rigid body part
body_58= chrono.ChBodyAuxRef()
body_58.SetName('robo_leg_link-12/single_bot1-1')
body_58.SetPos(chrono.ChVectorD(1.94299149971424,-0.0974870411913733,-1.71869920965352))
body_58.SetRot(chrono.ChQuaternionD(0.688895110260688,-0.685399902916871,0.134200387513429,0.194012257684155))
body_58.SetMass(3.5)
body_58.SetInertiaXX(chrono.ChVectorD(0.00243287100418674,0.00250960901009867,0.00110709858882453))
body_58.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-05,-0.000233417013594839,-1.75548754038152e-05))
body_58.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_58.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_58_1 =chrono.ChMarker()
marker_58_1.SetName('My_marker')
body_58.AddMarker(marker_58_1)
marker_58_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.91264534348175,0.0342055756532745,-1.61234409811843),chrono.ChQuaternionD(0.774410405867979,0.0127334444782963,0.632363200797332,0.0155937472232777)))

# Collision shapes 
body_58.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_58.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_58.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_58.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_58.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_58.GetCollisionModel().BuildModel()
body_58.SetCollide(True)

exported_items.append(body_58)



# Rigid body part
body_59= chrono.ChBodyAuxRef()
body_59.SetName('robo_leg_link-12/leg-1')
body_59.SetPos(chrono.ChVectorD(1.91198886781665,0.034205575104295,-1.61247803727323))
body_59.SetRot(chrono.ChQuaternionD(-0.286281606580131,0.690682369570119,0.350588799071038,-0.563993085121469))
body_59.SetMass(0.1)
body_59.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-06,5.11889399343246e-06,5.94333456279105e-06))
body_59.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-10,-6.97451787573893e-23,-2.11470064202402e-23))
body_59.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_59.GetAssets().push_back(body_4_1_level) 

# Collision shapes 
body_59.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_59.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00447410307383378,0.0100490000340686,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_59.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00922537624739966,0.00599102938516531,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_59.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0106251840891798,0.00284700949612767,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_59.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00736043666994747,0.00817459308025131,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_59.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.011,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_59.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00172077911544257,0.0108645717465465,0.00399999999999999),mr)
body_59.GetCollisionModel().BuildModel()
body_59.SetCollide(True)

exported_items.append(body_59)



# Rigid body part
body_60= chrono.ChBodyAuxRef()
body_60.SetName('robo_leg_link-12/link2-1')
body_60.SetPos(chrono.ChVectorD(1.90540652890647,0.0282055756532801,-1.61586221971195))
body_60.SetRot(chrono.ChQuaternionD(0.774567390011396,2.64273157900762e-16,0.632491389926324,2.23960303305731e-16))
body_60.SetMass(0.2)
body_60.SetInertiaXX(chrono.ChVectorD(0.000104983041023413,0.000208152025768899,0.000123483128699975))
body_60.SetInertiaXY(chrono.ChVectorD(-2.4713143785015e-10,-1.47996614267629e-10,-7.2032729945964e-10))
body_60.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.363578295109e-07,0.0335000115691779,0.0764730062719303),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_60.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_60.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_60.GetCollisionModel().AddBox(0.0113982379521279,0.0140083638039784,0.013,chrono.ChVectorD(-0.000134890389966315,0.0335,0.0511),mr)
body_60.GetCollisionModel().BuildModel()
body_60.SetCollide(True)

exported_items.append(body_60)



# Rigid body part
body_61= chrono.ChBodyAuxRef()
body_61.SetName('robo_leg_link-12/link1-1')
body_61.SetPos(chrono.ChVectorD(1.9054065289065,0.0952055756532614,-1.61586221971174))
body_61.SetRot(chrono.ChQuaternionD(-1.05964018005315e-14,0.774567390011396,-8.19806690250628e-15,-0.632491389926324))
body_61.SetMass(0.2)
body_61.SetInertiaXX(chrono.ChVectorD(0.00021102890321418,0.000342254939964149,0.000166734961647356))
body_61.SetInertiaXY(chrono.ChVectorD(1.44186233077584e-07,8.15219838791517e-10,1.07420085359569e-10))
body_61.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-2.37534873691438e-05,0.0334739058996041,1.03286017725802e-08),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_61.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_61)



# Rigid body part
body_62= chrono.ChBodyAuxRef()
body_62.SetName('object-1')
body_62.SetPos(chrono.ChVectorD(2.58738777521909,0.0100000000000007,-2.74788941954557))
body_62.SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,0,0))
body_62.SetMass(2.296712567856)
body_62.SetInertiaXX(chrono.ChVectorD(0.00200653250454309,0.000987831386851984,0.00200653250454309))
body_62.SetInertiaXY(chrono.ChVectorD(-2.18658709118247e-20,7.46322558787853e-21,3.66176004107828e-19))
body_62.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00635000000000003,0.000151190476190455,0.04445),chrono.ChQuaternionD(1,0,0,0)))
body_62.SetBodyFixed(True)

# Visualization shape 
body_62_1_shape = chrono.ChObjShapeFile() 
body_62_1_shape.SetFilename(shapes_dir +'body_62_1.obj') 
body_62_1_level = chrono.ChAssetLevel() 
body_62_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_62_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_62_1_level.GetAssets().push_back(body_62_1_shape) 
body_62.GetAssets().push_back(body_62_1_level) 

# Auxiliary marker (coordinate system feature)
marker_62_1 =chrono.ChMarker()
marker_62_1.SetName('object_marker')
body_62.AddMarker(marker_62_1)
marker_62_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(2.59373777521908,0.0100000000000007,-2.74804061002176),chrono.ChQuaternionD(-9.65854120745433E-17,9.65854120745432E-17,0.707106781186547,0.707106781186548)))

# Collision shapes 
body_62.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_62.GetCollisionModel().AddBox(0.0254,0.0254,0.04445,chrono.ChVectorD(0.00635000000000003,0.000151190476190454,0.04445),mr)
body_62.GetCollisionModel().BuildModel()
body_62.SetCollide(True)

exported_items.append(body_62)




# Mate constraint: Concentric9 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_31 , SW name: robo_leg_link-8/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-9/link2-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.92011834970288,0.108205575653221,-2.02873438231419)
dA = chrono.ChVectorD(-5.12902367818727e-15,-1,1.40314098715331e-14)
cB = chrono.ChVectorD(1.92011835412113,0.015205575653228,-2.02873437934948)
dB = chrono.ChVectorD(1.18811210869652e-14,1,-6.73072708679001e-15)
link_1.SetFlipped(True)
link_1.Initialize(body_31,body_43,False,cA,cB,dA,dB)
link_1.SetName("Concentric9")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.92011834970288,0.108205575653221,-2.02873438231419)
cB = chrono.ChVectorD(1.92011835412113,0.015205575653228,-2.02873437934948)
dA = chrono.ChVectorD(-5.12902367818727e-15,-1,1.40314098715331e-14)
dB = chrono.ChVectorD(1.18811210869652e-14,1,-6.73072708679001e-15)
link_2.Initialize(body_31,body_43,False,cA,cB,dA,dB)
link_2.SetName("Concentric9")
exported_items.append(link_2)


# Mate constraint: Coincident22 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_31 , SW name: robo_leg_link-8/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-9/link2-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95838566710164,0.0757139394571999,-2.01166914223082)
cB = chrono.ChVectorD(1.89639183418425,0.0757139394572065,-2.06326932182352)
dA = chrono.ChVectorD(4.88878366979852e-15,1,-1.34926925368823e-14)
dB = chrono.ChVectorD(1.18811210869652e-14,1,-6.73072708679001e-15)
link_3.Initialize(body_31,body_43,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident22")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95838566710164,0.0757139394571999,-2.01166914223082)
dA = chrono.ChVectorD(4.88878366979852e-15,1,-1.34926925368823e-14)
cB = chrono.ChVectorD(1.89639183418425,0.0757139394572065,-2.06326932182352)
dB = chrono.ChVectorD(1.18811210869652e-14,1,-6.73072708679001e-15)
link_4.Initialize(body_31,body_43,False,cA,cB,dA,dB)
link_4.SetName("Coincident22")
exported_items.append(link_4)


# Mate constraint: Concentric11 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_33 , SW name: robo_leg_link-7/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-8/link2-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.78093126495663,0.108205575653217,-2.09080460703937)
dA = chrono.ChVectorD(-5.02375918642883e-15,-1,1.43496325932801e-14)
cB = chrono.ChVectorD(1.78093125730302,0.0152055756532202,-2.09080461116243)
dB = chrono.ChVectorD(5.48807724404125e-15,1,-1.46519436799504e-14)
link_5.SetFlipped(True)
link_5.Initialize(body_33,body_30,False,cA,cB,dA,dB)
link_5.SetName("Concentric11")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.78093126495663,0.108205575653217,-2.09080460703937)
cB = chrono.ChVectorD(1.78093125730302,0.0152055756532202,-2.09080461116243)
dA = chrono.ChVectorD(-5.02375918642883e-15,-1,1.43496325932801e-14)
dB = chrono.ChVectorD(5.48807724404125e-15,1,-1.46519436799504e-14)
link_6.Initialize(body_33,body_30,False,cA,cB,dA,dB)
link_6.SetName("Concentric11")
exported_items.append(link_6)


# Mate constraint: Coincident24 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_33 , SW name: robo_leg_link-7/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-8/link2-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.81276578667949,0.0757139394571961,-2.06356175060549)
cB = chrono.ChVectorD(1.74266393990427,0.0757139394571986,-2.1078698512458)
dA = chrono.ChVectorD(4.64024134394706e-15,1,-1.39014744819154e-14)
dB = chrono.ChVectorD(5.48807724404125e-15,1,-1.46519436799504e-14)
link_7.Initialize(body_33,body_30,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident24")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.81276578667949,0.0757139394571961,-2.06356175060549)
dA = chrono.ChVectorD(4.64024134394706e-15,1,-1.39014744819154e-14)
cB = chrono.ChVectorD(1.74266393990427,0.0757139394571986,-2.1078698512458)
dB = chrono.ChVectorD(5.48807724404125e-15,1,-1.46519436799504e-14)
link_8.Initialize(body_33,body_30,False,cA,cB,dA,dB)
link_8.SetName("Coincident24")
exported_items.append(link_8)


# Mate constraint: Concentric13 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_40 , SW name: robo_leg_link-6/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link2-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.66514173035065,0.108205576079239,-2.18989318792366)
dA = chrono.ChVectorD(-5.4678483962789e-15,-1,1.52308721190764e-14)
cB = chrono.ChVectorD(1.66514173964519,0.0152055756532162,-2.18989318270857)
dB = chrono.ChVectorD(5.49560397189452e-15,1,-1.49671941507279e-14)
link_9.SetFlipped(True)
link_9.Initialize(body_40,body_35,False,cA,cB,dA,dB)
link_9.SetName("Concentric13")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.66514173035065,0.108205576079239,-2.18989318792366)
cB = chrono.ChVectorD(1.66514173964519,0.0152055756532162,-2.18989318270857)
dA = chrono.ChVectorD(-5.4678483962789e-15,-1,1.52308721190764e-14)
dB = chrono.ChVectorD(5.49560397189452e-15,1,-1.49671941507279e-14)
link_10.Initialize(body_40,body_35,False,cA,cB,dA,dB)
link_10.SetName("Concentric13")
exported_items.append(link_10)


# Mate constraint: Coincident26 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_40 , SW name: robo_leg_link-6/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link2-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.70701404280988,0.0757139398832173,-2.18837021408371)
cB = chrono.ChVectorD(1.63330721792233,0.0757139394571944,-2.21713603914245)
dA = chrono.ChVectorD(5.44640836693866e-15,1,-1.46414046022225e-14)
dB = chrono.ChVectorD(5.49560397189452e-15,1,-1.49671941507279e-14)
link_11.Initialize(body_40,body_35,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident26")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.70701404280988,0.0757139398832173,-2.18837021408371)
dA = chrono.ChVectorD(5.44640836693866e-15,1,-1.46414046022225e-14)
cB = chrono.ChVectorD(1.63330721792233,0.0757139394571944,-2.21713603914245)
dB = chrono.ChVectorD(5.49560397189452e-15,1,-1.49671941507279e-14)
link_12.Initialize(body_40,body_35,False,cA,cB,dA,dB)
link_12.SetName("Coincident26")
exported_items.append(link_12)


# Mate constraint: Concentric14 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_26 , SW name: robo_leg_link-5/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-6/link2-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.51284244191646,0.108205581764713,-2.19543259738667)
dA = chrono.ChVectorD(-5.19965811488746e-15,-1,1.64378822244325e-14)
cB = chrono.ChVectorD(1.51284243634622,0.0152055760792376,-2.1954325963535)
dB = chrono.ChVectorD(5.46781689790468e-15,1,-1.59308343939014e-14)
link_13.SetFlipped(True)
link_13.Initialize(body_26,body_39,False,cA,cB,dA,dB)
link_13.SetName("Concentric14")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.51284244191646,0.108205581764713,-2.19543259738667)
cB = chrono.ChVectorD(1.51284243634622,0.0152055760792376,-2.1954325963535)
dA = chrono.ChVectorD(-5.19965811488746e-15,-1,1.64378822244325e-14)
dB = chrono.ChVectorD(5.46781689790468e-15,1,-1.59308343939014e-14)
link_14.Initialize(body_26,body_39,False,cA,cB,dA,dB)
link_14.SetName("Concentric14")
exported_items.append(link_14)


# Mate constraint: Coincident27 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_26 , SW name: robo_leg_link-5/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-6/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.53024115519472,0.0757139455686911,-2.23354945423847)
cB = chrono.ChVectorD(1.470970123887,0.0757139398832162,-2.19695557019345)
dA = chrono.ChVectorD(5.73625730511814e-15,1,-1.61929476671979e-14)
dB = chrono.ChVectorD(5.46781689790468e-15,1,-1.59308343939014e-14)
link_15.Initialize(body_26,body_39,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident27")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.53024115519472,0.0757139455686911,-2.23354945423847)
dA = chrono.ChVectorD(5.73625730511814e-15,1,-1.61929476671979e-14)
cB = chrono.ChVectorD(1.470970123887,0.0757139398832162,-2.19695557019345)
dB = chrono.ChVectorD(5.46781689790468e-15,1,-1.59308343939014e-14)
link_16.Initialize(body_26,body_39,False,cA,cB,dA,dB)
link_16.SetName("Coincident27")
exported_items.append(link_16)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_18 , SW name: robo_leg_link-4/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-5/link2-1 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.44955928919794,0.10820557565322,-2.05679276113317)
dA = chrono.ChVectorD(-6.10622663543836e-15,-1,1.59872115546023e-14)
cB = chrono.ChVectorD(1.44955929425923,0.0152055807421761,-2.05679276590648)
dB = chrono.ChVectorD(5.74540415243519e-15,1,-1.60982338570648e-14)
link_17.SetFlipped(True)
link_17.Initialize(body_18,body_22,False,cA,cB,dA,dB)
link_17.SetName("Concentric15")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateGeneric()
link_18.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.44955928919794,0.10820557565322,-2.05679276113317)
cB = chrono.ChVectorD(1.44955929425923,0.0152055807421761,-2.05679276590648)
dA = chrono.ChVectorD(-6.10622663543836e-15,-1,1.59872115546023e-14)
dB = chrono.ChVectorD(5.74540415243519e-15,1,-1.60982338570648e-14)
link_18.Initialize(body_18,body_22,False,cA,cB,dA,dB)
link_18.SetName("Concentric15")
exported_items.append(link_18)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_18 , SW name: robo_leg_link-4/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-5/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.44489088434564,0.0757139394571978,-2.09843187731177)
cB = chrono.ChVectorD(1.43216058098097,0.0757139445461552,-2.01867590905468)
dA = chrono.ChVectorD(6.69241127574669e-15,1,-1.6052932143085e-14)
dB = chrono.ChVectorD(5.74540415243519e-15,1,-1.60982338570648e-14)
link_19.Initialize(body_18,body_22,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident29")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.44489088434564,0.0757139394571978,-2.09843187731177)
dA = chrono.ChVectorD(6.69241127574669e-15,1,-1.6052932143085e-14)
cB = chrono.ChVectorD(1.43216058098097,0.0757139445461552,-2.01867590905468)
dB = chrono.ChVectorD(5.74540415243519e-15,1,-1.60982338570648e-14)
link_20.Initialize(body_18,body_22,False,cA,cB,dA,dB)
link_20.SetName("Coincident29")
exported_items.append(link_20)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: robo_leg_link-3/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-4/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.46653935748793,0.108205576079236,-1.90534165864944)
dA = chrono.ChVectorD(-6.35075884765483e-15,-1,1.57425209190776e-14)
cB = chrono.ChVectorD(1.46653935839766,0.0152055756532219,-1.90534165598708)
dB = chrono.ChVectorD(6.20337115009306e-15,1,-1.59993546189341e-14)
link_21.SetFlipped(True)
link_21.Initialize(body_8,body_17,False,cA,cB,dA,dB)
link_21.SetName("Concentric16")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateGeneric()
link_22.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.46653935748793,0.108205576079236,-1.90534165864944)
cB = chrono.ChVectorD(1.46653935839766,0.0152055756532219,-1.90534165598708)
dA = chrono.ChVectorD(-6.35075884765483e-15,-1,1.57425209190776e-14)
dB = chrono.ChVectorD(6.20337115009306e-15,1,-1.59993546189341e-14)
link_22.Initialize(body_8,body_17,False,cA,cB,dA,dB)
link_22.SetName("Concentric16")
exported_items.append(link_22)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_8 , SW name: robo_leg_link-3/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-4/link2-1 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.43244900000339,0.0757139398832146,-1.92970264234419)
cB = chrono.ChVectorD(1.47120776324996,0.075713939457201,-1.86370253980848)
dA = chrono.ChVectorD(6.69370643576922e-15,1,-1.62224360991776e-14)
dB = chrono.ChVectorD(6.20337115009306e-15,1,-1.59993546189341e-14)
link_23.Initialize(body_8,body_17,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident30")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.43244900000339,0.0757139398832146,-1.92970264234419)
dA = chrono.ChVectorD(6.69370643576922e-15,1,-1.62224360991776e-14)
cB = chrono.ChVectorD(1.47120776324996,0.075713939457201,-1.86370253980848)
dB = chrono.ChVectorD(6.20337115009306e-15,1,-1.59993546189341e-14)
link_24.Initialize(body_8,body_17,False,cA,cB,dA,dB)
link_24.SetName("Coincident30")
exported_items.append(link_24)


# Mate constraint: Concentric17 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_42 , SW name: robo_leg_link-9/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-10/link2-1 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00641719990597,0.108205575653228,-1.90312279860867)
dA = chrono.ChVectorD(-1.13407547241984e-14,-1,6.24500451351651e-15)
cB = chrono.ChVectorD(2.00641720144125,0.0152055756607635,-1.90312279715428)
dB = chrono.ChVectorD(1.12687636999453e-14,1,-6.41153796721028e-15)
link_25.SetFlipped(True)
link_25.Initialize(body_42,body_48,False,cA,cB,dA,dB)
link_25.SetName("Concentric17")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(2.00641719990597,0.108205575653228,-1.90312279860867)
cB = chrono.ChVectorD(2.00641720144125,0.0152055756607635,-1.90312279715428)
dA = chrono.ChVectorD(-1.13407547241984e-14,-1,6.24500451351651e-15)
dB = chrono.ChVectorD(1.12687636999453e-14,1,-6.41153796721028e-15)
link_26.Initialize(body_42,body_48,False,cA,cB,dA,dB)
link_26.SetName("Concentric17")
exported_items.append(link_26)


# Mate constraint: Coincident36 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_42 , SW name: robo_leg_link-9/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-10/link2-1 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.03014371984285,0.075713939457206,-1.86858785613464)
cB = chrono.ChVectorD(2.00639955281713,0.0757139394647417,-1.9450227934374)
dA = chrono.ChVectorD(1.08545807923267e-14,1,-5.91098874105603e-15)
dB = chrono.ChVectorD(1.12687636999453e-14,1,-6.41153796721028e-15)
link_27.Initialize(body_42,body_48,False,cA,cB,dB)
link_27.SetDistance(0)
link_27.SetName("Coincident36")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.03014371984285,0.075713939457206,-1.86858785613464)
dA = chrono.ChVectorD(1.08545807923267e-14,1,-5.91098874105603e-15)
cB = chrono.ChVectorD(2.00639955281713,0.0757139394647417,-1.9450227934374)
dB = chrono.ChVectorD(1.12687636999453e-14,1,-6.41153796721028e-15)
link_28.Initialize(body_42,body_48,False,cA,cB,dA,dB)
link_28.SetName("Coincident36")
exported_items.append(link_28)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-2/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-3/link2-1 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.59053388769207,0.108205575653223,-1.81673511676836)
dA = chrono.ChVectorD(-5.96142907481141e-15,-1,1.60442647917024e-14)
cB = chrono.ChVectorD(1.59053387969904,0.0152055760792344,-1.81673512129672)
dB = chrono.ChVectorD(6.28663787693995e-15,1,-1.5779044737485e-14)
link_29.SetFlipped(True)
link_29.Initialize(body_14,body_9,False,cA,cB,dA,dB)
link_29.SetName("Concentric22")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.59053388769207,0.108205575653223,-1.81673511676836)
cB = chrono.ChVectorD(1.59053387969904,0.0152055760792344,-1.81673512129672)
dA = chrono.ChVectorD(-5.96142907481141e-15,-1,1.60442647917024e-14)
dB = chrono.ChVectorD(6.28663787693995e-15,1,-1.5779044737485e-14)
link_30.Initialize(body_14,body_9,False,cA,cB,dA,dB)
link_30.SetName("Concentric22")
exported_items.append(link_30)


# Mate constraint: Coincident55 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-2/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-3/link2-1 ,  SW ref.type:2 (2)

link_31 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.55979897588472,0.0757139394572009,-1.84521274300713)
cB = chrono.ChVectorD(1.62462423718359,0.075713939883213,-1.79237413760197)
dA = chrono.ChVectorD(6.36232968523749e-15,1,-1.64769428808679e-14)
dB = chrono.ChVectorD(6.28663787693995e-15,1,-1.5779044737485e-14)
link_31.Initialize(body_14,body_9,False,cA,cB,dB)
link_31.SetDistance(0)
link_31.SetName("Coincident55")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.55979897588472,0.0757139394572009,-1.84521274300713)
dA = chrono.ChVectorD(6.36232968523749e-15,1,-1.64769428808679e-14)
cB = chrono.ChVectorD(1.62462423718359,0.075713939883213,-1.79237413760197)
dB = chrono.ChVectorD(6.28663787693995e-15,1,-1.5779044737485e-14)
link_32.Initialize(body_14,body_9,False,cA,cB,dA,dB)
link_32.SetName("Coincident55")
exported_items.append(link_32)


# Mate constraint: Concentric27 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_54 , SW name: robo_leg_link-11/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-12/link2-1 ,  SW ref.type:2 (2)

link_33 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.9800683869337,0.10820557566863,-1.60062913253016)
dA = chrono.ChVectorD(-9.6034291630076e-15,-1,6.55378529224038e-15)
cB = chrono.ChVectorD(1.9800683869637,0.0152055756532801,-1.60062913232158)
dB = chrono.ChVectorD(0,1,6.93889390390723e-16)
link_33.SetFlipped(True)
link_33.Initialize(body_54,body_60,False,cA,cB,dA,dB)
link_33.SetName("Concentric27")
exported_items.append(link_33)

link_34 = chrono.ChLinkMateGeneric()
link_34.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.9800683869337,0.10820557566863,-1.60062913253016)
cB = chrono.ChVectorD(1.9800683869637,0.0152055756532801,-1.60062913232158)
dA = chrono.ChVectorD(-9.6034291630076e-15,-1,6.55378529224038e-15)
dB = chrono.ChVectorD(0,1,6.93889390390723e-16)
link_34.Initialize(body_54,body_60,False,cA,cB,dA,dB)
link_34.SetName("Concentric27")
exported_items.append(link_34)


# Mate constraint: Coincident58 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_54 , SW name: robo_leg_link-11/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-12/link2-1 ,  SW ref.type:2 (2)

link_35 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.97280654416136,0.0757139394726086,-1.5593632187568)
cB = chrono.ChVectorD(2.02112261075107,0.0757139394572585,-1.59225293334972)
dA = chrono.ChVectorD(9.02249836902836e-15,1,-6.65601562477715e-15)
dB = chrono.ChVectorD(0,1,6.93889390390723e-16)
link_35.Initialize(body_54,body_60,False,cA,cB,dB)
link_35.SetDistance(0)
link_35.SetName("Coincident58")
exported_items.append(link_35)

link_36 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.97280654416136,0.0757139394726086,-1.5593632187568)
dA = chrono.ChVectorD(9.02249836902836e-15,1,-6.65601562477715e-15)
cB = chrono.ChVectorD(2.02112261075107,0.0757139394572585,-1.59225293334972)
dB = chrono.ChVectorD(0,1,6.93889390390723e-16)
link_36.Initialize(body_54,body_60,False,cA,cB,dA,dB)
link_36.SetName("Coincident58")
exported_items.append(link_36)


# Mate constraint: Concentric28 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_49 , SW name: robo_leg_link-10/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-11/link2-1 ,  SW ref.type:2 (2)

link_37 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.0064813935743,0.108205575660701,-1.75072281067374)
dA = chrono.ChVectorD(-1.06165076729781e-14,-1,6.40806852025833e-15)
cB = chrono.ChVectorD(2.00648139023928,0.0152055756685819,-1.75072281413062)
dB = chrono.ChVectorD(9.68669588985449e-15,1,-6.56419363309624e-15)
link_37.SetFlipped(True)
link_37.Initialize(body_49,body_52,False,cA,cB,dA,dB)
link_37.SetName("Concentric28")
exported_items.append(link_37)

link_38 = chrono.ChLinkMateGeneric()
link_38.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(2.0064813935743,0.108205575660701,-1.75072281067374)
cB = chrono.ChVectorD(2.00648139023928,0.0152055756685819,-1.75072281413062)
dA = chrono.ChVectorD(-1.06165076729781e-14,-1,6.40806852025833e-15)
dB = chrono.ChVectorD(9.68669588985449e-15,1,-6.56419363309624e-15)
link_38.Initialize(body_49,body_52,False,cA,cB,dA,dB)
link_38.SetName("Concentric28")
exported_items.append(link_38)


# Mate constraint: Coincident59 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_49 , SW name: robo_leg_link-10/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-11/link2-1 ,  SW ref.type:2 (2)

link_39 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.00649904219842,0.0757139394646802,-1.70882281439061)
cB = chrono.ChVectorD(2.01374323301161,0.07571393947256,-1.79198872790397)
dA = chrono.ChVectorD(1.00266504304565e-14,1,-6.40782006752133e-15)
dB = chrono.ChVectorD(9.68669588985449e-15,1,-6.56419363309624e-15)
link_39.Initialize(body_49,body_52,False,cA,cB,dB)
link_39.SetDistance(0)
link_39.SetName("Coincident59")
exported_items.append(link_39)

link_40 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00649904219842,0.0757139394646802,-1.70882281439061)
dA = chrono.ChVectorD(1.00266504304565e-14,1,-6.40782006752133e-15)
cB = chrono.ChVectorD(2.01374323301161,0.07571393947256,-1.79198872790397)
dB = chrono.ChVectorD(9.68669588985449e-15,1,-6.56419363309624e-15)
link_40.Initialize(body_49,body_52,False,cA,cB,dA,dB)
link_40.SetName("Coincident59")
exported_items.append(link_40)


# Mate constraint: Parallel2 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)

link_41 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.71990397838016,0.0272055756532369,-2.18065263842081)
dB = chrono.ChVectorD(3.88578058618805e-16,-1,7.97972798949331e-15)
link_41.Initialize(body_29,body_36,False,cA,cB,dA,dB)
link_41.SetName("Parallel2")
exported_items.append(link_41)


# Mate constraint: Parallel3 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)

link_42 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.56146497488945,0.0272055760792373,-2.22226800571525)
dB = chrono.ChVectorD(-5.24580379135386e-15,-1,7.70217223333702e-15)
link_42.Initialize(body_29,body_41,False,cA,cB,dA,dB)
link_42.SetName("Parallel3")
exported_items.append(link_42)


# Mate constraint: Parallel4 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)

link_43 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.44332706007836,0.0272055756532142,-2.11197835271695)
dB = chrono.ChVectorD(-1.28022592527088e-14,-1,1.22124532708767e-14)
link_43.Initialize(body_29,body_23,False,cA,cB,dA,dB)
link_43.SetName("Parallel4")
exported_items.append(link_43)


# Mate constraint: Parallel5 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)

link_44 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.43282718099906,0.0272055756532205,-1.9494753114559)
dB = chrono.ChVectorD(-1.44606548957427e-14,-1,1.70002900645727e-14)
link_44.Initialize(body_29,body_19,False,cA,cB,dA,dB)
link_44.SetName("Parallel5")
exported_items.append(link_44)


# Mate constraint: Parallel6 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)

link_45 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.53517418780068,0.0272055760792373,-1.82116174110411)
dB = chrono.ChVectorD(-1.12687636999453e-14,-1,2.27092650240124e-14)
link_45.Initialize(body_29,body_10,False,cA,cB,dA,dB)
link_45.SetName("Parallel6")
exported_items.append(link_45)


# Mate constraint: Parallel7 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_46 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.64796884682585,0.0272055756532179,-1.72454930484867)
dB = chrono.ChVectorD(-1.14769305170626e-14,-1,2.14758766325929e-14)
link_46.Initialize(body_29,body_15,False,cA,cB,dA,dB)
link_46.SetName("Parallel7")
exported_items.append(link_46)


# Mate constraint: Parallel8 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_47 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.77522994217606,0.0272055737557717,-1.63264637619669)
dB = chrono.ChVectorD(-1.74860126378462e-15,-1,1.80064296806393e-15)
link_47.Initialize(body_29,body_5,False,cA,cB,dA,dB)
link_47.SetName("Parallel8")
exported_items.append(link_47)


# Mate constraint: Parallel9 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)

link_48 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.92770011695785,0.0272055756532748,-1.58213981793194)
dB = chrono.ChVectorD(-1.94289029309402e-15,-1,7.44890260584441e-15)
link_48.Initialize(body_29,body_58,False,cA,cB,dA,dB)
link_48.SetName("Parallel9")
exported_items.append(link_48)


# Mate constraint: Parallel10 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)

link_49 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(2.02638147380963,0.0272055756683767,-1.69887421604098)
dB = chrono.ChVectorD(-1.83533743758346e-15,-1,8.24340595784179e-15)
link_49.Initialize(body_29,body_53,False,cA,cB,dA,dB)
link_49.SetName("Parallel10")
exported_items.append(link_49)


# Mate constraint: Parallel11 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)

link_50 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(2.03502225473496,0.0272055756608153,-1.85551984161503)
dB = chrono.ChVectorD(-2.82412981889024e-15,-1,6.66133814775094e-15)
link_50.Initialize(body_29,body_47,False,cA,cB,dA,dB)
link_50.SetName("Parallel11")
exported_items.append(link_50)


# Mate constraint: Parallel12 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)

link_51 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83606033142533,0.0272055756532359,-2.09751845033339)
dA = chrono.ChVectorD(-1.12410081243297e-15,-1,8.00141203294302e-15)
cB = chrono.ChVectorD(1.97064149439361,0.0272055756532478,-2.00567569883624)
dB = chrono.ChVectorD(-4.69069227904129e-15,-1,2.51187959321442e-15)
link_51.Initialize(body_29,body_45,False,cA,cB,dA,dB)
link_51.SetName("Parallel12")
exported_items.append(link_51)


# Mate constraint: Concentric29 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: robo_leg_link-1/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-2/link2-1 ,  SW ref.type:2 (2)

link_52 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.70232386930843,0.108205573755214,-1.71315540046707)
dA = chrono.ChVectorD(-1.87350135405495e-15,-1,1.79110198894605e-15)
cB = chrono.ChVectorD(1.70232387717771,0.0152055756532234,-1.71315539746579)
dB = chrono.ChVectorD(5.94004323980378e-15,1,-1.60829991512724e-14)
link_52.SetFlipped(True)
link_52.Initialize(body_2,body_12,False,cA,cB,dA,dB)
link_52.SetName("Concentric29")
exported_items.append(link_52)

link_53 = chrono.ChLinkMateGeneric()
link_53.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.70232386930843,0.108205573755214,-1.71315540046707)
cB = chrono.ChVectorD(1.70232387717771,0.0152055756532234,-1.71315539746579)
dA = chrono.ChVectorD(-1.87350135405495e-15,-1,1.79110198894605e-15)
dB = chrono.ChVectorD(5.94004323980378e-15,1,-1.60829991512724e-14)
link_53.Initialize(body_2,body_12,False,cA,cB,dA,dB)
link_53.SetName("Concentric29")
exported_items.append(link_53)


# Mate constraint: Coincident71 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: robo_leg_link-1/link2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-2/link2-1 ,  SW ref.type:2 (2)

link_54 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.66701657659224,0.0757139375591927,-1.735716541419)
cB = chrono.ChVectorD(1.73305878898505,0.0757139394572021,-1.68467777122703)
dA = chrono.ChVectorD(2.19111122447416e-15,1,-2.28814886651015e-15)
dB = chrono.ChVectorD(5.94004323980378e-15,1,-1.60829991512724e-14)
link_54.Initialize(body_2,body_12,False,cA,cB,dB)
link_54.SetDistance(0)
link_54.SetName("Coincident71")
exported_items.append(link_54)

link_55 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.66701657659224,0.0757139375591927,-1.735716541419)
dA = chrono.ChVectorD(2.19111122447416e-15,1,-2.28814886651015e-15)
cB = chrono.ChVectorD(1.73305878898505,0.0757139394572021,-1.68467777122703)
dB = chrono.ChVectorD(5.94004323980378e-15,1,-1.60829991512724e-14)
link_55.Initialize(body_2,body_12,False,cA,cB,dA,dB)
link_55.SetName("Coincident71")
exported_items.append(link_55)


# Mate constraint: Concentric30 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: robo_leg_link-1/link2-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-12/link2-2 ,  SW ref.type:2 (2)

link_56 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83074466668184,0.0152055737556763,-1.63109530784044)
dA = chrono.ChVectorD(1.88737914186277e-15,1,-1.7650811368064e-15)
cB = chrono.ChVectorD(1.83074467084908,0.108205575653279,-1.63109530710151)
dB = chrono.ChVectorD(-1.04083408558608e-17,-1,-7.21644966006352e-16)
link_56.SetFlipped(True)
link_56.Initialize(body_3,body_57,False,cA,cB,dA,dB)
link_56.SetName("Concentric30")
exported_items.append(link_56)

link_57 = chrono.ChLinkMateGeneric()
link_57.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.83074466668184,0.0152055737556763,-1.63109530784044)
cB = chrono.ChVectorD(1.83074467084908,0.108205575653279,-1.63109530710151)
dA = chrono.ChVectorD(1.88737914186277e-15,1,-1.7650811368064e-15)
dB = chrono.ChVectorD(-1.04083408558608e-17,-1,-7.21644966006352e-16)
link_57.Initialize(body_3,body_57,False,cA,cB,dA,dB)
link_57.SetName("Concentric30")
exported_items.append(link_57)


# Mate constraint: Coincident72 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: robo_leg_link-1/link2-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-12/link2-2 ,  SW ref.type:2 (2)

link_58 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.86605195939804,0.0757139375596547,-1.6085341668885)
cB = chrono.ChVectorD(1.78969044706171,0.0757139394572578,-1.63947150607338)
dA = chrono.ChVectorD(1.88737914186277e-15,1,-1.7650811368064e-15)
dB = chrono.ChVectorD(1.28326289940445e-16,1,1.43694288554015e-16)
link_58.Initialize(body_3,body_57,False,cA,cB,dB)
link_58.SetDistance(0)
link_58.SetName("Coincident72")
exported_items.append(link_58)

link_59 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.86605195939804,0.0757139375596547,-1.6085341668885)
dA = chrono.ChVectorD(1.88737914186277e-15,1,-1.7650811368064e-15)
cB = chrono.ChVectorD(1.78969044706171,0.0757139394572578,-1.63947150607338)
dB = chrono.ChVectorD(1.28326289940445e-16,1,1.43694288554015e-16)
link_59.Initialize(body_3,body_57,False,cA,cB,dA,dB)
link_59.SetName("Coincident72")
exported_items.append(link_59)


# Mate constraint: Coincident73 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: graound-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_62 , SW name: object-1 ,  SW ref.type:2 (2)

link_60 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
cB = chrono.ChVectorD(2.58738777521909,0.0100000000000007,-2.74788941954557)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
dB = chrono.ChVectorD(0,-1,-2.88998133608509e-16)
link_60.Initialize(body_1,body_62,False,cA,cB,dB)
link_60.SetDistance(0)
link_60.SetName("Coincident73")
exported_items.append(link_60)

link_61 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
cB = chrono.ChVectorD(2.58738777521909,0.0100000000000007,-2.74788941954557)
dB = chrono.ChVectorD(0,-1,-2.88998133608509e-16)
link_61.SetFlipped(True)
link_61.Initialize(body_1,body_62,False,cA,cB,dA,dB)
link_61.SetName("Coincident73")
exported_items.append(link_61)


# Mate constraint: Parallel15 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: graound-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_62 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
cB = chrono.ChVectorD(1.64796884682585,0.0272055756532179,-1.72454930484867)
dB = chrono.ChVectorD(-1.14769305170626e-14,-1,2.14758766325929e-14)
link_62.SetFlipped(True)
link_62.Initialize(body_1,body_15,False,cA,cB,dA,dB)
link_62.SetName("Parallel15")
exported_items.append(link_62)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-1/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.77146213137511,0.0342055711593255,-1.66660303514652)
dA = chrono.ChVectorD(-0.842656150744453,1.66533453693773e-16,-0.538452051358839)
cB = chrono.ChVectorD(1.76472088241666,0.0342055737557716,-1.67091064894522)
dB = chrono.ChVectorD(0.842656150744447,3.20576898360514e-15,0.538452051358849)
link_1.SetFlipped(True)
link_1.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.77146213137511,0.0342055711593255,-1.66660303514652)
cB = chrono.ChVectorD(1.76472088241666,0.0342055737557716,-1.67091064894522)
dA = chrono.ChVectorD(-0.842656150744453,1.66533453693773e-16,-0.538452051358839)
dB = chrono.ChVectorD(0.842656150744447,3.20576898360514e-15,0.538452051358849)
link_2.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.727055290038,0.0282055737557717,-1.66342967997288)
cB = chrono.ChVectorD(1.76653426799513,0.0282055737557866,-1.67212535415382)
dA = chrono.ChVectorD(1.73472347597681e-15,1,-1.80324505327789e-15)
dB = chrono.ChVectorD(-2.02060590481778e-14,-1,-1.95555377446865e-14)
link_3.Initialize(body_5,body_6,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.727055290038,0.0282055737557717,-1.66342967997288)
dA = chrono.ChVectorD(1.73472347597681e-15,1,-1.80324505327789e-15)
cB = chrono.ChVectorD(1.76653426799513,0.0282055737557866,-1.67212535415382)
dB = chrono.ChVectorD(-2.02060590481778e-14,-1,-1.95555377446865e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.74121297476917,0.0805155737557915,-1.66115801271655)
dA = chrono.ChVectorD(-0.53845205135884,2.12330153459561e-15,0.842656150744453)
cB = chrono.ChVectorD(1.76608668135965,0.0384055737557872,-1.70471426216686)
dB = chrono.ChVectorD(0.538452051358839,5.60662627435704e-15,-0.842656150744453)
link_5.SetFlipped(True)
link_5.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-1/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.76472088216915,0.0342055711593255,-1.67091065155739)
cB = chrono.ChVectorD(1.76364343986189,0.0322045737557716,-1.66922449398758)
dA = chrono.ChVectorD(-0.842656150744453,1.66533453693773e-16,-0.538452051358839)
dB = chrono.ChVectorD(0.842656150744447,3.20576898360514e-15,0.538452051358849)
link_6.Initialize(body_4,body_5,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.76472088216915,0.0342055711593255,-1.67091065155739)
dA = chrono.ChVectorD(-0.842656150744453,1.66533453693773e-16,-0.538452051358839)
cB = chrono.ChVectorD(1.76364343986189,0.0322045737557716,-1.66922449398758)
dB = chrono.ChVectorD(0.842656150744447,3.20576898360514e-15,0.538452051358849)
link_7.SetFlipped(True)
link_7.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.76653426799513,0.0961555737558022,-1.67212535415381)
dA = chrono.ChVectorD(-1.97758476261356e-14,-1,-1.94731383795776e-14)
cB = chrono.ChVectorD(1.76653426799513,0.0282055737557866,-1.67212535415382)
dB = chrono.ChVectorD(2.02060590481778e-14,1,1.95555377446865e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.76653426799513,0.0961555737558022,-1.67212535415381)
cB = chrono.ChVectorD(1.76653426799513,0.0282055737557866,-1.67212535415382)
dA = chrono.ChVectorD(-1.97758476261356e-14,-1,-1.94731383795776e-14)
dB = chrono.ChVectorD(2.02060590481778e-14,1,1.95555377446865e-14)
link_9.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.7247490938121,0.096155573755803,-1.66851025822227)
dA = chrono.ChVectorD(-1.97758476261356e-14,-1,-1.94731383795776e-14)
cB = chrono.ChVectorD(1.76653426799513,0.0952055737557866,-1.67212535415382)
dB = chrono.ChVectorD(2.02060590481778e-14,1,1.95555377446865e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.76653426799513,0.0282055737557717,-1.67212535415381)
dA = chrono.ChVectorD(1.73472347597681e-15,1,-1.80324505327789e-15)
cB = chrono.ChVectorD(1.76653426799513,0.0282055737557866,-1.67212535415382)
dB = chrono.ChVectorD(2.02060590481778e-14,1,1.95555377446865e-14)
link_11.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.76653426799513,0.0282055737557717,-1.67212535415381)
cB = chrono.ChVectorD(1.76653426799513,0.0282055737557866,-1.67212535415382)
dA = chrono.ChVectorD(1.73472347597681e-15,1,-1.80324505327789e-15)
dB = chrono.ChVectorD(2.02060590481778e-14,1,1.95555377446865e-14)
link_12.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: robo_leg_link-1/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.76407758543287,0.0617055737557882,-1.75083219763511)
dA = chrono.ChVectorD(-0.538452051358839,-5.60662627435704e-15,0.842656150744453)
cB = chrono.ChVectorD(1.77430817440871,0.061705573755214,-1.76684266449896)
dB = chrono.ChVectorD(-0.538452051358839,2.52575738102223e-15,0.842656150744453)
link_13.Initialize(body_6,body_2,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.76407758543287,0.0617055737557882,-1.75083219763511)
cB = chrono.ChVectorD(1.77430817440871,0.061705573755214,-1.76684266449896)
dA = chrono.ChVectorD(-0.538452051358839,-5.60662627435704e-15,0.842656150744453)
dB = chrono.ChVectorD(-0.538452051358839,2.52575738102223e-15,0.842656150744453)
link_14.Initialize(body_6,body_2,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-1/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.83898971723405,0.0617055737557858,-1.70296381026931)
dA = chrono.ChVectorD(-0.538452051358839,-5.60662627435704e-15,0.842656150744453)
cB = chrono.ChVectorD(1.84922030620985,0.0617055737556761,-1.71897427713361)
dB = chrono.ChVectorD(-0.538452051358839,2.50494069931051e-15,0.842656150744453)
link_15.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.83898971723405,0.0617055737557858,-1.70296381026931)
cB = chrono.ChVectorD(1.84922030620985,0.0617055737556761,-1.71897427713361)
dA = chrono.ChVectorD(-0.538452051358839,-5.60662627435704e-15,0.842656150744453)
dB = chrono.ChVectorD(-0.538452051358839,2.50494069931051e-15,0.842656150744453)
link_16.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: robo_leg_link-1/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.79003536160262,0.0952055737557847,-1.5989588413269)
cB = chrono.ChVectorD(1.64063242607565,0.0952055737552143,-1.69442639003252)
dA = chrono.ChVectorD(-0.538452051358839,-4.90579799006241e-15,0.842656150744453)
dB = chrono.ChVectorD(-0.538452051358839,2.52575738102223e-15,0.842656150744453)
link_17.Initialize(body_6,body_2,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.79003536160262,0.0952055737557847,-1.5989588413269)
dA = chrono.ChVectorD(-0.538452051358839,-4.90579799006241e-15,0.842656150744453)
cB = chrono.ChVectorD(1.64063242607565,0.0952055737552143,-1.69442639003252)
dB = chrono.ChVectorD(-0.538452051358839,2.52575738102223e-15,0.842656150744453)
link_18.Initialize(body_6,body_2,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-1/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.79003536160262,0.0952055737557847,-1.5989588413269)
cB = chrono.ChVectorD(1.83966780888145,0.0282055737556764,-1.56724401550202)
dA = chrono.ChVectorD(-0.538452051358839,-4.90579799006241e-15,0.842656150744453)
dB = chrono.ChVectorD(-0.538452051358839,2.50494069931051e-15,0.842656150744453)
link_19.Initialize(body_6,body_3,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.79003536160262,0.0952055737557847,-1.5989588413269)
dA = chrono.ChVectorD(-0.538452051358839,-4.90579799006241e-15,0.842656150744453)
cB = chrono.ChVectorD(1.83966780888145,0.0282055737556764,-1.56724401550202)
dB = chrono.ChVectorD(-0.538452051358839,2.50494069931051e-15,0.842656150744453)
link_20.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-1/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.84280366263579,0.0952055737557853,-1.68153914409986)
cB = chrono.ChVectorD(1.89243610991462,0.0282055737556762,-1.64982431827498)
dA = chrono.ChVectorD(0.538452051358839,5.60662627435704e-15,-0.842656150744453)
dB = chrono.ChVectorD(0.538452051358839,-1.74860126378462e-15,-0.842656150744453)
link_21.Initialize(body_6,body_3,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.84280366263579,0.0952055737557853,-1.68153914409986)
dA = chrono.ChVectorD(0.538452051358839,5.60662627435704e-15,-0.842656150744453)
cB = chrono.ChVectorD(1.89243610991462,0.0282055737556762,-1.64982431827498)
dB = chrono.ChVectorD(0.538452051358839,-1.74860126378462e-15,-0.842656150744453)
link_22.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: robo_leg_link-1/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.84280366263579,0.0952055737557853,-1.68153914409986)
cB = chrono.ChVectorD(1.69340072710882,0.0952055737552141,-1.77700669280548)
dA = chrono.ChVectorD(0.538452051358839,5.60662627435704e-15,-0.842656150744453)
dB = chrono.ChVectorD(0.538452051358839,-3.27515792264421e-15,-0.842656150744453)
link_23.Initialize(body_6,body_2,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.84280366263579,0.0952055737557853,-1.68153914409986)
dA = chrono.ChVectorD(0.538452051358839,5.60662627435704e-15,-0.842656150744453)
cB = chrono.ChVectorD(1.69340072710882,0.0952055737552141,-1.77700669280548)
dB = chrono.ChVectorD(0.538452051358839,-3.27515792264421e-15,-0.842656150744453)
link_24.Initialize(body_6,body_2,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: robo_leg_link-3/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.5331716047521,0.0342055760792813,-1.85526805382088)
dA = chrono.ChVectorD(-0.81361235046644,-4.40272818202914e-15,-0.581407725411759)
cB = chrono.ChVectorD(1.52666270594836,0.0342055760792364,-1.85991931562417)
dB = chrono.ChVectorD(0.813612350466434,7.7715611723761e-15,0.581407725411767)
link_1.SetFlipped(True)
link_1.Initialize(body_11,body_10,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.5331716047521,0.0342055760792813,-1.85526805382088)
cB = chrono.ChVectorD(1.52666270594836,0.0342055760792364,-1.85991931562417)
dA = chrono.ChVectorD(-0.81361235046644,-4.40272818202914e-15,-0.581407725411759)
dB = chrono.ChVectorD(0.813612350466434,7.7715611723761e-15,0.581407725411767)
link_2.Initialize(body_11,body_10,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.48865996972452,0.028205576079237,-1.8544008207659)
cB = chrono.ChVectorD(1.5285366185935,0.0282055760792356,-1.86103838997309)
dA = chrono.ChVectorD(1.12548859121375e-14,1,-2.27179386413923e-14)
dB = chrono.ChVectorD(-2.80886425230165e-14,-1,1.73472347597681e-16)
link_3.Initialize(body_10,body_7,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.48865996972452,0.028205576079237,-1.8544008207659)
dA = chrono.ChVectorD(1.12548859121375e-14,1,-2.27179386413923e-14)
cB = chrono.ChVectorD(1.5285366185935,0.0282055760792356,-1.86103838997309)
dB = chrono.ChVectorD(-2.80886425230165e-14,-1,1.73472347597681e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.50268086820672,0.0805155760792567,-1.85139833599121)
dA = chrono.ChVectorD(-0.581407725411758,2.46330733588707e-14,0.81361235046644)
cB = chrono.ChVectorD(1.52977889787033,0.0384055760792355,-1.89360668746432)
dB = chrono.ChVectorD(0.581407725411759,-1.65006897034914e-14,-0.81361235046644)
link_5.SetFlipped(True)
link_5.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: robo_leg_link-3/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.52666270594837,0.0342055760792812,-1.85991931562417)
cB = chrono.ChVectorD(1.52549930908981,0.0322045760792365,-1.85829127731089)
dA = chrono.ChVectorD(-0.81361235046644,-4.40272818202914e-15,-0.581407725411759)
dB = chrono.ChVectorD(0.813612350466434,7.7715611723761e-15,0.581407725411767)
link_6.Initialize(body_11,body_10,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.52666270594837,0.0342055760792812,-1.85991931562417)
dA = chrono.ChVectorD(-0.81361235046644,-4.40272818202914e-15,-0.581407725411759)
cB = chrono.ChVectorD(1.52549930908981,0.0322045760792365,-1.85829127731089)
dB = chrono.ChVectorD(0.813612350466434,7.7715611723761e-15,0.581407725411767)
link_7.SetFlipped(True)
link_7.Initialize(body_11,body_10,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.5285366185935,0.096155576079267,-1.86103838997309)
dA = chrono.ChVectorD(-2.81719092498633e-14,-1,5.20417042793042e-16)
cB = chrono.ChVectorD(1.5285366185935,0.0282055760792356,-1.86103838997309)
dB = chrono.ChVectorD(2.80886425230165e-14,1,-1.73472347597681e-16)
link_8.SetFlipped(True)
link_8.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.5285366185935,0.096155576079267,-1.86103838997309)
cB = chrono.ChVectorD(1.5285366185935,0.0282055760792356,-1.86103838997309)
dA = chrono.ChVectorD(-2.81719092498633e-14,-1,5.20417042793042e-16)
dB = chrono.ChVectorD(2.80886425230165e-14,1,-1.73472347597681e-16)
link_9.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.48662022847309,0.0961555760792682,-1.85959411182339)
dA = chrono.ChVectorD(-2.81719092498633e-14,-1,5.20417042793042e-16)
cB = chrono.ChVectorD(1.5285366185935,0.0952055760792356,-1.86103838997309)
dB = chrono.ChVectorD(2.80886425230165e-14,1,-1.73472347597681e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.52853661859349,0.0282055760792364,-1.86103838997309)
dA = chrono.ChVectorD(1.12548859121375e-14,1,-2.27179386413923e-14)
cB = chrono.ChVectorD(1.5285366185935,0.0282055760792356,-1.86103838997309)
dB = chrono.ChVectorD(2.80886425230165e-14,1,-1.73472347597681e-16)
link_11.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.52853661859349,0.0282055760792364,-1.86103838997309)
cB = chrono.ChVectorD(1.5285366185935,0.0282055760792356,-1.86103838997309)
dA = chrono.ChVectorD(1.12548859121375e-14,1,-2.27179386413923e-14)
dB = chrono.ChVectorD(2.80886425230165e-14,1,-1.73472347597681e-16)
link_12.Initialize(body_10,body_7,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: robo_leg_link-3/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.53016305176703,0.0617055760792355,-1.93976676614796)
dA = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
cB = chrono.ChVectorD(1.54120979854983,0.061705576079235,-1.9552254008068)
dB = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
link_13.Initialize(body_7,body_8,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.53016305176703,0.0617055760792355,-1.93976676614796)
cB = chrono.ChVectorD(1.54120979854983,0.061705576079235,-1.9552254008068)
dA = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
dB = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
link_14.Initialize(body_7,body_8,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-3/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.60249318972349,0.0617055760792335,-1.88807961935885)
dA = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
cB = chrono.ChVectorD(1.61353993650632,0.0617055760792329,-1.90353825401772)
dB = chrono.ChVectorD(-0.581407725411759,1.65145674912992e-14,0.81361235046644)
link_15.Initialize(body_7,body_9,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.60249318972349,0.0617055760792335,-1.88807961935885)
cB = chrono.ChVectorD(1.61353993650632,0.0617055760792329,-1.90353825401772)
dA = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
dB = chrono.ChVectorD(-0.581407725411759,1.65145674912992e-14,0.81361235046644)
link_16.Initialize(body_7,body_9,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: robo_leg_link-3/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.54821349119594,0.0952055760792351,-1.78675204745586)
cB = chrono.ChVectorD(1.40396002145821,0.0952055760792369,-1.88983563717134)
dA = chrono.ChVectorD(-0.581407725411759,1.72223346694977e-14,0.81361235046644)
dB = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
link_17.Initialize(body_7,body_8,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.54821349119594,0.0952055760792351,-1.78675204745586)
dA = chrono.ChVectorD(-0.581407725411759,1.72223346694977e-14,0.81361235046644)
cB = chrono.ChVectorD(1.40396002145821,0.0952055760792369,-1.88983563717134)
dB = chrono.ChVectorD(-0.581407725411759,1.65006897034914e-14,0.81361235046644)
link_18.Initialize(body_7,body_8,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-3/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.54821349119594,0.0952055760792351,-1.78675204745586)
cB = chrono.ChVectorD(1.59613525863841,0.0282055760792354,-1.75250713242911)
dA = chrono.ChVectorD(-0.581407725411759,1.72223346694977e-14,0.81361235046644)
dB = chrono.ChVectorD(-0.581407725411759,1.65145674912992e-14,0.81361235046644)
link_19.Initialize(body_7,body_9,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.54821349119594,0.0952055760792351,-1.78675204745586)
dA = chrono.ChVectorD(-0.581407725411759,1.72223346694977e-14,0.81361235046644)
cB = chrono.ChVectorD(1.59613525863841,0.0282055760792354,-1.75250713242911)
dB = chrono.ChVectorD(-0.581407725411759,1.65145674912992e-14,0.81361235046644)
link_20.Initialize(body_7,body_9,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-3/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.60519144828629,0.0952055760792334,-1.86648605780157)
cB = chrono.ChVectorD(1.65311321572876,0.0282055760792338,-1.83224114277482)
dA = chrono.ChVectorD(0.581407725411759,-1.65006897034914e-14,-0.81361235046644)
dB = chrono.ChVectorD(0.581407725411759,-1.57512891618694e-14,-0.81361235046644)
link_21.Initialize(body_7,body_9,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.60519144828629,0.0952055760792334,-1.86648605780157)
dA = chrono.ChVectorD(0.581407725411759,-1.65006897034914e-14,-0.81361235046644)
cB = chrono.ChVectorD(1.65311321572876,0.0282055760792338,-1.83224114277482)
dB = chrono.ChVectorD(0.581407725411759,-1.57512891618694e-14,-0.81361235046644)
link_22.Initialize(body_7,body_9,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: robo_leg_link-3/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.60519144828629,0.0952055760792334,-1.86648605780157)
cB = chrono.ChVectorD(1.46093797854857,0.0952055760792353,-1.96956964751705)
dA = chrono.ChVectorD(0.581407725411759,-1.65006897034914e-14,-0.81361235046644)
dB = chrono.ChVectorD(0.581407725411759,-1.72639680329212e-14,-0.81361235046644)
link_23.Initialize(body_7,body_8,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.60519144828629,0.0952055760792334,-1.86648605780157)
dA = chrono.ChVectorD(0.581407725411759,-1.65006897034914e-14,-0.81361235046644)
cB = chrono.ChVectorD(1.46093797854857,0.0952055760792353,-1.96956964751705)
dB = chrono.ChVectorD(0.581407725411759,-1.72639680329212e-14,-0.81361235046644)
link_24.Initialize(body_7,body_8,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: robo_leg_link-2/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.65029670413096,0.0342055756533005,-1.75863496145621)
dA = chrono.ChVectorD(-0.733530114733819,-9.78384040450919e-15,-0.679656950805765)
cB = chrono.ChVectorD(1.64442846321308,0.034205575653217,-1.76407221706266)
dB = chrono.ChVectorD(0.733530114733817,9.83935155574045e-15,0.679656950805767)
link_1.SetFlipped(True)
link_1.Initialize(body_13,body_15,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.65029670413096,0.0342055756533005,-1.75863496145621)
cB = chrono.ChVectorD(1.64442846321308,0.034205575653217,-1.76407221706266)
dA = chrono.ChVectorD(-0.733530114733819,-9.78384040450919e-15,-0.679656950805765)
dB = chrono.ChVectorD(0.733530114733817,9.83935155574045e-15,0.679656950805767)
link_2.Initialize(body_13,body_15,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.60603293016652,0.0282055756532175,-1.76340529272623)
cB = chrono.ChVectorD(1.64642888243496,0.0282055756532315,-1.76494525711712)
dA = chrono.ChVectorD(1.14908083048704e-14,1,-2.14845502499728e-14)
dB = chrono.ChVectorD(-2.50494069931051e-14,-1,-1.65145674912992e-15)
link_3.Initialize(body_15,body_16,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.60603293016652,0.0282055756532175,-1.76340529272623)
dA = chrono.ChVectorD(1.14908083048704e-14,1,-2.14845502499728e-14)
cB = chrono.ChVectorD(1.64642888243496,0.0282055756532315,-1.76494525711712)
dB = chrono.ChVectorD(-2.50494069931051e-14,-1,-1.65145674912992e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.61956139228878,0.0805155756532373,-1.75865332243203)
dA = chrono.ChVectorD(-0.679656950805759,2.32452945780892e-14,0.733530114733825)
cB = chrono.ChVectorD(1.65178098132441,0.0384055756532314,-1.79709478738983)
dB = chrono.ChVectorD(0.679656950805757,-1.58206781009085e-14,-0.733530114733827)
link_5.SetFlipped(True)
link_5.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: robo_leg_link-2/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.64442846321309,0.0342055756533004,-1.76407221706266)
cB = chrono.ChVectorD(1.64306846965451,0.0322045756532171,-1.76260442330307)
dA = chrono.ChVectorD(-0.733530114733819,-9.78384040450919e-15,-0.679656950805765)
dB = chrono.ChVectorD(0.733530114733817,9.83935155574045e-15,0.679656950805767)
link_6.Initialize(body_13,body_15,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.64442846321309,0.0342055756533004,-1.76407221706266)
dA = chrono.ChVectorD(-0.733530114733819,-9.78384040450919e-15,-0.679656950805765)
cB = chrono.ChVectorD(1.64306846965451,0.0322045756532171,-1.76260442330307)
dB = chrono.ChVectorD(0.733530114733817,9.83935155574045e-15,0.679656950805767)
link_7.SetFlipped(True)
link_7.Initialize(body_13,body_15,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.64642888243497,0.0961555756532476,-1.76494525711711)
dA = chrono.ChVectorD(-2.54518628395317e-14,-1,-2.65933108867245e-15)
cB = chrono.ChVectorD(1.64642888243496,0.0282055756532315,-1.76494525711712)
dB = chrono.ChVectorD(2.50494069931051e-14,1,1.65145674912992e-15)
link_8.SetFlipped(True)
link_8.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.64642888243497,0.0961555756532476,-1.76494525711711)
cB = chrono.ChVectorD(1.64642888243496,0.0282055756532315,-1.76494525711712)
dA = chrono.ChVectorD(-2.54518628395317e-14,-1,-2.65933108867245e-15)
dB = chrono.ChVectorD(2.50494069931051e-14,1,1.65145674912992e-15)
link_9.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.60466651108167,0.0961555756532487,-1.76881488746754)
dA = chrono.ChVectorD(-2.54518628395317e-14,-1,-2.65933108867245e-15)
cB = chrono.ChVectorD(1.64642888243496,0.0952055756532315,-1.76494525711712)
dB = chrono.ChVectorD(2.50494069931051e-14,1,1.65145674912992e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.64642888243496,0.0282055756532171,-1.76494525711711)
dA = chrono.ChVectorD(1.14908083048704e-14,1,-2.14845502499728e-14)
cB = chrono.ChVectorD(1.64642888243496,0.0282055756532315,-1.76494525711712)
dB = chrono.ChVectorD(2.50494069931051e-14,1,1.65145674912992e-15)
link_11.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.64642888243496,0.0282055756532171,-1.76494525711711)
cB = chrono.ChVectorD(1.64642888243496,0.0282055756532315,-1.76494525711712)
dA = chrono.ChVectorD(1.14908083048704e-14,1,-2.14845502499728e-14)
dB = chrono.ChVectorD(2.50494069931051e-14,1,1.65145674912992e-15)
link_12.Initialize(body_15,body_16,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: robo_leg_link-2/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.65800117063741,0.0617055756532313,-1.84283546603813)
dA = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
cB = chrono.ChVectorD(1.67091465270255,0.0617055756532216,-1.85677253821792)
dB = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
link_13.Initialize(body_16,body_14,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.65800117063741,0.0617055756532313,-1.84283546603813)
cB = chrono.ChVectorD(1.67091465270255,0.0617055756532216,-1.85677253821792)
dA = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
dB = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
link_14.Initialize(body_16,body_14,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-2/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.72321199783725,0.0617055756532296,-1.7824139631115)
dA = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
cB = chrono.ChVectorD(1.73612547990259,0.0617055756532218,-1.79635103529151)
dB = chrono.ChVectorD(-0.679656950805757,1.58345558887163e-14,0.733530114733827)
link_15.Initialize(body_16,body_12,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.72321199783725,0.0617055756532296,-1.7824139631115)
cB = chrono.ChVectorD(1.73612547990259,0.0617055756532218,-1.79635103529151)
dA = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
dB = chrono.ChVectorD(-0.679656950805757,1.58345558887163e-14,0.733530114733827)
link_16.Initialize(body_16,body_12,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: robo_leg_link-2/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.65655067463772,0.0952055756532311,-1.68876659000746)
cB = chrono.ChVectorD(1.52649578529524,0.0952055756532232,-1.80926976738517)
dA = chrono.ChVectorD(-0.679656950805757,1.6528445279107e-14,0.733530114733827)
dB = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
link_17.Initialize(body_16,body_14,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.65655067463772,0.0952055756532311,-1.68876659000746)
dA = chrono.ChVectorD(-0.679656950805757,1.6528445279107e-14,0.733530114733827)
cB = chrono.ChVectorD(1.52649578529524,0.0952055756532232,-1.80926976738517)
dB = chrono.ChVectorD(-0.679656950805757,1.58206781009085e-14,0.733530114733827)
link_18.Initialize(body_16,body_14,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-2/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.65655067463772,0.0952055756532311,-1.68876659000746)
cB = chrono.ChVectorD(1.69975559839557,0.0282055756532244,-1.64873479560507)
dA = chrono.ChVectorD(-0.679656950805757,1.6528445279107e-14,0.733530114733827)
dB = chrono.ChVectorD(-0.679656950805757,1.58345558887163e-14,0.733530114733827)
link_19.Initialize(body_16,body_12,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.65655067463772,0.0952055756532311,-1.68876659000746)
dA = chrono.ChVectorD(-0.679656950805757,1.6528445279107e-14,0.733530114733827)
cB = chrono.ChVectorD(1.69975559839557,0.0282055756532244,-1.64873479560507)
dB = chrono.ChVectorD(-0.679656950805757,1.58345558887163e-14,0.733530114733827)
link_20.Initialize(body_16,body_12,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-2/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.72315705581668,0.0952055756532295,-1.76065254125137)
cB = chrono.ChVectorD(1.76636197957454,0.0282055756532229,-1.72062074684899)
dA = chrono.ChVectorD(0.679656950805757,-1.58206781009085e-14,-0.733530114733827)
dB = chrono.ChVectorD(0.679656950805757,-1.50712775592865e-14,-0.733530114733827)
link_21.Initialize(body_16,body_12,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.72315705581668,0.0952055756532295,-1.76065254125137)
dA = chrono.ChVectorD(0.679656950805757,-1.58206781009085e-14,-0.733530114733827)
cB = chrono.ChVectorD(1.76636197957454,0.0282055756532229,-1.72062074684899)
dB = chrono.ChVectorD(0.679656950805757,-1.50712775592865e-14,-0.733530114733827)
link_22.Initialize(body_16,body_12,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: robo_leg_link-2/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.72315705581668,0.0952055756532295,-1.76065254125137)
cB = chrono.ChVectorD(1.5931021664742,0.0952055756532216,-1.88115571862908)
dA = chrono.ChVectorD(0.679656950805757,-1.58206781009085e-14,-0.733530114733827)
dB = chrono.ChVectorD(0.679656950805757,-1.65839564303383e-14,-0.733530114733827)
link_23.Initialize(body_16,body_14,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.72315705581668,0.0952055756532295,-1.76065254125137)
dA = chrono.ChVectorD(0.679656950805757,-1.58206781009085e-14,-0.733530114733827)
cB = chrono.ChVectorD(1.5931021664742,0.0952055756532216,-1.88115571862908)
dB = chrono.ChVectorD(0.679656950805757,-1.65839564303383e-14,-0.733530114733827)
link_24.Initialize(body_16,body_14,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_21 , SW name: robo_leg_link-4/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.45685573956354,0.034205575679288,-1.97376274193494)
dA = chrono.ChVectorD(-0.111417776904548,-1.49888781941776e-14,-0.993773655813862)
cB = chrono.ChVectorD(1.45596439734904,0.0342055756532196,-1.98171293118152)
dB = chrono.ChVectorD(0.111417776904539,1.89639970393785e-14,0.993773655813863)
link_1.SetFlipped(True)
link_1.Initialize(body_21,body_19,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.45685573956354,0.034205575679288,-1.97376274193494)
cB = chrono.ChVectorD(1.45596439734904,0.0342055756532196,-1.98171293118152)
dA = chrono.ChVectorD(-0.111417776904548,-1.49888781941776e-14,-0.993773655813862)
dB = chrono.ChVectorD(0.111417776904539,1.89639970393785e-14,0.993773655813863)
link_2.Initialize(body_21,body_19,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.42645742669343,0.0282055756532196,-2.00628935135877)
cB = chrono.ChVectorD(1.45804932379766,0.0282055756532568,-1.98106720856015)
dA = chrono.ChVectorD(1.44606548957427e-14,1,-1.69898817237168e-14)
dB = chrono.ChVectorD(-9.17321774096536e-15,-1,-1.04985464766116e-14)
link_3.Initialize(body_19,body_20,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.42645742669343,0.0282055756532196,-2.00628935135877)
dA = chrono.ChVectorD(1.44606548957427e-14,1,-1.69898817237168e-14)
cB = chrono.ChVectorD(1.45804932379766,0.0282055756532568,-1.98106720856015)
dB = chrono.ChVectorD(-9.17321774096536e-15,-1,-1.04985464766116e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.4335963857015,0.0805155756532396,-1.99385408805832)
dA = chrono.ChVectorD(-0.993773655813862,1.59872115546023e-14,0.11141777690455)
cB = chrono.ChVectorD(1.48310294793395,0.0384055756532567,-2.00191314765908)
dB = chrono.ChVectorD(0.993773655813862,-7.92421683826205e-15,-0.111417776904549)
link_5.SetFlipped(True)
link_5.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_21 , SW name: robo_leg_link-4/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.4559643973483,0.0342055756792879,-1.98171293118145)
cB = chrono.ChVectorD(1.45397585626376,0.0322045756532196,-1.98148998420994)
dA = chrono.ChVectorD(-0.111417776904548,-1.49888781941776e-14,-0.993773655813862)
dB = chrono.ChVectorD(0.111417776904539,1.89639970393785e-14,0.993773655813863)
link_6.Initialize(body_21,body_19,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.4559643973483,0.0342055756792879,-1.98171293118145)
dA = chrono.ChVectorD(-0.111417776904548,-1.49888781941776e-14,-0.993773655813862)
cB = chrono.ChVectorD(1.45397585626376,0.0322045756532196,-1.98148998420994)
dB = chrono.ChVectorD(0.111417776904539,1.89639970393785e-14,0.993773655813863)
link_7.SetFlipped(True)
link_7.Initialize(body_21,body_19,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.45804932379768,0.0961555756532502,-1.98106720856015)
dA = chrono.ChVectorD(-9.27036225562006e-15,-1,-1.04100755793368e-14)
cB = chrono.ChVectorD(1.45804932379766,0.0282055756532568,-1.98106720856015)
dB = chrono.ChVectorD(9.17321774096536e-15,1,1.04985464766116e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.45804932379768,0.0961555756532502,-1.98106720856015)
cB = chrono.ChVectorD(1.45804932379766,0.0282055756532568,-1.98106720856015)
dA = chrono.ChVectorD(-9.27036225562006e-15,-1,-1.04100755793368e-14)
dB = chrono.ChVectorD(9.17321774096536e-15,1,1.04985464766116e-14)
link_9.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.42895658538698,0.0961555756532508,-2.01127784032889)
dA = chrono.ChVectorD(-9.27036225562006e-15,-1,-1.04100755793368e-14)
cB = chrono.ChVectorD(1.45804932379766,0.0952055756532568,-1.98106720856015)
dB = chrono.ChVectorD(9.17321774096536e-15,1,1.04985464766116e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.45804932379768,0.0282055756532196,-1.98106720856015)
dA = chrono.ChVectorD(1.44606548957427e-14,1,-1.69898817237168e-14)
cB = chrono.ChVectorD(1.45804932379766,0.0282055756532568,-1.98106720856015)
dB = chrono.ChVectorD(9.17321774096536e-15,1,1.04985464766116e-14)
link_11.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.45804932379768,0.0282055756532196,-1.98106720856015)
cB = chrono.ChVectorD(1.45804932379766,0.0282055756532568,-1.98106720856015)
dA = chrono.ChVectorD(1.44606548957427e-14,1,-1.69898817237168e-14)
dB = chrono.ChVectorD(9.17321774096536e-15,1,1.04985464766116e-14)
link_12.Initialize(body_19,body_20,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: robo_leg_link-4/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.51769209124216,0.0617055756532568,-2.03248260305987)
dA = chrono.ChVectorD(-0.993773655813862,7.92421683826205e-15,0.111417776904549)
cB = chrono.ChVectorD(1.53657379070302,0.0617055756532199,-2.03459954082106)
dB = chrono.ChVectorD(-0.993773655813862,7.88258347483861e-15,0.111417776904549)
link_13.Initialize(body_20,body_18,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.51769209124216,0.0617055756532568,-2.03248260305987)
cB = chrono.ChVectorD(1.53657379070302,0.0617055756532199,-2.03459954082106)
dA = chrono.ChVectorD(-0.993773655813862,7.92421683826205e-15,0.111417776904549)
dB = chrono.ChVectorD(-0.993773655813862,7.88258347483861e-15,0.111417776904549)
link_14.Initialize(body_20,body_18,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-4/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.52759713160897,0.0617055756532558,-1.94413612505802)
dA = chrono.ChVectorD(-0.993773655813862,7.92421683826205e-15,0.111417776904549)
cB = chrono.ChVectorD(1.5464788310693,0.0617055756532208,-1.94625306281915)
dB = chrono.ChVectorD(-0.993773655813862,7.93809462606987e-15,0.111417776904549)
link_15.Initialize(body_20,body_17,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.52759713160897,0.0617055756532558,-1.94413612505802)
cB = chrono.ChVectorD(1.5464788310693,0.0617055756532208,-1.94625306281915)
dA = chrono.ChVectorD(-0.993773655813862,7.92421683826205e-15,0.111417776904549)
dB = chrono.ChVectorD(-0.993773655813862,7.93809462606987e-15,0.111417776904549)
link_16.Initialize(body_20,body_17,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: robo_leg_link-4/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.41595034705553,0.0952055756532565,-1.91677633706765)
cB = chrono.ChVectorD(1.39619597521076,0.0952055756532198,-2.09297240624345)
dA = chrono.ChVectorD(-0.993773655813862,8.64586180426841e-15,0.111417776904549)
dB = chrono.ChVectorD(-0.993773655813862,7.88258347483861e-15,0.111417776904549)
link_17.Initialize(body_20,body_18,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.41595034705553,0.0952055756532565,-1.91677633706765)
dA = chrono.ChVectorD(-0.993773655813862,8.64586180426841e-15,0.111417776904549)
cB = chrono.ChVectorD(1.39619597521076,0.0952055756532198,-2.09297240624345)
dB = chrono.ChVectorD(-0.993773655813862,7.88258347483861e-15,0.111417776904549)
link_18.Initialize(body_20,body_18,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-4/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.41595034705553,0.0952055756532565,-1.91677633706765)
cB = chrono.ChVectorD(1.42251285411508,0.028205575653223,-1.85824306874016)
dA = chrono.ChVectorD(-0.993773655813862,8.64586180426841e-15,0.111417776904549)
dB = chrono.ChVectorD(-0.993773655813862,7.93809462606987e-15,0.111417776904549)
link_19.Initialize(body_20,body_17,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.41595034705553,0.0952055756532565,-1.91677633706765)
dA = chrono.ChVectorD(-0.993773655813862,8.64586180426841e-15,0.111417776904549)
cB = chrono.ChVectorD(1.42251285411508,0.028205575653223,-1.85824306874016)
dB = chrono.ChVectorD(-0.993773655813862,7.93809462606987e-15,0.111417776904549)
link_20.Initialize(body_20,body_17,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-4/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.51334016532529,0.0952055756532557,-1.92769527920429)
cB = chrono.ChVectorD(1.51990267238484,0.0282055756532222,-1.8691620108768)
dA = chrono.ChVectorD(0.993773655813862,-7.92421683826205e-15,-0.111417776904549)
dB = chrono.ChVectorD(0.993773655813862,-7.17481629664007e-15,-0.111417776904549)
link_21.Initialize(body_20,body_17,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.51334016532529,0.0952055756532557,-1.92769527920429)
dA = chrono.ChVectorD(0.993773655813862,-7.92421683826205e-15,-0.111417776904549)
cB = chrono.ChVectorD(1.51990267238484,0.0282055756532222,-1.8691620108768)
dB = chrono.ChVectorD(0.993773655813862,-7.17481629664007e-15,-0.111417776904549)
link_22.Initialize(body_20,body_17,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_20 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: robo_leg_link-4/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.51334016532529,0.0952055756532557,-1.92769527920429)
cB = chrono.ChVectorD(1.49358579348052,0.095205575653219,-2.1038913483801)
dA = chrono.ChVectorD(0.993773655813862,-7.92421683826205e-15,-0.111417776904549)
dB = chrono.ChVectorD(0.993773655813862,-8.64586180426841e-15,-0.111417776904549)
link_23.Initialize(body_20,body_18,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.51334016532529,0.0952055756532557,-1.92769527920429)
dA = chrono.ChVectorD(0.993773655813862,-7.92421683826205e-15,-0.111417776904549)
cB = chrono.ChVectorD(1.49358579348052,0.095205575653219,-2.1038913483801)
dB = chrono.ChVectorD(0.993773655813862,-8.64586180426841e-15,-0.111417776904549)
link_24.Initialize(body_20,body_18,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_25 , SW name: robo_leg_link-5/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.47642242111997,0.0342055756535343,-2.12046056843724)
dA = chrono.ChVectorD(0.415243753657806,-1.70852915148956e-14,-0.909710187393862)
cB = chrono.ChVectorD(1.47974437114924,0.0342055756532134,-2.12773824993639)
dB = chrono.ChVectorD(-0.415243753657818,2.01418742795667e-14,0.909710187393857)
link_1.SetFlipped(True)
link_1.Initialize(body_25,body_23,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.47642242111997,0.0342055756535343,-2.12046056843724)
cB = chrono.ChVectorD(1.47974437114924,0.0342055756532134,-2.12773824993639)
dA = chrono.ChVectorD(0.415243753657806,-1.70852915148956e-14,-0.909710187393862)
dB = chrono.ChVectorD(-0.415243753657818,2.01418742795667e-14,0.909710187393857)
link_2.Initialize(body_25,body_23,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.46706654547499,0.0282055756532132,-2.16398648413025)
cB = chrono.ChVectorD(1.48120086848332,0.028205575653262,-2.12611267572529)
dA = chrono.ChVectorD(1.28369537222284e-14,1,-1.22332699525884e-14)
dB = chrono.ChVectorD(5.25968157916168e-15,-1,-8.06993361024411e-15)
link_3.Initialize(body_23,body_24,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.46706654547499,0.0282055756532132,-2.16398648413025)
dA = chrono.ChVectorD(1.28369537222284e-14,1,-1.22332699525884e-14)
cB = chrono.ChVectorD(1.48120086848332,0.028205575653262,-2.12611267572529)
dB = chrono.ChVectorD(5.25968157916168e-15,-1,-8.06993361024411e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.466798278518,0.0805155756532332,-2.14965021746455)
dA = chrono.ChVectorD(-0.909710187393861,6.23112672570869e-15,-0.415243753657808)
cB = chrono.ChVectorD(1.51340652847874,0.0384055756532622,-2.13111594700893)
dB = chrono.ChVectorD(0.909710187393862,1.47104550762833e-15,0.415243753657806)
link_5.SetFlipped(True)
link_5.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_25 , SW name: robo_leg_link-5/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.47974437114923,0.0342055756535341,-2.12773824993639)
cB = chrono.ChVectorD(1.47792404106427,0.0322045756532135,-2.12856915268746)
dA = chrono.ChVectorD(0.415243753657806,-1.70852915148956e-14,-0.909710187393862)
dB = chrono.ChVectorD(-0.415243753657818,2.01418742795667e-14,0.909710187393857)
link_6.Initialize(body_25,body_23,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.47974437114923,0.0342055756535341,-2.12773824993639)
dA = chrono.ChVectorD(0.415243753657806,-1.70852915148956e-14,-0.909710187393862)
cB = chrono.ChVectorD(1.47792404106427,0.0322045756532135,-2.12856915268746)
dB = chrono.ChVectorD(-0.415243753657818,2.01418742795667e-14,0.909710187393857)
link_7.SetFlipped(True)
link_7.Initialize(body_25,body_23,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.48120086848333,0.096155575653244,-2.12611267572529)
dA = chrono.ChVectorD(5.71764857681956e-15,-1,-8.60422844084496e-15)
cB = chrono.ChVectorD(1.48120086848332,0.028205575653262,-2.12611267572529)
dB = chrono.ChVectorD(-5.25968157916168e-15,1,8.06993361024411e-15)
link_8.SetFlipped(True)
link_8.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.48120086848333,0.096155575653244,-2.12611267572529)
cB = chrono.ChVectorD(1.48120086848332,0.028205575653262,-2.12611267572529)
dA = chrono.ChVectorD(5.71764857681956e-15,-1,-8.60422844084496e-15)
dB = chrono.ChVectorD(-5.25968157916168e-15,1,8.06993361024411e-15)
link_9.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.47177443911906,0.0961555756532444,-2.16698090457779)
dA = chrono.ChVectorD(5.71764857681956e-15,-1,-8.60422844084496e-15)
cB = chrono.ChVectorD(1.48120086848332,0.095205575653262,-2.12611267572529)
dB = chrono.ChVectorD(-5.25968157916168e-15,1,8.06993361024411e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.48120086848333,0.0282055756532135,-2.12611267572529)
dA = chrono.ChVectorD(1.28369537222284e-14,1,-1.22332699525884e-14)
cB = chrono.ChVectorD(1.48120086848332,0.028205575653262,-2.12611267572529)
dB = chrono.ChVectorD(-5.25968157916168e-15,1,8.06993361024411e-15)
link_11.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.48120086848333,0.0282055756532135,-2.12611267572529)
cB = chrono.ChVectorD(1.48120086848332,0.028205575653262,-2.12611267572529)
dA = chrono.ChVectorD(1.28369537222284e-14,1,-1.22332699525884e-14)
dB = chrono.ChVectorD(-5.25968157916168e-15,1,8.06993361024411e-15)
link_12.Initialize(body_23,body_24,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: robo_leg_link-5/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.55878961551401,0.0617055756532625,-2.13955844956719)
dA = chrono.ChVectorD(-0.909710187393862,-1.47104550762833e-15,-0.415243753657806)
cB = chrono.ChVectorD(1.5760741084789,0.0617055817647141,-2.13166882362966)
dB = chrono.ChVectorD(-0.909710187393862,-2.09554595897998e-15,-0.415243753657806)
link_13.Initialize(body_24,body_26,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.55878961551401,0.0617055756532625,-2.13955844956719)
cB = chrono.ChVectorD(1.5760741084789,0.0617055817647141,-2.13166882362966)
dA = chrono.ChVectorD(-0.909710187393862,-1.47104550762833e-15,-0.415243753657806)
dB = chrono.ChVectorD(-0.909710187393862,-2.09554595897998e-15,-0.415243753657806)
link_14.Initialize(body_24,body_26,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-5/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.52187444581383,0.0617055756532617,-2.05868521390788)
dA = chrono.ChVectorD(-0.909710187393862,-1.47104550762833e-15,-0.415243753657806)
cB = chrono.ChVectorD(1.53915893917895,0.0617055807421757,-2.05079558904898)
dB = chrono.ChVectorD(-0.909710187393862,-1.48492329543615e-15,-0.415243753657806)
link_15.Initialize(body_24,body_22,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.52187444581383,0.0617055756532617,-2.05868521390788)
cB = chrono.ChVectorD(1.53915893917895,0.0617055807421757,-2.05079558904898)
dA = chrono.ChVectorD(-0.909710187393862,-1.47104550762833e-15,-0.415243753657806)
dB = chrono.ChVectorD(-0.909710187393862,-1.48492329543615e-15,-0.415243753657806)
link_16.Initialize(body_24,body_22,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: robo_leg_link-5/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.41204263908447,0.0952055756532614,-2.09260477656081)
cB = chrono.ChVectorD(1.48566535601242,0.0952055817647126,-2.2538963981677)
dA = chrono.ChVectorD(-0.909710187393862,-7.63278329429795e-16,-0.415243753657806)
dB = chrono.ChVectorD(-0.909710187393862,-2.09554595897998e-15,-0.415243753657806)
link_17.Initialize(body_24,body_26,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.41204263908447,0.0952055756532614,-2.09260477656081)
dA = chrono.ChVectorD(-0.909710187393862,-7.63278329429795e-16,-0.415243753657806)
cB = chrono.ChVectorD(1.48566535601242,0.0952055817647126,-2.2538963981677)
dB = chrono.ChVectorD(-0.909710187393862,-2.09554595897998e-15,-0.415243753657806)
link_18.Initialize(body_24,body_26,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-5/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.41204263908447,0.0952055756532614,-2.09260477656081)
cB = chrono.ChVectorD(1.38758478179867,0.0282055807421767,-2.03902285298391)
dA = chrono.ChVectorD(-0.909710187393862,-7.63278329429795e-16,-0.415243753657806)
dB = chrono.ChVectorD(-0.909710187393862,-1.48492329543615e-15,-0.415243753657806)
link_19.Initialize(body_24,body_22,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.41204263908447,0.0952055756532614,-2.09260477656081)
dA = chrono.ChVectorD(-0.909710187393862,-7.63278329429795e-16,-0.415243753657806)
cB = chrono.ChVectorD(1.38758478179867,0.0282055807421767,-2.03902285298391)
dB = chrono.ChVectorD(-0.909710187393862,-1.48492329543615e-15,-0.415243753657806)
link_20.Initialize(body_24,body_22,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-5/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.50119423744907,0.0952055756532615,-2.05191088870234)
cB = chrono.ChVectorD(1.47673638016327,0.0282055807421769,-1.99832896512545)
dA = chrono.ChVectorD(0.909710187393862,1.47104550762833e-15,0.415243753657806)
dB = chrono.ChVectorD(0.909710187393862,2.23432383705813e-15,0.415243753657806)
link_21.Initialize(body_24,body_22,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.50119423744907,0.0952055756532615,-2.05191088870234)
dA = chrono.ChVectorD(0.909710187393862,1.47104550762833e-15,0.415243753657806)
cB = chrono.ChVectorD(1.47673638016327,0.0282055807421769,-1.99832896512545)
dB = chrono.ChVectorD(0.909710187393862,2.23432383705813e-15,0.415243753657806)
link_22.Initialize(body_24,body_22,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_26 , SW name: robo_leg_link-5/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.50119423744907,0.0952055756532615,-2.05191088870234)
cB = chrono.ChVectorD(1.57481695437702,0.0952055817647128,-2.21320251030924)
dA = chrono.ChVectorD(0.909710187393862,1.47104550762833e-15,0.415243753657806)
dB = chrono.ChVectorD(0.909710187393862,1.346145417358e-15,0.415243753657806)
link_23.Initialize(body_24,body_26,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.50119423744907,0.0952055756532615,-2.05191088870234)
dA = chrono.ChVectorD(0.909710187393862,1.47104550762833e-15,0.415243753657806)
cB = chrono.ChVectorD(1.57481695437702,0.0952055817647128,-2.21320251030924)
dB = chrono.ChVectorD(0.909710187393862,1.346145417358e-15,0.415243753657806)
link_24.Initialize(body_24,body_26,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-8/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.84483118957432,0.0342055756533141,-2.06449841167006)
dA = chrono.ChVectorD(0.913301131235134,7.35522753814166e-16,0.407284966190289)
cB = chrono.ChVectorD(1.85213759862421,0.0342055756532361,-2.0612401319405)
dB = chrono.ChVectorD(-0.913301131235129,1.45716771982052e-15,-0.407284966190299)
link_1.SetFlipped(True)
link_1.Initialize(body_28,body_29,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.84483118957432,0.0342055756533141,-2.06449841167006)
cB = chrono.ChVectorD(1.85213759862421,0.0342055756532361,-2.0612401319405)
dA = chrono.ChVectorD(0.913301131235134,7.35522753814166e-16,0.407284966190289)
dB = chrono.ChVectorD(-0.913301131235129,1.45716771982052e-15,-0.407284966190299)
link_2.Initialize(body_28,body_29,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.88827375709804,0.028205575653236,-2.07423396881629)
cB = chrono.ChVectorD(1.85052480350311,0.0282055756532246,-2.05976949673849)
dA = chrono.ChVectorD(1.23512311489549e-15,1,-7.95891130778159e-15)
dB = chrono.ChVectorD(1.72084568816899e-14,-1,2.47649123430449e-14)
link_3.Initialize(body_29,body_27,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.88827375709804,0.028205575653236,-2.07423396881629)
dA = chrono.ChVectorD(1.23512311489549e-15,1,-7.95891130778159e-15)
cB = chrono.ChVectorD(1.85052480350311,0.0282055756532246,-2.05976949673849)
dB = chrono.ChVectorD(1.72084568816899e-14,-1,2.47649123430449e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.8739356945894,0.0805155756532559,-2.07437705120128)
dA = chrono.ChVectorD(0.40728496619029,-8.09508710064577e-15,-0.913301131235134)
cB = chrono.ChVectorD(1.8558090815749,0.0384055756532255,-2.02760874947511)
dB = chrono.ChVectorD(-0.407284966190289,1.55925619638175e-14,0.913301131235134)
link_5.SetFlipped(True)
link_5.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-8/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.8521375986242,0.0342055756533141,-2.06124013194054)
cB = chrono.ChVectorD(1.85295257584156,0.0322045756532361,-2.0630676475041)
dA = chrono.ChVectorD(0.913301131235134,7.35522753814166e-16,0.407284966190289)
dB = chrono.ChVectorD(-0.913301131235129,1.45716771982052e-15,-0.407284966190299)
link_6.Initialize(body_28,body_29,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.8521375986242,0.0342055756533141,-2.06124013194054)
dA = chrono.ChVectorD(0.913301131235134,7.35522753814166e-16,0.407284966190289)
cB = chrono.ChVectorD(1.85295257584156,0.0322045756532361,-2.0630676475041)
dB = chrono.ChVectorD(-0.913301131235129,1.45716771982052e-15,-0.407284966190299)
link_7.SetFlipped(True)
link_7.Initialize(body_28,body_29,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.85052480350313,0.0961555756532668,-2.05976949673848)
dA = chrono.ChVectorD(1.974809205052e-14,-1,2.63122856836162e-14)
cB = chrono.ChVectorD(1.85052480350311,0.0282055756532246,-2.05976949673849)
dB = chrono.ChVectorD(-1.72084568816899e-14,1,-2.47649123430449e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.85052480350313,0.0961555756532668,-2.05976949673848)
cB = chrono.ChVectorD(1.85052480350311,0.0282055756532246,-2.05976949673849)
dA = chrono.ChVectorD(1.974809205052e-14,-1,2.63122856836162e-14)
dB = chrono.ChVectorD(-1.72084568816899e-14,1,-2.47649123430449e-14)
link_9.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.89130916946741,0.0961555756532674,-2.0695523998348)
dA = chrono.ChVectorD(1.974809205052e-14,-1,2.63122856836162e-14)
cB = chrono.ChVectorD(1.85052480350311,0.0952055756532246,-2.05976949673849)
dB = chrono.ChVectorD(-1.72084568816899e-14,1,-2.47649123430449e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.85052480350313,0.0282055756532362,-2.05976949673848)
dA = chrono.ChVectorD(1.23512311489549e-15,1,-7.95891130778159e-15)
cB = chrono.ChVectorD(1.85052480350311,0.0282055756532246,-2.05976949673849)
dB = chrono.ChVectorD(-1.72084568816899e-14,1,-2.47649123430449e-14)
link_11.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.85052480350313,0.0282055756532362,-2.05976949673848)
cB = chrono.ChVectorD(1.85052480350311,0.0282055756532246,-2.05976949673849)
dA = chrono.ChVectorD(1.23512311489549e-15,1,-7.95891130778159e-15)
dB = chrono.ChVectorD(-1.72084568816899e-14,1,-2.47649123430449e-14)
link_12.Initialize(body_29,body_27,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_31 , SW name: robo_leg_link-8/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.86464751598414,0.0617055756532268,-1.98230110646105)
dA = chrono.ChVectorD(0.407284966190289,-1.55925619638175e-14,-0.913301131235134)
cB = chrono.ChVectorD(1.85690910162618,0.0617055756532226,-1.96494838496698)
dB = chrono.ChVectorD(0.407284966190289,-1.49030093821167e-14,-0.913301131235134)
link_13.Initialize(body_27,body_31,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.86464751598414,0.0617055756532268,-1.98230110646105)
cB = chrono.ChVectorD(1.85690910162618,0.0617055756532226,-1.96494838496698)
dA = chrono.ChVectorD(0.407284966190289,-1.55925619638175e-14,-0.913301131235134)
dB = chrono.ChVectorD(0.407284966190289,-1.49030093821167e-14,-0.913301131235134)
link_14.Initialize(body_27,body_31,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-8/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.78345504541734,0.0617055756532245,-2.01850873995536)
dA = chrono.ChVectorD(0.407284966190289,-1.55925619638175e-14,-0.913301131235134)
cB = chrono.ChVectorD(1.77571663105975,0.0617055756532215,-2.00115601846213)
dB = chrono.ChVectorD(0.407284966190289,-1.56159807307432e-14,-0.913301131235134)
link_15.Initialize(body_27,body_30,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.78345504541734,0.0617055756532245,-2.01850873995536)
cB = chrono.ChVectorD(1.77571663105975,0.0617055756532215,-2.00115601846213)
dA = chrono.ChVectorD(0.407284966190289,-1.55925619638175e-14,-0.913301131235134)
dB = chrono.ChVectorD(0.407284966190289,-1.56159807307432e-14,-0.913301131235134)
link_16.Initialize(body_27,body_30,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_31 , SW name: robo_leg_link-8/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.81641433987731,0.0952055756532224,-2.12863252216748)
cB = chrono.ChVectorD(1.97834263044496,0.0952055756532207,-2.05642089766134)
dA = chrono.ChVectorD(0.407284966190289,-1.4887396870833e-14,-0.913301131235134)
dB = chrono.ChVectorD(0.407284966190289,-1.49030093821167e-14,-0.913301131235134)
link_17.Initialize(body_27,body_31,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.81641433987731,0.0952055756532224,-2.12863252216748)
dA = chrono.ChVectorD(0.407284966190289,-1.4887396870833e-14,-0.913301131235134)
cB = chrono.ChVectorD(1.97834263044496,0.0952055756532207,-2.05642089766134)
dB = chrono.ChVectorD(0.407284966190289,-1.49030093821167e-14,-0.913301131235134)
link_18.Initialize(body_27,body_31,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-8/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.81641433987731,0.0952055756532224,-2.12863252216748)
cB = chrono.ChVectorD(1.76262090324759,0.0282055756532193,-2.15262160667632)
dA = chrono.ChVectorD(0.407284966190289,-1.4887396870833e-14,-0.913301131235134)
dB = chrono.ChVectorD(0.407284966190289,-1.56159807307432e-14,-0.913301131235134)
link_19.Initialize(body_27,body_30,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.81641433987731,0.0952055756532224,-2.12863252216748)
dA = chrono.ChVectorD(0.407284966190289,-1.4887396870833e-14,-0.913301131235134)
cB = chrono.ChVectorD(1.76262090324759,0.0282055756532193,-2.15262160667632)
dB = chrono.ChVectorD(0.407284966190289,-1.56159807307432e-14,-0.913301131235134)
link_20.Initialize(body_27,body_30,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-8/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.77650041319067,0.0952055756532239,-2.03912901130643)
cB = chrono.ChVectorD(1.72270697656094,0.0282055756532209,-2.06311809581528)
dA = chrono.ChVectorD(-0.407284966190289,1.55925619638175e-14,0.913301131235134)
dB = chrono.ChVectorD(-0.407284966190289,1.63697180810551e-14,0.913301131235134)
link_21.Initialize(body_27,body_30,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.77650041319067,0.0952055756532239,-2.03912901130643)
dA = chrono.ChVectorD(-0.407284966190289,1.55925619638175e-14,0.913301131235134)
cB = chrono.ChVectorD(1.72270697656094,0.0282055756532209,-2.06311809581528)
dB = chrono.ChVectorD(-0.407284966190289,1.63697180810551e-14,0.913301131235134)
link_22.Initialize(body_27,body_30,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_27 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_31 , SW name: robo_leg_link-8/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.77650041319067,0.0952055756532239,-2.03912901130643)
cB = chrono.ChVectorD(1.93842870375831,0.0952055756532222,-1.9669173868003)
dA = chrono.ChVectorD(-0.407284966190289,1.55925619638175e-14,0.913301131235134)
dB = chrono.ChVectorD(-0.407284966190289,1.41510067552808e-14,0.913301131235134)
link_23.Initialize(body_27,body_31,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.77650041319067,0.0952055756532239,-2.03912901130643)
dA = chrono.ChVectorD(-0.407284966190289,1.55925619638175e-14,0.913301131235134)
cB = chrono.ChVectorD(1.93842870375831,0.0952055756532222,-1.9669173868003)
dB = chrono.ChVectorD(-0.407284966190289,1.41510067552808e-14,0.913301131235134)
link_24.Initialize(body_27,body_31,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_34 , SW name: robo_leg_link-7/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.71892272931436,0.0342055742743635,-2.14650167862855)
dA = chrono.ChVectorD(0.759773788135012,5.07580089070814e-15,0.650187504388522)
cB = chrono.ChVectorD(1.72500091960066,0.0342055756532371,-2.14130017857146)
dB = chrono.ChVectorD(-0.759773788135005,-1.76941794549634e-15,-0.65018750438853)
link_1.SetFlipped(True)
link_1.Initialize(body_34,body_36,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.71892272931436,0.0342055742743635,-2.14650167862855)
cB = chrono.ChVectorD(1.72500091960066,0.0342055756532371,-2.14130017857146)
dA = chrono.ChVectorD(0.759773788135012,5.07580089070814e-15,0.650187504388522)
dB = chrono.ChVectorD(-0.759773788135005,-1.76941794549634e-15,-0.65018750438853)
link_2.Initialize(body_34,body_36,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.76334024584783,0.0282055756532372,-2.14348141879492)
cB = chrono.ChVectorD(1.72303650230103,0.0282055756532059,-2.14034889487404)
dA = chrono.ChVectorD(-3.46944695195361e-16,1,-7.97972798949331e-15)
dB = chrono.ChVectorD(1.52933221642115e-14,-1,3.17176840347599e-14)
link_3.Initialize(body_36,body_32,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.76334024584783,0.0282055756532372,-2.14348141879492)
dA = chrono.ChVectorD(-3.46944695195361e-16,1,-7.97972798949331e-15)
cB = chrono.ChVectorD(1.72303650230103,0.0282055756532059,-2.14034889487404)
dB = chrono.ChVectorD(1.52933221642115e-14,-1,3.17176840347599e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.74963483535512,0.0805155756532569,-2.14769594513801)
dA = chrono.ChVectorD(0.650187504388521,-6.16173778666962e-15,-0.759773788135013)
cB = chrono.ChVectorD(1.71895697946606,0.0384055756532068,-2.1080132372695)
dB = chrono.ChVectorD(-0.650187504388521,1.41137102005473e-14,0.759773788135013)
link_5.SetFlipped(True)
link_5.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_34 , SW name: robo_leg_link-7/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.72500091961944,0.0342055742743635,-2.14130017859344)
cB = chrono.ChVectorD(1.72630194479694,0.0322045756532371,-2.14282048592152)
dA = chrono.ChVectorD(0.759773788135012,5.07580089070814e-15,0.650187504388522)
dB = chrono.ChVectorD(-0.759773788135005,-1.76941794549634e-15,-0.65018750438853)
link_6.Initialize(body_34,body_36,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.72500091961944,0.0342055742743635,-2.14130017859344)
dA = chrono.ChVectorD(0.759773788135012,5.07580089070814e-15,0.650187504388522)
cB = chrono.ChVectorD(1.72630194479694,0.0322045756532371,-2.14282048592152)
dB = chrono.ChVectorD(-0.759773788135005,-1.76941794549634e-15,-0.65018750438853)
link_7.SetFlipped(True)
link_7.Initialize(body_34,body_36,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.72303650230105,0.0961555756532677,-2.14034889487403)
dA = chrono.ChVectorD(1.52655665885959e-14,-1,3.15442116871623e-14)
cB = chrono.ChVectorD(1.72303650230103,0.0282055756532059,-2.14034889487404)
dB = chrono.ChVectorD(-1.52933221642115e-14,1,-3.17176840347599e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.72303650230105,0.0961555756532677,-2.14034889487403)
cB = chrono.ChVectorD(1.72303650230103,0.0282055756532059,-2.14034889487404)
dA = chrono.ChVectorD(1.52655665885959e-14,-1,3.15442116871623e-14)
dB = chrono.ChVectorD(-1.52933221642115e-14,1,-3.17176840347599e-14)
link_9.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.76491902807458,0.0961555756532684,-2.13812994581442)
dA = chrono.ChVectorD(1.52655665885959e-14,-1,3.15442116871623e-14)
cB = chrono.ChVectorD(1.72303650230103,0.0952055756532059,-2.14034889487404)
dB = chrono.ChVectorD(-1.52933221642115e-14,1,-3.17176840347599e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.72303650230105,0.0282055756532372,-2.14034889487403)
dA = chrono.ChVectorD(-3.46944695195361e-16,1,-7.97972798949331e-15)
cB = chrono.ChVectorD(1.72303650230103,0.0282055756532059,-2.14034889487404)
dB = chrono.ChVectorD(-1.52933221642115e-14,1,-3.17176840347599e-14)
link_11.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.72303650230105,0.0282055756532372,-2.14034889487403)
cB = chrono.ChVectorD(1.72303650230103,0.0282055756532059,-2.14034889487404)
dA = chrono.ChVectorD(-3.46944695195361e-16,1,-7.97972798949331e-15)
dB = chrono.ChVectorD(-1.52933221642115e-14,1,-3.17176840347599e-14)
link_12.Initialize(body_36,body_32,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: robo_leg_link-7/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.71454625939838,0.0617055756532082,-2.06206276407519)
dA = chrono.ChVectorD(0.650187504388521,-1.41137102005473e-14,-0.759773788135013)
cB = chrono.ChVectorD(1.70219269681471,0.0617055756532185,-2.04762706210036)
dB = chrono.ChVectorD(0.650187504388521,-1.41414657761629e-14,-0.759773788135013)
link_13.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.71454625939838,0.0617055756532082,-2.06206276407519)
cB = chrono.ChVectorD(1.70219269681471,0.0617055756532185,-2.04762706210036)
dA = chrono.ChVectorD(0.650187504388521,-1.41137102005473e-14,-0.759773788135013)
dB = chrono.ChVectorD(0.650187504388521,-1.41414657761629e-14,-0.759773788135013)
link_14.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.64700236963317,0.0617055756532053,-2.11986443321533)
dA = chrono.ChVectorD(0.650187504388521,-1.41137102005473e-14,-0.759773788135013)
cB = chrono.ChVectorD(1.63464880704984,0.0617055756532176,-2.10542873124089)
dB = chrono.ChVectorD(0.650187504388521,-1.49324996812084e-14,-0.759773788135012)
link_15.Initialize(body_32,body_35,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.64700236963317,0.0617055756532053,-2.11986443321533)
cB = chrono.ChVectorD(1.63464880704984,0.0617055756532176,-2.10542873124089)
dA = chrono.ChVectorD(0.650187504388521,-1.41137102005473e-14,-0.759773788135013)
dB = chrono.ChVectorD(0.650187504388521,-1.49324996812084e-14,-0.759773788135012)
link_16.Initialize(body_32,body_35,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: robo_leg_link-7/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.70991708175847,0.0952055756532033,-2.21606891075246)
cB = chrono.ChVectorD(1.84462497439453,0.095205575653217,-2.1007906662241)
dA = chrono.ChVectorD(0.650187504388521,-1.34059430223488e-14,-0.759773788135013)
dB = chrono.ChVectorD(0.650187504388521,-1.41414657761629e-14,-0.759773788135013)
link_17.Initialize(body_32,body_33,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.70991708175847,0.0952055756532033,-2.21606891075246)
dA = chrono.ChVectorD(0.650187504388521,-1.34059430223488e-14,-0.759773788135013)
cB = chrono.ChVectorD(1.84462497439453,0.095205575653217,-2.1007906662241)
dB = chrono.ChVectorD(0.650187504388521,-1.41414657761629e-14,-0.759773788135013)
link_18.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.70991708175847,0.0952055756532033,-2.21606891075246)
cB = chrono.ChVectorD(1.66516640563737,0.0282055756532152,-2.25436495476106)
dA = chrono.ChVectorD(0.650187504388521,-1.34059430223488e-14,-0.759773788135013)
dB = chrono.ChVectorD(0.650187504388521,-1.49324996812084e-14,-0.759773788135012)
link_19.Initialize(body_32,body_35,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.70991708175847,0.0952055756532033,-2.21606891075246)
dA = chrono.ChVectorD(0.650187504388521,-1.34059430223488e-14,-0.759773788135013)
cB = chrono.ChVectorD(1.66516640563737,0.0282055756532152,-2.25436495476106)
dB = chrono.ChVectorD(0.650187504388521,-1.49324996812084e-14,-0.759773788135012)
link_20.Initialize(body_32,body_35,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.6461987063284,0.0952055756532047,-2.14161107951522)
cB = chrono.ChVectorD(1.60144803020729,0.0282055756532167,-2.17990712352383)
dA = chrono.ChVectorD(-0.650187504388521,1.41137102005473e-14,0.759773788135013)
dB = chrono.ChVectorD(-0.650187504388521,1.56680224350225e-14,0.759773788135012)
link_21.Initialize(body_32,body_35,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.6461987063284,0.0952055756532047,-2.14161107951522)
dA = chrono.ChVectorD(-0.650187504388521,1.41137102005473e-14,0.759773788135013)
cB = chrono.ChVectorD(1.60144803020729,0.0282055756532167,-2.17990712352383)
dB = chrono.ChVectorD(-0.650187504388521,1.56680224350225e-14,0.759773788135012)
link_22.Initialize(body_32,body_35,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: robo_leg_link-7/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.6461987063284,0.0952055756532047,-2.14161107951522)
cB = chrono.ChVectorD(1.78090659896445,0.0952055756532184,-2.02633283498687)
dA = chrono.ChVectorD(-0.650187504388521,1.41137102005473e-14,0.759773788135013)
dB = chrono.ChVectorD(-0.650187504388521,1.3392065234541e-14,0.759773788135013)
link_23.Initialize(body_32,body_33,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.6461987063284,0.0952055756532047,-2.14161107951522)
dA = chrono.ChVectorD(-0.650187504388521,1.41137102005473e-14,0.759773788135013)
cB = chrono.ChVectorD(1.78090659896445,0.0952055756532184,-2.02633283498687)
dB = chrono.ChVectorD(-0.650187504388521,1.3392065234541e-14,0.759773788135013)
link_24.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_38 , SW name: robo_leg_link-6/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.58194348786147,0.0342055760792723,-2.19492058513386)
dA = chrono.ChVectorD(0.999339199504135,-5.57887069874141e-15,0.0363478243425172)
cB = chrono.ChVectorD(1.5899382014575,0.0342055760792372,-2.19462980253912)
dB = chrono.ChVectorD(-0.999339199504135,8.65973959207622e-15,-0.0363478243425267)
link_1.SetFlipped(True)
link_1.Initialize(body_38,body_41,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.58194348786147,0.0342055760792723,-2.19492058513386)
cB = chrono.ChVectorD(1.5899382014575,0.0342055760792372,-2.19462980253912)
dA = chrono.ChVectorD(0.999339199504135,-5.57887069874141e-15,0.0363478243425172)
dB = chrono.ChVectorD(-0.999339199504135,8.65973959207622e-15,-0.0363478243425267)
link_2.Initialize(body_38,body_41,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.6185971969251,0.028205576079237,-2.22019000059758)
cB = chrono.ChVectorD(1.58899208334844,0.0282055760792379,-2.19266289213859)
dA = chrono.ChVectorD(5.24580379135386e-15,1,-7.72298891504875e-15)
dB = chrono.ChVectorD(2.1316282072803e-14,-1,1.69239622316297e-14)
link_3.Initialize(body_41,body_37,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.6185971969251,0.028205576079237,-2.22019000059758)
dA = chrono.ChVectorD(5.24580379135386e-15,1,-7.72298891504875e-15)
cB = chrono.ChVectorD(1.58899208334844,0.0282055760792379,-2.19266289213859)
dB = chrono.ChVectorD(2.1316282072803e-14,-1,1.69239622316297e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.6052451348529,0.0805155760792569,-2.21496286491055)
dA = chrono.ChVectorD(0.0363478243425161,-8.26422263955351e-15,-0.999339199504135)
cB = chrono.ChVectorD(1.60591556818685,0.0384055760792387,-2.16480911465324)
dB = chrono.ChVectorD(-0.0363478243425152,1.61329283265843e-14,0.999339199504135)
link_5.SetFlipped(True)
link_5.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_38 , SW name: robo_leg_link-6/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.5899382014575,0.0342055760792723,-2.19462980253912)
cB = chrono.ChVectorD(1.59001093345401,0.0322045760792372,-2.19662948027733)
dA = chrono.ChVectorD(0.999339199504135,-5.57887069874141e-15,0.0363478243425172)
dB = chrono.ChVectorD(-0.999339199504135,8.65973959207622e-15,-0.0363478243425267)
link_6.Initialize(body_38,body_41,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.5899382014575,0.0342055760792723,-2.19462980253912)
dA = chrono.ChVectorD(0.999339199504135,-5.57887069874141e-15,0.0363478243425172)
cB = chrono.ChVectorD(1.59001093345401,0.0322045760792372,-2.19662948027733)
dB = chrono.ChVectorD(-0.999339199504135,8.65973959207622e-15,-0.0363478243425267)
link_7.SetFlipped(True)
link_7.Initialize(body_38,body_41,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.58899208334844,0.0961555760792679,-2.19266289213859)
dA = chrono.ChVectorD(2.11081152556858e-14,-1,1.68962066560141e-14)
cB = chrono.ChVectorD(1.58899208334844,0.0282055760792379,-2.19266289213859)
dB = chrono.ChVectorD(-2.1316282072803e-14,1,-1.69239622316297e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.58899208334844,0.0961555760792679,-2.19266289213859)
cB = chrono.ChVectorD(1.58899208334844,0.0282055760792379,-2.19266289213859)
dA = chrono.ChVectorD(2.11081152556858e-14,-1,1.68962066560141e-14)
dB = chrono.ChVectorD(-2.1316282072803e-14,1,-1.69239622316297e-14)
link_9.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.62316260753098,0.0961555760792682,-2.21698253393675)
dA = chrono.ChVectorD(2.11081152556858e-14,-1,1.68962066560141e-14)
cB = chrono.ChVectorD(1.58899208334844,0.0952055760792379,-2.19266289213859)
dB = chrono.ChVectorD(-2.1316282072803e-14,1,-1.69239622316297e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.58899208334845,0.0282055760792373,-2.19266289213859)
dA = chrono.ChVectorD(5.24580379135386e-15,1,-7.72298891504875e-15)
cB = chrono.ChVectorD(1.58899208334844,0.0282055760792379,-2.19266289213859)
dB = chrono.ChVectorD(-2.1316282072803e-14,1,-1.69239622316297e-14)
link_11.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.58899208334845,0.0282055760792373,-2.19266289213859)
cB = chrono.ChVectorD(1.58899208334844,0.0282055760792379,-2.19266289213859)
dA = chrono.ChVectorD(5.24580379135386e-15,1,-7.72298891504875e-15)
dB = chrono.ChVectorD(-2.1316282072803e-14,1,-1.69239622316297e-14)
link_12.Initialize(body_41,body_37,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-6/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.63105010218414,0.0617055760792399,-2.12609018337879)
dA = chrono.ChVectorD(0.0363478243425152,-1.61329283265843e-14,-0.999339199504135)
cB = chrono.ChVectorD(1.63035949352162,0.0617055760792405,-2.10710273858819)
dB = chrono.ChVectorD(0.0363478243425156,-1.54251611483858e-14,-0.999339199504135)
link_13.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.63105010218414,0.0617055760792399,-2.12609018337879)
cB = chrono.ChVectorD(1.63035949352162,0.0617055760792405,-2.10710273858819)
dA = chrono.ChVectorD(0.0363478243425152,-1.61329283265843e-14,-0.999339199504135)
dB = chrono.ChVectorD(0.0363478243425156,-1.54251611483858e-14,-0.999339199504135)
link_14.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-6/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.54220884734822,0.0617055760792379,-2.12932150496284)
dA = chrono.ChVectorD(0.0363478243425152,-1.61329283265843e-14,-0.999339199504135)
cB = chrono.ChVectorD(1.54151823868571,0.0617055760792388,-2.11033406017227)
dB = chrono.ChVectorD(0.0363478243425154,-1.61259894326804e-14,-0.999339199504135)
link_15.Initialize(body_37,body_39,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.54220884734822,0.0617055760792379,-2.12932150496284)
cB = chrono.ChVectorD(1.54151823868571,0.0617055760792388,-2.11033406017227)
dA = chrono.ChVectorD(0.0363478243425152,-1.61329283265843e-14,-0.999339199504135)
dB = chrono.ChVectorD(0.0363478243425154,-1.61259894326804e-14,-0.999339199504135)
link_16.Initialize(body_37,body_39,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-6/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.53161224613058,0.0952055760792358,-2.24378230411537)
cB = chrono.ChVectorD(1.70879508620266,0.0952055760792381,-2.23733783485941)
dA = chrono.ChVectorD(0.0363478243425153,-1.54182222544819e-14,-0.999339199504135)
dB = chrono.ChVectorD(0.0363478243425156,-1.54251611483858e-14,-0.999339199504135)
link_17.Initialize(body_37,body_40,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.53161224613058,0.0952055760792358,-2.24378230411537)
dA = chrono.ChVectorD(0.0363478243425153,-1.54182222544819e-14,-0.999339199504135)
cB = chrono.ChVectorD(1.70879508620266,0.0952055760792381,-2.23733783485941)
dB = chrono.ChVectorD(0.0363478243425156,-1.54251611483858e-14,-0.999339199504135)
link_18.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-6/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.53161224613058,0.0952055760792358,-2.24378230411537)
cB = chrono.ChVectorD(1.47275116727978,0.028205576079237,-2.24592319096915)
dA = chrono.ChVectorD(0.0363478243425153,-1.54182222544819e-14,-0.999339199504135)
dB = chrono.ChVectorD(0.0363478243425154,-1.61259894326804e-14,-0.999339199504135)
link_19.Initialize(body_37,body_39,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.53161224613058,0.0952055760792358,-2.24378230411537)
dA = chrono.ChVectorD(0.0363478243425153,-1.54182222544819e-14,-0.999339199504135)
cB = chrono.ChVectorD(1.47275116727978,0.028205576079237,-2.24592319096915)
dB = chrono.ChVectorD(0.0363478243425154,-1.61259894326804e-14,-0.999339199504135)
link_20.Initialize(body_37,body_39,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-6/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.52805015934501,0.0952055760792374,-2.14584706256396)
cB = chrono.ChVectorD(1.46918908049421,0.0282055760792386,-2.14798794941774)
dA = chrono.ChVectorD(-0.0363478243425152,1.61329283265843e-14,0.999339199504135)
dB = chrono.ChVectorD(-0.0363478243425154,1.68823288682063e-14,0.999339199504135)
link_21.Initialize(body_37,body_39,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.52805015934501,0.0952055760792374,-2.14584706256396)
dA = chrono.ChVectorD(-0.0363478243425152,1.61329283265843e-14,0.999339199504135)
cB = chrono.ChVectorD(1.46918908049421,0.0282055760792386,-2.14798794941774)
dB = chrono.ChVectorD(-0.0363478243425154,1.68823288682063e-14,0.999339199504135)
link_22.Initialize(body_37,body_39,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-6/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.52805015934501,0.0952055760792374,-2.14584706256396)
cB = chrono.ChVectorD(1.70523299941709,0.0952055760792396,-2.13940259330801)
dA = chrono.ChVectorD(-0.0363478243425152,1.61329283265843e-14,0.999339199504135)
dB = chrono.ChVectorD(-0.0363478243425156,1.46757606067638e-14,0.999339199504135)
link_23.Initialize(body_37,body_40,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.52805015934501,0.0952055760792374,-2.14584706256396)
dA = chrono.ChVectorD(-0.0363478243425152,1.61329283265843e-14,0.999339199504135)
cB = chrono.ChVectorD(1.70523299941709,0.0952055760792396,-2.13940259330801)
dB = chrono.ChVectorD(-0.0363478243425156,1.46757606067638e-14,0.999339199504135)
link_24.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-9/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.96088101577675,0.034205575168286,-1.97293453265807)
dA = chrono.ChVectorD(0.566265392288243,-5.55111512312578e-17,0.82422297074071)
cB = chrono.ChVectorD(1.96541113890788,0.0342055756532478,-1.9663407488872)
dB = chrono.ChVectorD(-0.566265392288234,4.26048085699904e-15,-0.824222970740717)
link_1.SetFlipped(True)
link_1.Initialize(body_44,body_45,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.96088101577675,0.034205575168286,-1.97293453265807)
cB = chrono.ChVectorD(1.96541113890788,0.0342055756532478,-1.9663407488872)
dA = chrono.ChVectorD(0.566265392288243,-5.55111512312578e-17,0.82422297074071)
dB = chrono.ChVectorD(-0.566265392288234,4.26048085699904e-15,-0.824222970740717)
link_2.Initialize(body_44,body_45,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.00301488687072,0.0282055756532477,-1.95855487159899)
cB = chrono.ChVectorD(1.96326777701352,0.0282055756532122,-1.96592858897907)
dA = chrono.ChVectorD(4.65686517125974e-15,1,-2.51187959321442e-15)
dB = chrono.ChVectorD(2.60728938439314e-15,-1,2.77694534034367e-14)
link_3.Initialize(body_45,body_46,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00301488687072,0.0282055756532477,-1.95855487159899)
dA = chrono.ChVectorD(4.65686517125974e-15,1,-2.51187959321442e-15)
cB = chrono.ChVectorD(1.96326777701352,0.0282055756532122,-1.96592858897907)
dB = chrono.ChVectorD(2.60728938439314e-15,-1,2.77694534034367e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.99086117104978,0.0805155756532676,-1.96616326832985)
dA = chrono.ChVectorD(0.824222970740711,-5.59274848654923e-15,-0.566265392288243)
cB = chrono.ChVectorD(1.95098238206882,0.038405575653213,-1.93574073827168)
dB = chrono.ChVectorD(-0.824222970740711,1.36071709455621e-14,0.566265392288243)
link_5.SetFlipped(True)
link_5.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-9/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.96541113891505,0.034205575168286,-1.96634074889214)
cB = chrono.ChVectorD(1.96706040907233,0.0322045756532478,-1.96747384593717)
dA = chrono.ChVectorD(0.566265392288243,-5.55111512312578e-17,0.82422297074071)
dB = chrono.ChVectorD(-0.566265392288234,4.26048085699904e-15,-0.824222970740717)
link_6.Initialize(body_44,body_45,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.96541113891505,0.034205575168286,-1.96634074889214)
dA = chrono.ChVectorD(0.566265392288243,-5.55111512312578e-17,0.82422297074071)
cB = chrono.ChVectorD(1.96706040907233,0.0322045756532478,-1.96747384593717)
dB = chrono.ChVectorD(-0.566265392288234,4.26048085699904e-15,-0.824222970740717)
link_7.SetFlipped(True)
link_7.Initialize(body_44,body_45,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.96326777701354,0.0961555756532785,-1.96592858897906)
dA = chrono.ChVectorD(3.64899083171721e-15,-1,2.91294766086025e-14)
cB = chrono.ChVectorD(1.96326777701352,0.0282055756532122,-1.96592858897907)
dB = chrono.ChVectorD(-2.60728938439314e-15,1,-2.77694534034367e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.96326777701354,0.0961555756532785,-1.96592858897906)
cB = chrono.ChVectorD(1.96326777701352,0.0282055756532122,-1.96592858897907)
dA = chrono.ChVectorD(3.64899083171721e-15,-1,2.91294766086025e-14)
dB = chrono.ChVectorD(-2.60728938439314e-15,1,-2.77694534034367e-14)
link_9.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00315928336277,0.096155575653279,-1.95297724073691)
dA = chrono.ChVectorD(3.64899083171721e-15,-1,2.91294766086025e-14)
cB = chrono.ChVectorD(1.96326777701352,0.0952055756532122,-1.96592858897907)
dB = chrono.ChVectorD(-2.60728938439314e-15,1,-2.77694534034367e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.96326777701354,0.0282055756532479,-1.96592858897906)
dA = chrono.ChVectorD(4.65686517125974e-15,1,-2.51187959321442e-15)
cB = chrono.ChVectorD(1.96326777701352,0.0282055756532122,-1.96592858897907)
dB = chrono.ChVectorD(-2.60728938439314e-15,1,-2.77694534034367e-14)
link_11.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.96326777701354,0.0282055756532479,-1.96592858897906)
cB = chrono.ChVectorD(1.96326777701352,0.0282055756532122,-1.96592858897907)
dA = chrono.ChVectorD(4.65686517125974e-15,1,-2.51187959321442e-15)
dB = chrono.ChVectorD(-2.60728938439314e-15,1,-2.77694534034367e-14)
link_12.Initialize(body_45,body_46,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_42 , SW name: robo_leg_link-9/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.93486378060258,0.0617055756532141,-1.89248462743091)
dA = chrono.ChVectorD(0.824222970740711,-1.36071709455621e-14,-0.566265392288243)
cB = chrono.ChVectorD(1.9192035441586,0.0617055756532287,-1.88172558497748)
dB = chrono.ChVectorD(0.824222970740711,-1.29132815551714e-14,-0.566265392288242)
link_13.Initialize(body_46,body_42,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.93486378060258,0.0617055756532141,-1.89248462743091)
cB = chrono.ChVectorD(1.9192035441586,0.0617055756532287,-1.88172558497748)
dA = chrono.ChVectorD(0.824222970740711,-1.36071709455621e-14,-0.566265392288243)
dB = chrono.ChVectorD(0.824222970740711,-1.29132815551714e-14,-0.566265392288242)
link_14.Initialize(body_46,body_42,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-9/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.88452278722816,0.061705575653212,-1.96575804952976)
dA = chrono.ChVectorD(0.824222970740711,-1.36071709455621e-14,-0.566265392288243)
cB = chrono.ChVectorD(1.86886255078406,0.0617055756532291,-1.95499900707625)
dB = chrono.ChVectorD(0.824222970740711,-1.3614109839466e-14,-0.566265392288243)
link_15.Initialize(body_46,body_43,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.88452278722816,0.061705575653212,-1.96575804952976)
cB = chrono.ChVectorD(1.86886255078406,0.0617055756532291,-1.95499900707625)
dA = chrono.ChVectorD(0.824222970740711,-1.36071709455621e-14,-0.566265392288243)
dB = chrono.ChVectorD(0.824222970740711,-1.3614109839466e-14,-0.566265392288243)
link_16.Initialize(body_46,body_43,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_42 , SW name: robo_leg_link-9/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.97013179135635,0.0952055756532101,-2.04246959306904)
cB = chrono.ChVectorD(2.07053064540915,0.0952055756532269,-1.89633486035676)
dA = chrono.ChVectorD(0.824222970740711,-1.29063426612674e-14,-0.566265392288242)
dB = chrono.ChVectorD(0.824222970740711,-1.29132815551714e-14,-0.566265392288242)
link_17.Initialize(body_46,body_42,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.97013179135635,0.0952055756532101,-2.04246959306904)
dA = chrono.ChVectorD(0.824222970740711,-1.29063426612674e-14,-0.566265392288242)
cB = chrono.ChVectorD(2.07053064540915,0.0952055756532269,-1.89633486035676)
dB = chrono.ChVectorD(0.824222970740711,-1.29132815551714e-14,-0.566265392288242)
link_18.Initialize(body_46,body_42,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-9/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.97013179135635,0.0952055756532101,-2.04246959306904)
cB = chrono.ChVectorD(1.93677875975055,0.0282055756532274,-2.09101632604564)
dA = chrono.ChVectorD(0.824222970740711,-1.29063426612674e-14,-0.566265392288242)
dB = chrono.ChVectorD(0.824222970740711,-1.3614109839466e-14,-0.566265392288243)
link_19.Initialize(body_46,body_43,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.97013179135635,0.0952055756532101,-2.04246959306904)
dA = chrono.ChVectorD(0.824222970740711,-1.29063426612674e-14,-0.566265392288242)
cB = chrono.ChVectorD(1.93677875975055,0.0282055756532274,-2.09101632604564)
dB = chrono.ChVectorD(0.824222970740711,-1.3614109839466e-14,-0.566265392288243)
link_20.Initialize(body_46,body_43,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-9/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.88935794022376,0.0952055756532114,-1.9869755846248)
cB = chrono.ChVectorD(1.85600490861796,0.0282055756532287,-2.03552231760139)
dA = chrono.ChVectorD(-0.824222970740711,1.36071709455621e-14,0.566265392288243)
dB = chrono.ChVectorD(-0.824222970740711,1.43704492749919e-14,0.566265392288242)
link_21.Initialize(body_46,body_43,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.88935794022376,0.0952055756532114,-1.9869755846248)
dA = chrono.ChVectorD(-0.824222970740711,1.36071709455621e-14,0.566265392288243)
cB = chrono.ChVectorD(1.85600490861796,0.0282055756532287,-2.03552231760139)
dB = chrono.ChVectorD(-0.824222970740711,1.43704492749919e-14,0.566265392288242)
link_22.Initialize(body_46,body_43,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_42 , SW name: robo_leg_link-9/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.88935794022376,0.0952055756532114,-1.9869755846248)
cB = chrono.ChVectorD(1.98975679427656,0.0952055756532281,-1.84084085191251)
dA = chrono.ChVectorD(-0.824222970740711,1.36071709455621e-14,0.566265392288243)
dB = chrono.ChVectorD(-0.824222970740711,1.21638810135494e-14,0.566265392288242)
link_23.Initialize(body_46,body_42,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.88935794022376,0.0952055756532114,-1.9869755846248)
dA = chrono.ChVectorD(-0.824222970740711,1.36071709455621e-14,0.566265392288243)
cB = chrono.ChVectorD(1.98975679427656,0.0952055756532281,-1.84084085191251)
dB = chrono.ChVectorD(-0.824222970740711,1.21638810135494e-14,0.566265392288242)
link_24.Initialize(body_46,body_42,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_50 , SW name: robo_leg_link-10/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00844629582269,0.03420557561238,-1.83404964565574)
dA = chrono.ChVectorD(0.000421208212830482,2.16493489801906e-15,0.999999911291817)
cB = chrono.ChVectorD(2.008449665466,0.0342055756608154,-1.82604964640795)
dB = chrono.ChVectorD(-0.000421208212827643,-2.97331603782425e-15,-0.999999911291817)
link_1.SetFlipped(True)
link_1.Initialize(body_50,body_47,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(2.00844629582269,0.03420557561238,-1.83404964565574)
cB = chrono.ChVectorD(2.008449665466,0.0342055756608154,-1.82604964640795)
dA = chrono.ChVectorD(0.000421208212830482,2.16493489801906e-15,0.999999911291817)
dB = chrono.ChVectorD(-0.000421208212827643,-2.97331603782425e-15,-0.999999911291817)
link_2.Initialize(body_50,body_47,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.03504633520848,0.0282055756608156,-1.79834984668648)
cB = chrono.ChVectorD(2.00644929750744,0.0282055756607767,-1.82692280391395)
dA = chrono.ChVectorD(2.80331313717852e-15,1,-6.66133814775094e-15)
dB = chrono.ChVectorD(-1.13381526389844e-14,-1,3.27203542038745e-14)
link_3.Initialize(body_47,body_51,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.03504633520848,0.0282055756608156,-1.79834984668648)
dA = chrono.ChVectorD(2.80331313717852e-15,1,-6.66133814775094e-15)
cB = chrono.ChVectorD(2.00644929750744,0.0282055756607767,-1.82692280391395)
dB = chrono.ChVectorD(-1.13381526389844e-14,-1,3.27203542038745e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.02933179546052,0.0805155756608354,-1.81150068485519)
dA = chrono.ChVectorD(0.999999911291817,-3.15025783237388e-15,-0.000421208212838567)
cB = chrono.ChVectorD(1.9792366101227,0.0384055756607775,-1.8089866141129)
dB = chrono.ChVectorD(-0.999999911291817,1.13520304267922e-14,0.000421208212839321)
link_5.SetFlipped(True)
link_5.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_50 , SW name: robo_leg_link-10/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.00844966548839,0.03420557561238,-1.82604964636541)
cB = chrono.ChVectorD(2.01045066528849,0.0322045756608154,-1.82605048924558)
dA = chrono.ChVectorD(0.000421208212830482,2.16493489801906e-15,0.999999911291817)
dB = chrono.ChVectorD(-0.000421208212827643,-2.97331603782425e-15,-0.999999911291817)
link_6.Initialize(body_50,body_47,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00844966548839,0.03420557561238,-1.82604964636541)
dA = chrono.ChVectorD(0.000421208212830482,2.16493489801906e-15,0.999999911291817)
cB = chrono.ChVectorD(2.01045066528849,0.0322045756608154,-1.82605048924558)
dB = chrono.ChVectorD(-0.000421208212827643,-2.97331603782425e-15,-0.999999911291817)
link_7.SetFlipped(True)
link_7.Initialize(body_50,body_47,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00644929750744,0.0961555756608461,-1.826922803914)
dA = chrono.ChVectorD(-1.10050857315969e-14,-1,3.33205685265625e-14)
cB = chrono.ChVectorD(2.00644929750744,0.0282055756607767,-1.82692280391395)
dB = chrono.ChVectorD(1.13381526389844e-14,1,-3.27203542038745e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(2.00644929750744,0.0961555756608461,-1.826922803914)
cB = chrono.ChVectorD(2.00644929750744,0.0282055756607767,-1.82692280391395)
dA = chrono.ChVectorD(-1.10050857315969e-14,-1,3.33205685265625e-14)
dB = chrono.ChVectorD(1.13381526389844e-14,1,-3.27203542038745e-14)
link_9.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.03200890187963,0.0961555756608469,-1.79366958866439)
dA = chrono.ChVectorD(-1.10050857315969e-14,-1,3.33205685265625e-14)
cB = chrono.ChVectorD(2.00644929750744,0.0952055756607767,-1.82692280391396)
dB = chrono.ChVectorD(1.13381526389844e-14,1,-3.27203542038745e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.00644929750744,0.0282055756608155,-1.82692280391399)
dA = chrono.ChVectorD(2.80331313717852e-15,1,-6.66133814775094e-15)
cB = chrono.ChVectorD(2.00644929750744,0.0282055756607767,-1.82692280391395)
dB = chrono.ChVectorD(1.13381526389844e-14,1,-3.27203542038745e-14)
link_11.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(2.00644929750744,0.0282055756608155,-1.82692280391399)
cB = chrono.ChVectorD(2.00644929750744,0.0282055756607767,-1.82692280391395)
dA = chrono.ChVectorD(2.80331313717852e-15,1,-6.66133814775094e-15)
dB = chrono.ChVectorD(1.13381526389844e-14,1,-3.27203542038745e-14)
link_12.Initialize(body_47,body_51,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_49 , SW name: robo_leg_link-10/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.94146802597853,0.0617055756607788,-1.7824454293232)
dA = chrono.ChVectorD(0.999999911291817,-1.13520304267922e-14,-0.000421208212839321)
cB = chrono.ChVectorD(1.92246802766503,0.0617055756607022,-1.78243742636737)
dB = chrono.ChVectorD(0.999999911291817,-1.06026298851702e-14,-0.000421208212839561)
link_13.Initialize(body_51,body_49,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.94146802597853,0.0617055756607788,-1.7824454293232)
cB = chrono.ChVectorD(1.92246802766503,0.0617055756607022,-1.78243742636737)
dA = chrono.ChVectorD(0.999999911291817,-1.13520304267922e-14,-0.000421208212839321)
dB = chrono.ChVectorD(0.999999911291817,-1.06026298851702e-14,-0.000421208212839561)
link_14.Initialize(body_51,body_49,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-10/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.94143058056841,0.0617055756607759,-1.87134542143704)
dA = chrono.ChVectorD(0.999999911291817,-1.13520304267922e-14,-0.000421208212839321)
cB = chrono.ChVectorD(1.9224305822535,0.0617055756607646,-1.87133741848088)
dB = chrono.ChVectorD(0.999999911291817,-1.1296519275561e-14,-0.000421208212839433)
link_15.Initialize(body_51,body_48,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.94143058056841,0.0617055756607759,-1.87134542143704)
cB = chrono.ChVectorD(1.9224305822535,0.0617055756607646,-1.87133741848088)
dA = chrono.ChVectorD(0.999999911291817,-1.13520304267922e-14,-0.000421208212839321)
dB = chrono.ChVectorD(0.999999911291817,-1.1296519275561e-14,-0.000421208212839433)
link_16.Initialize(body_51,body_48,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_49 , SW name: robo_leg_link-10/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.05542435763454,0.0952055756607742,-1.88614343786486)
cB = chrono.ChVectorD(2.05549903785172,0.0952055756607012,-1.70884345359304)
dA = chrono.ChVectorD(0.999999911291817,-1.06303854607859e-14,-0.000421208212839282)
dB = chrono.ChVectorD(0.999999911291817,-1.06026298851702e-14,-0.000421208212839561)
link_17.Initialize(body_51,body_49,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.05542435763454,0.0952055756607742,-1.88614343786486)
dA = chrono.ChVectorD(0.999999911291817,-1.06303854607859e-14,-0.000421208212839282)
cB = chrono.ChVectorD(2.05549903785172,0.0952055756607012,-1.70884345359304)
dB = chrono.ChVectorD(0.999999911291817,-1.06026298851702e-14,-0.000421208212839561)
link_18.Initialize(body_51,body_49,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-10/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.05542435763454,0.0952055756607742,-1.88614343786486)
cB = chrono.ChVectorD(2.05539954847043,0.0282055756607627,-1.94504343263983)
dA = chrono.ChVectorD(0.999999911291817,-1.06303854607859e-14,-0.000421208212839282)
dB = chrono.ChVectorD(0.999999911291817,-1.1296519275561e-14,-0.000421208212839433)
link_19.Initialize(body_51,body_48,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.05542435763454,0.0952055756607742,-1.88614343786486)
dA = chrono.ChVectorD(0.999999911291817,-1.06303854607859e-14,-0.000421208212839282)
cB = chrono.ChVectorD(2.05539954847043,0.0282055756607627,-1.94504343263983)
dB = chrono.ChVectorD(0.999999911291817,-1.1296519275561e-14,-0.000421208212839433)
link_20.Initialize(body_51,body_48,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-10/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95742436632794,0.0952055756607753,-1.88610215946)
cB = chrono.ChVectorD(1.95739955716383,0.0282055756607638,-1.94500215423498)
dA = chrono.ChVectorD(-0.999999911291817,1.13520304267922e-14,0.000421208212839321)
dB = chrono.ChVectorD(-0.999999911291817,1.20597976049908e-14,0.000421208212839471)
link_21.Initialize(body_51,body_48,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95742436632794,0.0952055756607753,-1.88610215946)
dA = chrono.ChVectorD(-0.999999911291817,1.13520304267922e-14,0.000421208212839321)
cB = chrono.ChVectorD(1.95739955716383,0.0282055756607638,-1.94500215423498)
dB = chrono.ChVectorD(-0.999999911291817,1.20597976049908e-14,0.000421208212839471)
link_22.Initialize(body_51,body_48,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_49 , SW name: robo_leg_link-10/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95742436632794,0.0952055756607753,-1.88610215946)
cB = chrono.ChVectorD(1.95749904654512,0.0952055756607022,-1.70880217518818)
dA = chrono.ChVectorD(-0.999999911291817,1.13520304267922e-14,0.000421208212839321)
dB = chrono.ChVectorD(-0.999999911291817,9.85322934354826e-15,0.000421208212839523)
link_23.Initialize(body_51,body_49,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95742436632794,0.0952055756607753,-1.88610215946)
dA = chrono.ChVectorD(-0.999999911291817,1.13520304267922e-14,0.000421208212839321)
cB = chrono.ChVectorD(1.95749904654512,0.0952055756607022,-1.70880217518818)
dB = chrono.ChVectorD(-0.999999911291817,9.85322934354826e-15,0.000421208212839523)
link_24.Initialize(body_51,body_49,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_55 , SW name: robo_leg_link-11/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.9964796551536,0.0342055756576191,-1.68234750593182)
dA = chrono.ChVectorD(-0.173313669984081,7.76722436368615e-15,0.984866677168362)
cB = chrono.ChVectorD(1.99509314579304,0.0342055756683768,-1.67446857251481)
dB = chrono.ChVectorD(0.173313669984095,-4.73579508941668e-15,-0.984866677168359)
link_1.SetFlipped(True)
link_1.Initialize(body_55,body_53,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.9964796551536,0.0342055756576191,-1.68234750593182)
cB = chrono.ChVectorD(1.99509314579304,0.0342055756683768,-1.67446857251481)
dA = chrono.ChVectorD(-0.173313669984081,7.76722436368615e-15,0.984866677168362)
dB = chrono.ChVectorD(0.173313669984095,-4.73579508941668e-15,-0.984866677168359)
link_2.Initialize(body_55,body_53,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.01647313129663,0.0282055756683772,-1.64256938810727)
cB = chrono.ChVectorD(1.99327488858649,0.028205575669298,-1.67567597333031)
dA = chrono.ChVectorD(1.83186799063151e-15,1,-8.25034485174569e-15)
dB = chrono.ChVectorD(-1.43079992298567e-14,-1,3.27758653551058e-14)
link_3.Initialize(body_53,body_56,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.01647313129663,0.0282055756683772,-1.64256938810727)
dA = chrono.ChVectorD(1.83186799063151e-15,1,-8.25034485174569e-15)
cB = chrono.ChVectorD(1.99327488858649,0.028205575669298,-1.67567597333031)
dB = chrono.ChVectorD(-1.43079992298567e-14,-1,3.27758653551058e-14)
link_4.SetFlipped(True)
link_4.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.01313016442881,0.0805155756683969,-1.65651302757447)
dA = chrono.ChVectorD(0.984866677168361,-7.07767178198537e-16,0.173313669984084)
cB = chrono.ChVectorD(1.9633599813754,0.0384055756692988,-1.66274014765775)
dB = chrono.ChVectorD(-0.984866677168361,8.39606162372775e-15,-0.173313669984081)
link_5.SetFlipped(True)
link_5.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_55 , SW name: robo_leg_link-11/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.99509314579373,0.0342055756576192,-1.67446857251447)
cB = chrono.ChVectorD(1.99706386401405,0.0322045756683769,-1.67412177186117)
dA = chrono.ChVectorD(-0.173313669984081,7.76722436368615e-15,0.984866677168362)
dB = chrono.ChVectorD(0.173313669984095,-4.73579508941668e-15,-0.984866677168359)
link_6.Initialize(body_55,body_53,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.99509314579373,0.0342055756576192,-1.67446857251447)
dA = chrono.ChVectorD(-0.173313669984081,7.76722436368615e-15,0.984866677168362)
cB = chrono.ChVectorD(1.99706386401405,0.0322045756683769,-1.67412177186117)
dB = chrono.ChVectorD(0.173313669984095,-4.73579508941668e-15,-0.984866677168359)
link_7.SetFlipped(True)
link_7.Initialize(body_55,body_53,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.99327488858627,0.0961555756684075,-1.67567597333063)
dA = chrono.ChVectorD(-1.45439216225896e-14,-1,3.30811766868777e-14)
cB = chrono.ChVectorD(1.99327488858649,0.028205575669298,-1.67567597333031)
dB = chrono.ChVectorD(1.43079992298567e-14,1,-3.27758653551058e-14)
link_8.SetFlipped(True)
link_8.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.99327488858627,0.0961555756684075,-1.67567597333063)
cB = chrono.ChVectorD(1.99327488858649,0.028205575669298,-1.67567597333031)
dA = chrono.ChVectorD(-1.45439216225896e-14,-1,3.30811766868777e-14)
dB = chrono.ChVectorD(1.43079992298567e-14,1,-3.27758653551058e-14)
link_9.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.01266879227623,0.0961555756684084,-1.6384879887151)
dA = chrono.ChVectorD(-1.45439216225896e-14,-1,3.30811766868777e-14)
cB = chrono.ChVectorD(1.99327488858649,0.095205575669298,-1.67567597333031)
dB = chrono.ChVectorD(1.43079992298567e-14,1,-3.27758653551058e-14)
link_10.SetFlipped(True)
link_10.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.99327488858627,0.0282055756683769,-1.67567597333062)
dA = chrono.ChVectorD(1.83186799063151e-15,1,-8.25034485174569e-15)
cB = chrono.ChVectorD(1.99327488858649,0.028205575669298,-1.67567597333031)
dB = chrono.ChVectorD(1.43079992298567e-14,1,-3.27758653551058e-14)
link_11.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.99327488858627,0.0282055756683769,-1.67567597333062)
cB = chrono.ChVectorD(1.99327488858649,0.028205575669298,-1.67567597333031)
dA = chrono.ChVectorD(1.83186799063151e-15,1,-8.25034485174569e-15)
dB = chrono.ChVectorD(1.43079992298567e-14,1,-3.27758653551058e-14)
link_12.Initialize(body_53,body_56,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_54 , SW name: robo_leg_link-11/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.92155476193975,0.0617055756693001,-1.64316403807914)
dA = chrono.ChVectorD(0.984866677168361,-8.39606162372775e-15,0.173313669984081)
cB = chrono.ChVectorD(1.90284229507355,0.0617055756686302,-1.64645699780891)
dB = chrono.ChVectorD(0.984866677168361,-8.34055047249649e-15,0.173313669984081)
link_13.Initialize(body_56,body_54,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.92155476193975,0.0617055756693001,-1.64316403807914)
cB = chrono.ChVectorD(1.90284229507355,0.0617055756686302,-1.64645699780891)
dA = chrono.ChVectorD(0.984866677168361,-8.39606162372775e-15,0.173313669984081)
dB = chrono.ChVectorD(0.984866677168361,-8.34055047249649e-15,0.173313669984081)
link_14.Initialize(body_56,body_54,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-11/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.93696234720134,0.061705575669297,-1.73071868567941)
dA = chrono.ChVectorD(0.984866677168361,-8.39606162372775e-15,0.173313669984081)
cB = chrono.ChVectorD(1.91824988033514,0.0617055756685828,-1.73401164540918)
dB = chrono.ChVectorD(0.984866677168361,-8.40993941153556e-15,0.173313669984081)
link_15.Initialize(body_56,body_52,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.93696234720134,0.061705575669297,-1.73071868567941)
cB = chrono.ChVectorD(1.91824988033514,0.0617055756685828,-1.73401164540918)
dA = chrono.ChVectorD(0.984866677168361,-8.39606162372775e-15,0.173313669984081)
dB = chrono.ChVectorD(0.984866677168361,-8.40993941153556e-15,0.173313669984081)
link_16.Initialize(body_56,body_52,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_54 , SW name: robo_leg_link-11/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.05179352503079,0.0952055756692956,-1.72548771078946)
cB = chrono.ChVectorD(2.02106501134261,0.0952055756686297,-1.55087084892758)
dA = chrono.ChVectorD(0.984866677168361,-7.67441665772139e-15,0.173313669984081)
dB = chrono.ChVectorD(0.984866677168361,-8.34055047249649e-15,0.173313669984081)
link_17.Initialize(body_56,body_54,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.05179352503079,0.0952055756692956,-1.72548771078946)
dA = chrono.ChVectorD(0.984866677168361,-7.67441665772139e-15,0.173313669984081)
cB = chrono.ChVectorD(2.02106501134261,0.0952055756686297,-1.55087084892758)
dB = chrono.ChVectorD(0.984866677168361,-8.34055047249649e-15,0.173313669984081)
link_18.Initialize(body_56,body_54,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-11/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(2.05179352503079,0.0952055756692956,-1.72548771078946)
cB = chrono.ChVectorD(2.06200170019286,0.0282055756685811,-1.78349635807475)
dA = chrono.ChVectorD(0.984866677168361,-7.67441665772139e-15,0.173313669984081)
dB = chrono.ChVectorD(0.984866677168361,-8.40993941153556e-15,0.173313669984081)
link_19.Initialize(body_56,body_52,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(2.05179352503079,0.0952055756692956,-1.72548771078946)
dA = chrono.ChVectorD(0.984866677168361,-7.67441665772139e-15,0.173313669984081)
cB = chrono.ChVectorD(2.06200170019286,0.0282055756685811,-1.78349635807475)
dB = chrono.ChVectorD(0.984866677168361,-8.40993941153556e-15,0.173313669984081)
link_20.Initialize(body_56,body_52,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-11/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95527659066829,0.0952055756692963,-1.7424724504479)
cB = chrono.ChVectorD(1.96548476583036,0.028205575668582,-1.80048109773319)
dA = chrono.ChVectorD(-0.984866677168361,8.39606162372775e-15,-0.173313669984081)
dB = chrono.ChVectorD(-0.984866677168361,9.17321774096536e-15,-0.173313669984081)
link_21.Initialize(body_56,body_52,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95527659066829,0.0952055756692963,-1.7424724504479)
dA = chrono.ChVectorD(-0.984866677168361,8.39606162372775e-15,-0.173313669984081)
cB = chrono.ChVectorD(1.96548476583036,0.028205575668582,-1.80048109773319)
dB = chrono.ChVectorD(-0.984866677168361,9.17321774096536e-15,-0.173313669984081)
link_22.Initialize(body_56,body_52,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_54 , SW name: robo_leg_link-11/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95527659066829,0.0952055756692963,-1.7424724504479)
cB = chrono.ChVectorD(1.92454807698011,0.0952055756686305,-1.56785558858602)
dA = chrono.ChVectorD(-0.984866677168361,8.39606162372775e-15,-0.173313669984081)
dB = chrono.ChVectorD(-0.984866677168361,7.57727214306669e-15,-0.173313669984081)
link_23.Initialize(body_56,body_54,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95527659066829,0.0952055756692963,-1.7424724504479)
dA = chrono.ChVectorD(-0.984866677168361,8.39606162372775e-15,-0.173313669984081)
cB = chrono.ChVectorD(1.92454807698011,0.0952055756686305,-1.56785558858602)
dB = chrono.ChVectorD(-0.984866677168361,7.57727214306669e-15,-0.173313669984081)
link_24.Initialize(body_56,body_54,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_59 , SW name: robo_leg_link-12/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.91198886781665,0.034205575104295,-1.61247803727323)
dA = chrono.ChVectorD(-0.979814410199827,1.48318857196017e-16,-0.199909283338133)
cB = chrono.ChVectorD(1.90415035254532,0.0342055756532745,-1.61407731160497)
dB = chrono.ChVectorD(0.979814410199825,3.12336961849624e-15,0.199909283338143)
link_1.SetFlipped(True)
link_1.Initialize(body_59,body_58,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.91198886781665,0.034205575104295,-1.61247803727323)
cB = chrono.ChVectorD(1.90415035254532,0.0342055756532745,-1.61407731160497)
dA = chrono.ChVectorD(-0.979814410199827,1.48318857196017e-16,-0.199909283338133)
dB = chrono.ChVectorD(0.979814410199825,3.12336961849624e-15,0.199909283338143)
link_2.Initialize(body_59,body_58,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.87168412712673,0.0282055756532748,-1.59356863166039)
cB = chrono.ChVectorD(1.9054065289065,0.0282055756532614,-1.61586221971174)
dA = chrono.ChVectorD(1.91686944095437e-15,1,-7.43849426498855e-15)
dB = chrono.ChVectorD(-2.61214661012588e-14,-1,-6.03683769639929e-15)
link_3.Initialize(body_58,body_61,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.87168412712673,0.0282055756532748,-1.59356863166039)
dA = chrono.ChVectorD(1.91686944095437e-15,1,-7.43849426498855e-15)
cB = chrono.ChVectorD(1.9054065289065,0.0282055756532614,-1.61586221971174)
dB = chrono.ChVectorD(-2.61214661012588e-14,-1,-6.03683769639929e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.88571314725031,0.0805155756532946,-1.59653293654397)
dA = chrono.ChVectorD(-0.199909283338133,7.42461647718073e-15,0.979814410199827)
cB = chrono.ChVectorD(1.89328520270926,0.0384055756532619,-1.64611632208644)
dB = chrono.ChVectorD(0.199909283338133,7.21644966006352e-16,-0.979814410199827)
link_5.SetFlipped(True)
link_5.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_59 , SW name: robo_leg_link-12/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.90415035253505,0.0342055751042949,-1.61407731153993)
cB = chrono.ChVectorD(1.90375033406936,0.0322045756532746,-1.61211670297016)
dA = chrono.ChVectorD(-0.979814410199827,1.48318857196017e-16,-0.199909283338133)
dB = chrono.ChVectorD(0.979814410199825,3.12336961849624e-15,0.199909283338143)
link_6.Initialize(body_59,body_58,False,cA,cB,dB)
link_6.SetDistance(0)
link_6.SetName("Coincident7")
exported_items.append(link_6)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.90415035253505,0.0342055751042949,-1.61407731153993)
dA = chrono.ChVectorD(-0.979814410199827,1.48318857196017e-16,-0.199909283338133)
cB = chrono.ChVectorD(1.90375033406936,0.0322045756532746,-1.61211670297016)
dB = chrono.ChVectorD(0.979814410199825,3.12336961849624e-15,0.199909283338143)
link_7.SetFlipped(True)
link_7.Initialize(body_59,body_58,False,cA,cB,dA,dB)
link_7.SetName("Coincident7")
exported_items.append(link_7)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.90540652890652,0.0961555756533051,-1.61586221971173)
dA = chrono.ChVectorD(-2.63938176869871e-14,-1,-5.93969318174459e-15)
cB = chrono.ChVectorD(1.9054065289065,0.0282055756532614,-1.61586221971174)
dB = chrono.ChVectorD(2.61214661012588e-14,1,6.03683769639929e-15)
link_8.SetFlipped(True)
link_8.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateGeneric()
link_9.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.90540652890652,0.0961555756533051,-1.61586221971173)
cB = chrono.ChVectorD(1.9054065289065,0.0282055756532614,-1.61586221971174)
dA = chrono.ChVectorD(-2.63938176869871e-14,-1,-5.93969318174459e-15)
dB = chrono.ChVectorD(2.61214661012588e-14,1,6.03683769639929e-15)
link_9.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_9.SetName("Concentric4")
exported_items.append(link_9)


# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.86770720200649,0.096155575653306,-1.59748205509372)
dA = chrono.ChVectorD(-2.63938176869871e-14,-1,-5.93969318174459e-15)
cB = chrono.ChVectorD(1.9054065289065,0.0952055756532614,-1.61586221971174)
dB = chrono.ChVectorD(2.61214661012588e-14,1,6.03683769639929e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_10.SetName("Parallel4")
exported_items.append(link_10)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.90540652890651,0.0282055756532746,-1.61586221971173)
dA = chrono.ChVectorD(1.91686944095437e-15,1,-7.43849426498855e-15)
cB = chrono.ChVectorD(1.9054065289065,0.0282055756532614,-1.61586221971174)
dB = chrono.ChVectorD(2.61214661012588e-14,1,6.03683769639929e-15)
link_11.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_11.SetName("Concentric5")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.90540652890651,0.0282055756532746,-1.61586221971173)
cB = chrono.ChVectorD(1.9054065289065,0.0282055756532614,-1.61586221971174)
dA = chrono.ChVectorD(1.91686944095437e-15,1,-7.43849426498855e-15)
dB = chrono.ChVectorD(2.61214661012588e-14,1,6.03683769639929e-15)
link_12.Initialize(body_58,body_61,False,cA,cB,dA,dB)
link_12.SetName("Concentric5")
exported_items.append(link_12)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-12/link2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.87484788179009,0.0617055756532626,-1.68843612401911)
dA = chrono.ChVectorD(-0.199909283338133,-7.21644966006352e-16,0.979814410199827)
cB = chrono.ChVectorD(1.87864615817333,0.0617055756532794,-1.70705259781231)
dB = chrono.ChVectorD(-0.199909283338133,-7.35522753814166e-16,0.979814410199827)
link_13.Initialize(body_61,body_57,False,cA,cB,dA,dB)
link_13.SetName("Concentric6")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.87484788179009,0.0617055756532626,-1.68843612401911)
cB = chrono.ChVectorD(1.87864615817333,0.0617055756532794,-1.70705259781231)
dA = chrono.ChVectorD(-0.199909283338133,-7.21644966006352e-16,0.979814410199827)
dB = chrono.ChVectorD(-0.199909283338133,-7.35522753814166e-16,0.979814410199827)
link_14.Initialize(body_61,body_57,False,cA,cB,dA,dB)
link_14.SetName("Concentric6")
exported_items.append(link_14)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-12/link2-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.96195338285686,0.0617055756532603,-1.67066418873035)
dA = chrono.ChVectorD(-0.199909283338133,-7.21644966006352e-16,0.979814410199827)
cB = chrono.ChVectorD(1.96575165924026,0.0617055756532802,-1.68928066252435)
dB = chrono.ChVectorD(-0.199909283338133,-7.07767178198537e-16,0.979814410199827)
link_15.Initialize(body_61,body_60,False,cA,cB,dA,dB)
link_15.SetName("Concentric7")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.96195338285686,0.0617055756532603,-1.67066418873035)
cB = chrono.ChVectorD(1.96575165924026,0.0617055756532802,-1.68928066252435)
dA = chrono.ChVectorD(-0.199909283338133,-7.21644966006352e-16,0.979814410199827)
dB = chrono.ChVectorD(-0.199909283338133,-7.07767178198537e-16,0.979814410199827)
link_16.Initialize(body_61,body_60,False,cA,cB,dA,dB)
link_16.SetName("Concentric7")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-12/link2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95361598710676,0.0952055756532598,-1.55601668403833)
cB = chrono.ChVectorD(1.77989489217814,0.0952055756532793,-1.59146059997359)
dA = chrono.ChVectorD(-0.199909283338133,0,0.979814410199827)
dB = chrono.ChVectorD(-0.199909283338133,-7.35522753814166e-16,0.979814410199827)
link_17.Initialize(body_61,body_57,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95361598710676,0.0952055756532598,-1.55601668403833)
dA = chrono.ChVectorD(-0.199909283338133,0,0.979814410199827)
cB = chrono.ChVectorD(1.77989489217814,0.0952055756532793,-1.59146059997359)
dB = chrono.ChVectorD(-0.199909283338133,-7.35522753814166e-16,0.979814410199827)
link_18.Initialize(body_61,body_57,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-12/link2-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.95361598710676,0.0952055756532598,-1.55601668403833)
cB = chrono.ChVectorD(2.0113270558675,0.0282055756532801,-1.54424202724992)
dA = chrono.ChVectorD(-0.199909283338133,0,0.979814410199827)
dB = chrono.ChVectorD(-0.199909283338133,-7.07767178198537e-16,0.979814410199827)
link_19.Initialize(body_61,body_60,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident10")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.95361598710676,0.0952055756532598,-1.55601668403833)
dA = chrono.ChVectorD(-0.199909283338133,0,0.979814410199827)
cB = chrono.ChVectorD(2.0113270558675,0.0282055756532801,-1.54424202724992)
dB = chrono.ChVectorD(-0.199909283338133,-7.07767178198537e-16,0.979814410199827)
link_20.Initialize(body_61,body_60,False,cA,cB,dA,dB)
link_20.SetName("Coincident10")
exported_items.append(link_20)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-12/link2-1 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.9732070968739,0.0952055756532598,-1.65203849623791)
cB = chrono.ChVectorD(2.03091816563464,0.0282055756532802,-1.64026383944951)
dA = chrono.ChVectorD(0.199909283338133,7.21644966006352e-16,-0.979814410199827)
dB = chrono.ChVectorD(0.199909283338133,1.4432899320127e-15,-0.979814410199827)
link_21.Initialize(body_61,body_60,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident11")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.9732070968739,0.0952055756532598,-1.65203849623791)
dA = chrono.ChVectorD(0.199909283338133,7.21644966006352e-16,-0.979814410199827)
cB = chrono.ChVectorD(2.03091816563464,0.0282055756532802,-1.64026383944951)
dB = chrono.ChVectorD(0.199909283338133,1.4432899320127e-15,-0.979814410199827)
link_22.Initialize(body_61,body_60,False,cA,cB,dA,dB)
link_22.SetName("Coincident11")
exported_items.append(link_22)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-12/link2-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.9732070968739,0.0952055756532598,-1.65203849623791)
cB = chrono.ChVectorD(1.79948600194528,0.0952055756532793,-1.68748241217317)
dA = chrono.ChVectorD(0.199909283338133,7.21644966006352e-16,-0.979814410199827)
dB = chrono.ChVectorD(0.199909283338133,-2.77555756156289e-17,-0.979814410199827)
link_23.Initialize(body_61,body_57,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident12")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.9732070968739,0.0952055756532598,-1.65203849623791)
dA = chrono.ChVectorD(0.199909283338133,7.21644966006352e-16,-0.979814410199827)
cB = chrono.ChVectorD(1.79948600194528,0.0952055756532793,-1.68748241217317)
dB = chrono.ChVectorD(0.199909283338133,-2.77555756156289e-17,-0.979814410199827)
link_24.Initialize(body_61,body_57,False,cA,cB,dA,dB)
link_24.SetName("Coincident12")
exported_items.append(link_24)

