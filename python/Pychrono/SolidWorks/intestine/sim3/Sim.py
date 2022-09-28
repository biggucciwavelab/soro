# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\Amin\Documents\SolidCAD\sim3\SimAssem2.SLDASM


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
body_1.SetName('robo_leg_link-5/leg-1')
body_1.SetPos(chrono.ChVectorD(0.359772792995903,0.0169999999999904,-0.551811124167946))
body_1.SetRot(chrono.ChQuaternionD(0.0505369781675448,0.0573266968109489,0.997071321921488,0.00290562767320605))
body_1.SetMass(0.00100053137131845)
body_1.SetInertiaXX(chrono.ChVectorD(1.97539468378929e-08,5.0793008502776e-08,5.90456981042962e-08))
body_1.SetInertiaXY(chrono.ChVectorD(3.6331013828816e-09,4.05114634921704e-09,-4.6776240555921e-10))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('robo_leg_link-5/link3-2')
body_2.SetPos(chrono.ChVectorD(0.355275467987794,0.0110000000000038,-0.494896162243881))
body_2.SetRot(chrono.ChQuaternionD(0.0840445019684549,1.10511234768232e-14,0.996462002129973,9.49999697747701e-15))
body_2.SetMass(0.0328351485462318)
body_2.SetInertiaXX(chrono.ChVectorD(2.94909194291117e-06,2.05022207359737e-05,2.11496364697425e-05))
body_2.SetInertiaXY(chrono.ChVectorD(1.64209656252081e-11,3.18407453560821e-06,-2.21036950043895e-12))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_2.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('robo_leg_link-5/link3-3')
body_3.SetPos(chrono.ChVectorD(0.370959732617845,0.0110000000000061,-0.62220486045802))
body_3.SetRot(chrono.ChQuaternionD(0.989533778547395,8.77324637494436e-15,-0.144301424503413,-1.17525914072589e-14))
body_3.SetMass(0.0328351485462318)
body_3.SetInertiaXX(chrono.ChVectorD(3.9807587924776e-06,2.05022207359737e-05,2.01179696201761e-05))
body_3.SetInertiaXY(chrono.ChVectorD(-1.60327358857912e-11,5.27739448896935e-06,4.18153387933261e-12))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_3.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('robo_leg_link-5/single_bot1-1')
body_4.SetPos(chrono.ChVectorD(0.46855801934615,-0.114692616844618,-0.531565750378))
body_4.SetRot(chrono.ChQuaternionD(0.600888874221706,-0.638424130378486,-0.362655177526013,-0.315956346352817))
body_4.SetMass(0.08424283226466)
body_4.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_4.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

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
marker_4_1.SetName('My_marker')
body_4.AddMarker(marker_4_1)
marker_4_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.359705048570072,0.0170000000000254,-0.551144557822582),chrono.ChQuaternionD(0.998515550290243,-0.00101910090552357,-0.0506101794881887,0.0201063918715023)))

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_4.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_4.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_4.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_4.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('robo_leg_link-5/link1-1')
body_5.SetPos(chrono.ChVectorD(0.362483060860357,0.0780000000000065,-0.558698382265537))
body_5.SetRot(chrono.ChQuaternionD(-1.32286135548322e-14,-0.050620438900524,2.67142957554253e-15,0.998717963774317))
body_5.SetMass(0.074045467558508)
body_5.SetInertiaXX(chrono.ChVectorD(3.41522381649532e-05,9.69039180113677e-05,8.0732953632542e-05))
body_5.SetInertiaXY(chrono.ChVectorD(1.17006998456514e-07,-4.78271401658578e-06,1.18916473710397e-08))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_5.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('robo_leg_link-5/link2-4')
body_6.SetPos(chrono.ChVectorD(0.366803983661598,0.0110000000000063,-0.601213749284849))
body_6.SetRot(chrono.ChQuaternionD(0.0506206186978415,-1.32425092781413e-14,0.998717954661199,-2.7009577554446e-15))
body_6.SetMass(0.0328349916079305)
body_6.SetInertiaXX(chrono.ChVectorD(2.60435632901987e-06,2.05032007850367e-05,2.14935091538843e-05))
body_6.SetInertiaXY(chrono.ChVectorD(1.65342888256643e-11,1.93978280817615e-06,-1.10620316253129e-12))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_6.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_6.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_6.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_6.GetCollisionModel().BuildModel()
body_6.SetCollide(True)

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRef()
body_7.SetName('robo_leg_link-5/link2-3')
body_7.SetPos(chrono.ChVectorD(0.358162139216762,0.0110000000000009,-0.516183015128578))
body_7.SetRot(chrono.ChQuaternionD(0.0506205646296153,2.08434038433429e-17,0.998717957401678,8.1636665053093e-17))
body_7.SetMass(0.0328349916079305)
body_7.SetInertiaXX(chrono.ChVectorD(2.60435590895909e-06,2.05032007850367e-05,2.14935095739451e-05))
body_7.SetInertiaXY(chrono.ChVectorD(1.65342894167387e-11,1.93978076294806e-06,-1.10620142794686e-12))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_7.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_7.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_7.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_7.GetCollisionModel().BuildModel()
body_7.SetCollide(True)

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRef()
body_8.SetName('robo_leg_link-10/leg-1')
body_8.SetPos(chrono.ChVectorD(0.910309791149956,0.017000000000001,-0.511736456362463))
body_8.SetRot(chrono.ChQuaternionD(0.799983179865877,0.00187026554333808,-0.60001432992902,-0.00249357540629893))
body_8.SetMass(0.00100053137131845)
body_8.SetInertiaXX(chrono.ChVectorD(5.6282558893255e-08,5.12127725232862e-08,2.20973220284238e-08))
body_8.SetInertiaXY(chrono.ChVectorD(3.05743077779347e-11,1.08955126371282e-08,7.14648467221568e-11))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_8.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_8.GetCollisionModel().BuildModel()
body_8.SetCollide(True)

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRef()
body_9.SetName('robo_leg_link-10/link2-3')
body_9.SetPos(chrono.ChVectorD(0.943934344180763,0.0110000000000008,-0.523625350225972))
body_9.SetRot(chrono.ChQuaternionD(0.799987022019064,8.02322623298911e-17,-0.600017303584713,-7.37269437626026e-17))
body_9.SetMass(0.0328349916079305)
body_9.SetInertiaXX(chrono.ChVectorD(2.01792794335167e-05,2.05032007850367e-05,3.91858604938753e-06))
body_9.SetInertiaXY(chrono.ChVectorD(-5.1849327793431e-12,5.1826869112461e-06,1.57392147258478e-11))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_9.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_9.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(True)

exported_items.append(body_9)



# Rigid body part
body_10= chrono.ChBodyAuxRef()
body_10.SetName('robo_leg_link-10/link3-2')
body_10.SetPos(chrono.ChVectorD(0.953433205426507,0.0110000000000008,-0.537349707068239))
body_10.SetRot(chrono.ChQuaternionD(0.999274017403076,9.87043656398167e-17,0.0380977445935581,-3.32748697803961e-17))
body_10.SetMass(0.0328351485462318)
body_10.SetInertiaXX(chrono.ChVectorD(2.5199226624826e-06,2.05022207359738e-05,2.15788057501711e-05))
body_10.SetInertiaXY(chrono.ChVectorD(-1.6467645007556e-11,-1.46390907472311e-06,-1.83043741518468e-12))
body_10.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_10.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_10.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_10.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_10.GetCollisionModel().BuildModel()
body_10.SetCollide(True)

exported_items.append(body_10)



# Rigid body part
body_11= chrono.ChBodyAuxRef()
body_11.SetName('robo_leg_link-10/link3-3')
body_11.SetPos(chrono.ChVectorD(0.841100610742247,0.0109999999999941,-0.494246197684279))
body_11.SetRot(chrono.ChQuaternionD(0.621572270290953,1.54720934588908e-14,0.783356823424262,2.28068736428582e-15))
body_11.SetMass(0.0328351485462318)
body_11.SetInertiaXX(chrono.ChVectorD(2.06943948908256e-05,2.05022207359738e-05,3.40433352182811e-06))
body_11.SetInertiaXY(chrono.ChVectorD(4.32016153440059e-12,4.26810618198242e-06,-1.59959385562205e-11))
body_11.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_11.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_11.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_11.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_11.GetCollisionModel().BuildModel()
body_11.SetCollide(True)

exported_items.append(body_11)



# Rigid body part
body_12= chrono.ChBodyAuxRef()
body_12.SetName('robo_leg_link-10/link2-4')
body_12.SetPos(chrono.ChVectorD(0.86188331812208,0.0110000000000008,-0.499697618422133))
body_12.SetRot(chrono.ChQuaternionD(0.799987668180422,6.07162575866058e-17,-0.600016442074091,-7.15584464413568e-17))
body_12.SetMass(0.0328349916079305)
body_12.SetInertiaXX(chrono.ChVectorD(2.0179257108389e-05,2.05032007850367e-05,3.91860837451523e-06))
body_12.SetInertiaXY(chrono.ChVectorD(-5.18496667890772e-12,5.1827219336522e-06,1.57392035589747e-11))
body_12.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_12.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_12.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_12.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_12.GetCollisionModel().BuildModel()
body_12.SetCollide(True)

exported_items.append(body_12)



# Rigid body part
body_13= chrono.ChBodyAuxRef()
body_13.SetName('robo_leg_link-10/link1-1')
body_13.SetPos(chrono.ChVectorD(0.902908827911348,0.0780000000000008,-0.511661495434621))
body_13.SetRot(chrono.ChQuaternionD(-6.50531820853038e-17,0.799987066137092,-6.61374017867255e-17,0.600017244763322))
body_13.SetMass(0.074045467558508)
body_13.SetInertiaXX(chrono.ChVectorD(7.7491509216241e-05,9.69039180113677e-05,3.73936825812542e-05))
body_13.SetInertiaXY(chrono.ChVectorD(-3.2925857210409e-08,-1.27810872589228e-05,-1.12906761939089e-07))
body_13.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_13.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_13)



# Rigid body part
body_14= chrono.ChBodyAuxRef()
body_14.SetName('robo_leg_link-10/single_bot1-1')
body_14.SetPos(chrono.ChVectorD(0.888214073735505,-0.114692616844647,-0.620160991887911))
body_14.SetRot(chrono.ChQuaternionD(0.636510526200655,-0.618754481073665,0.295703511960202,0.352925877772412))
body_14.SetMass(0.08424283226466)
body_14.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_14.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_14.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_14.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_14_1 =chrono.ChMarker()
marker_14_1.SetName('My_marker')
body_14.AddMarker(marker_14_1)
marker_14_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.910952999237218,0.017000000000001,-0.511924028632485),chrono.ChQuaternionD(0.599895637277107,0.0161055012799361,0.79982493010355,0.0120796684253876)))

# Collision shapes 
body_14.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_14.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_14.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_14.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_14.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_14.GetCollisionModel().BuildModel()
body_14.SetCollide(True)

exported_items.append(body_14)



# Rigid body part
body_15= chrono.ChBodyAuxRef()
body_15.SetName('robo_leg_link-2/single_bot1-1')
body_15.SetPos(chrono.ChVectorD(0.710489887568286,-0.114692616844647,-0.648630707375111))
body_15.SetRot(chrono.ChQuaternionD(-0.268596549525361,0.326516839267913,0.648415075806309,0.633088095548053))
body_15.SetMass(0.08424283226466)
body_15.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_15.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_15.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_15.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_15_1 =chrono.ChMarker()
marker_15_1.SetName('My_marker')
body_15.AddMarker(marker_15_1)
marker_15_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.696953189491262,0.0170000000000011,-0.758398902918721),chrono.ChQuaternionD(0.773813629929823,-0.0127481465044026,-0.63309332690693,0.0155817303739865)))

# Collision shapes 
body_15.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_15.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_15.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_15.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_15.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_15.GetCollisionModel().BuildModel()
body_15.SetCollide(True)

exported_items.append(body_15)



# Rigid body part
body_16= chrono.ChBodyAuxRef()
body_16.SetName('robo_leg_link-2/leg-1')
body_16.SetPos(chrono.ChVectorD(0.697609916635232,0.0170000000000011,-0.758531603553132))
body_16.SetRot(chrono.ChQuaternionD(0.631929053822461,-0.0494280599907456,0.772390568994837,-0.0404394207232424))
body_16.SetMass(0.00100053137131845)
body_16.SetInertiaXX(chrono.ChVectorD(5.77614977920465e-08,5.13209360711805e-08,2.05102195817381e-08))
body_16.SetInertiaXY(chrono.ChVectorD(8.42353789794299e-10,7.80662664509594e-09,9.99612059552689e-10))
body_16.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_16.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_16.GetCollisionModel().BuildModel()
body_16.SetCollide(True)

exported_items.append(body_16)



# Rigid body part
body_17= chrono.ChBodyAuxRef()
body_17.SetName('robo_leg_link-2/link1-1')
body_17.SetPos(chrono.ChVectorD(0.704990870212469,0.0780000000000009,-0.7579826041992))
body_17.SetRot(chrono.ChQuaternionD(6.27572468982869e-17,-0.633221664042994,-5.15505956664499e-17,0.773970493098169))
body_17.SetMass(0.074045467558508)
body_17.SetInertiaXX(chrono.ChVectorD(7.9353255525661e-05,9.69039180113677e-05,3.55319362718343e-05))
body_17.SetInertiaXY(chrono.ChVectorD(2.32938596936635e-08,-9.23244883257486e-06,1.15279855417942e-07))
body_17.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_17.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_17)



# Rigid body part
body_18= chrono.ChBodyAuxRef()
body_18.SetName('robo_leg_link-2/link2-3')
body_18.SetPos(chrono.ChVectorD(0.663103073325775,0.0110000000000009,-0.749518605225917))
body_18.SetRot(chrono.ChQuaternionD(0.633221629034332,6.90507966319165e-17,0.773970521740399,6.90141201044431e-17))
body_18.SetMass(0.0328349916079305)
body_18.SetInertiaXX(chrono.ChVectorD(2.09342011216544e-05,2.05032007850367e-05,3.16366436124975e-06))
body_18.SetInertiaXY(chrono.ChVectorD(3.84011835265378e-12,3.7436328298784e-06,-1.61201705683154e-11))
body_18.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_18.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_18.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_18.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_18.GetCollisionModel().BuildModel()
body_18.SetCollide(True)

exported_items.append(body_18)



# Rigid body part
body_19= chrono.ChBodyAuxRef()
body_19.SetName('robo_leg_link-2/link3-2')
body_19.SetPos(chrono.ChVectorD(0.641849629998111,0.0110000000000009,-0.746623026446583))
body_19.SetRot(chrono.ChQuaternionD(0.681405256637797,5.60076637209976e-17,0.731906330227017,-1.78206202748629e-17))
body_19.SetMass(0.0328351485462318)
body_19.SetInertiaXX(chrono.ChVectorD(2.15923635162212e-05,2.05022207359738e-05,2.50636489643247e-06))
body_19.SetInertiaXY(chrono.ChVectorD(1.75171064160807e-12,1.37275274101233e-06,-1.64762053604447e-11))
body_19.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_19.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_19.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_19.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_19.GetCollisionModel().BuildModel()
body_19.SetCollide(True)

exported_items.append(body_19)



# Rigid body part
body_20= chrono.ChBodyAuxRef()
body_20.SetName('robo_leg_link-2/link3-3')
body_20.SetPos(chrono.ChVectorD(0.767985175474771,0.0110000000000011,-0.770503062535919))
body_20.SetRot(chrono.ChQuaternionD(0.767917313835044,1.37121162779787e-15,-0.640548982601932,-1.45840894053757e-14))
body_20.SetMass(0.0328351485462318)
body_20.SetInertiaXX(chrono.ChVectorD(2.10700401169986e-05,2.05022207359738e-05,3.02868829565506e-06))
body_20.SetInertiaXY(chrono.ChVectorD(-3.53263016034004e-12,3.40304158529205e-06,1.61880934542396e-11))
body_20.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_20.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_20.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_20.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_20.GetCollisionModel().BuildModel()
body_20.SetCollide(True)

exported_items.append(body_20)



# Rigid body part
body_21= chrono.ChBodyAuxRef()
body_21.SetName('robo_leg_link-2/link2-4')
body_21.SetPos(chrono.ChVectorD(0.746878667051964,0.0110000000000009,-0.766446603406065))
body_21.SetRot(chrono.ChQuaternionD(0.633221620453234,6.5748486909913e-17,0.773970528760999,6.5748486909913e-17))
body_21.SetMass(0.0328349916079305)
body_21.SetInertiaXX(chrono.ChVectorD(2.09342009556301e-05,2.05032007850367e-05,3.16366452727408e-06))
body_21.SetInertiaXY(chrono.ChVectorD(3.84011871014339e-12,3.74363322392629e-06,-1.61201704833089e-11))
body_21.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_21.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_21.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_21.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_21.GetCollisionModel().BuildModel()
body_21.SetCollide(True)

exported_items.append(body_21)



# Rigid body part
body_22= chrono.ChBodyAuxRef()
body_22.SetName('robo_leg_link-3/link1-1')
body_22.SetPos(chrono.ChVectorD(0.557135210260595,0.0780000000000103,-0.734416899622841))
body_22.SetRot(chrono.ChQuaternionD(-1.21866743955621e-14,-0.638276996019986,-5.69672945117991e-15,0.769806778582589))
body_22.SetMass(0.074045467558508)
body_22.SetInertiaXX(chrono.ChVectorD(7.95875732981614e-05,9.69039180113677e-05,3.52976184993339e-05))
body_22.SetInertiaXY(chrono.ChVectorD(2.17819047769506e-08,-8.65535052508081e-06,1.15575073390786e-07))
body_22.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_22.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_22)



# Rigid body part
body_23= chrono.ChBodyAuxRef()
body_23.SetName('robo_leg_link-3/link2-3')
body_23.SetPos(chrono.ChVectorD(0.515140143093589,0.0110000000000009,-0.726502284011248))
body_23.SetRot(chrono.ChQuaternionD(0.638277079447929,6.52277275246256e-17,0.769806709409201,3.53316857425056e-17))
body_23.SetMass(0.0328349916079305)
body_23.SetInertiaXX(chrono.ChVectorD(2.10292156858931e-05,2.05032007850367e-05,3.06864979701106e-06))
body_23.SetInertiaXY(chrono.ChVectorD(3.62863901461136e-12,3.5096010016736e-06,-1.61690873899992e-11))
body_23.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_23.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_23.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_23.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_23.GetCollisionModel().BuildModel()
body_23.SetCollide(True)

exported_items.append(body_23)



# Rigid body part
body_24= chrono.ChBodyAuxRef()
body_24.SetName('robo_leg_link-3/link2-4')
body_24.SetPos(chrono.ChVectorD(0.599130276990599,0.0110000000000009,-0.742331517553176))
body_24.SetRot(chrono.ChQuaternionD(0.638276994939371,4.11575106798835e-17,0.769806779478569,2.64601837657002e-17))
body_24.SetMass(0.0328349916079305)
body_24.SetInertiaXX(chrono.ChVectorD(2.10292141447713e-05,2.05032007850367e-05,3.06865133813286e-06))
body_24.SetInertiaXY(chrono.ChVectorD(3.62864256536028e-12,3.50960494505658e-06,-1.61690865930336e-11))
body_24.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_24.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_24.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_24.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_24.GetCollisionModel().BuildModel()
body_24.SetCollide(True)

exported_items.append(body_24)



# Rigid body part
body_25= chrono.ChBodyAuxRef()
body_25.SetName('robo_leg_link-3/link3-2')
body_25.SetPos(chrono.ChVectorD(0.493845249209014,0.0110000000000009,-0.723984798826541))
body_25.SetRot(chrono.ChQuaternionD(0.689547668629229,7.52320585471088e-17,0.7242403003755,7.19273258905785e-17))
body_25.SetMass(0.0328351485462318)
body_25.SetInertiaXX(chrono.ChVectorD(2.1644204536424e-05,2.05022207359738e-05,2.45452387622967e-06))
body_25.SetInertiaXY(chrono.ChVectorD(1.38278359053554e-12,9.44629534019633e-07,-1.65112610883912e-11))
body_25.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_25.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_25.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_25.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_25.GetCollisionModel().BuildModel()
body_25.SetCollide(True)

exported_items.append(body_25)



# Rigid body part
body_26= chrono.ChBodyAuxRef()
body_26.SetName('robo_leg_link-3/leg-1')
body_26.SetPos(chrono.ChVectorD(0.549762080751647,0.0170000000000726,-0.73506252896247))
body_26.SetRot(chrono.ChQuaternionD(0.638201988667749,0.011801339131596,0.769716314460334,0.0097849531849987))
body_26.SetMass(0.00100053137131845)
body_26.SetInertiaXX(chrono.ChVectorD(5.80631905819487e-08,5.12189910273642e-08,2.03104718356521e-08))
body_26.SetInertiaXY(chrono.ChVectorD(-2.09434048126217e-10,7.37478312787792e-09,-2.29820569229822e-10))
body_26.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_26.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_26.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_26.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_26.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_26.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_26.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_26.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_26.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_26.GetCollisionModel().BuildModel()
body_26.SetCollide(True)

exported_items.append(body_26)



# Rigid body part
body_27= chrono.ChBodyAuxRef()
body_27.SetName('robo_leg_link-3/link3-3')
body_27.SetPos(chrono.ChVectorD(0.620410696764696,0.0110000000000036,-0.745088936854801))
body_27.SetRot(chrono.ChQuaternionD(0.731906330227016,5.38022712448183e-16,-0.681405256637797,-1.46320846488937e-14))
body_27.SetMass(0.0328351485462318)
body_27.SetInertiaXX(chrono.ChVectorD(2.15923635162212e-05,2.05022207359737e-05,2.50636489643247e-06))
body_27.SetInertiaXY(chrono.ChVectorD(-1.75171064335754e-12,1.37275274101233e-06,1.64762057334798e-11))
body_27.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_27.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_27.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_27.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_27.GetCollisionModel().BuildModel()
body_27.SetCollide(True)

exported_items.append(body_27)



# Rigid body part
body_28= chrono.ChBodyAuxRef()
body_28.SetName('robo_leg_link-3/single_bot1-1')
body_28.SetPos(chrono.ChVectorD(0.561201444714886,-0.114692616844636,-0.6250023564018))
body_28.SetRot(chrono.ChQuaternionD(-0.264344168308022,0.322363595815795,0.650160270726675,0.635212952593748))
body_28.SetMass(0.08424283226466)
body_28.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_28.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_28.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_28.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_28_1 =chrono.ChMarker()
marker_28_1.SetName('My_marker')
body_28.AddMarker(marker_28_1)
marker_28_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.549103671807726,0.0170000000000103,-0.734938441644163),chrono.ChQuaternionD(0.769650759288082,-0.0128499214693586,-0.638147634303014,0.0154979056319515)))

# Collision shapes 
body_28.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_28.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_28.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_28.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_28.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_28.GetCollisionModel().BuildModel()
body_28.SetCollide(True)

exported_items.append(body_28)



# Rigid body part
body_29= chrono.ChBodyAuxRef()
body_29.SetName('robo_leg_link-7/link3-2')
body_29.SetPos(chrono.ChVectorD(0.528944609166121,0.0109999999999987,-0.420458649452389))
body_29.SetRot(chrono.ChQuaternionD(0.775166428086816,1.69318441602079e-15,-0.63175708050415,-1.47506459995192e-14))
body_29.SetMass(0.0328351485462318)
body_29.SetInertiaXX(chrono.ChVectorD(2.09056126893458e-05,2.05022207359738e-05,3.19311572330785e-06))
body_29.SetInertiaXY(chrono.ChVectorD(-3.90061116488095e-12,3.81053129151632e-06,1.61033871628011e-11))
body_29.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_29.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_29.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_29.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_29.GetCollisionModel().BuildModel()
body_29.SetCollide(True)

exported_items.append(body_29)



# Rigid body part
body_30= chrono.ChBodyAuxRef()
body_30.SetName('robo_leg_link-7/single_bot1-1')
body_30.SetPos(chrono.ChVectorD(0.439985415210346,-0.114692616844652,-0.501695616326519))
body_30.SetRot(chrono.ChQuaternionD(0.614287689496159,-0.592575847646965,0.339465537990679,0.395307029833331))
body_30.SetMass(0.08424283226466)
body_30.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_30.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_30.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_30.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_30_1 =chrono.ChMarker()
marker_30_1.SetName('My_marker')
body_30.AddMarker(marker_30_1)
marker_30_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.477594358732605,0.0169999999999965,-0.397686644045912),chrono.ChQuaternionD(0.542529787435326,0.0169103557062613,0.839795287069792,0.0109245334286135)))

# Collision shapes 
body_30.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_30.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_30.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_30.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_30.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_30.GetCollisionModel().BuildModel()
body_30.SetCollide(True)

exported_items.append(body_30)



# Rigid body part
body_31= chrono.ChBodyAuxRef()
body_31.SetName('robo_leg_link-7/leg-1')
body_31.SetPos(chrono.ChVectorD(0.476983588479524,0.0170000000000234,-0.397411217652927))
body_31.SetRot(chrono.ChQuaternionD(0.839965384626025,0.000314466036538229,-0.542639674919153,-0.000486769761806237))
body_31.SetMass(0.00100053137131845)
body_31.SetInertiaXX(chrono.ChVectorD(5.26092557169663e-08,5.12125867177574e-08,2.57708110102413e-08))
body_31.SetInertiaXY(chrono.ChVectorD(1.02602561729166e-13,1.51921725920285e-08,2.09702615795503e-11))
body_31.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_31.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_31.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_31.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_31.GetCollisionModel().BuildModel()
body_31.SetCollide(True)

exported_items.append(body_31)



# Rigid body part
body_32= chrono.ChBodyAuxRef()
body_32.SetName('robo_leg_link-7/link1-1')
body_32.SetPos(chrono.ChVectorD(0.46966537712218,0.0780000000000103,-0.396305026652595))
body_32.SetRot(chrono.ChQuaternionD(-4.22158219177363e-15,0.839965525670401,1.27642791738891e-14,0.542639766037514))
body_32.SetMass(0.074045467558508)
body_32.SetInertiaXX(chrono.ChVectorD(7.31824351295804e-05,9.69039180113677e-05,4.17027566679149e-05))
body_32.SetInertiaXY(chrono.ChVectorD(-4.8347498734081e-08,-1.78205661447694e-05,-1.07212724669956e-07))
body_32.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_32.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_32)



# Rigid body part
body_33= chrono.ChBodyAuxRef()
body_33.SetName('robo_leg_link-7/link2-3')
body_33.SetPos(chrono.ChVectorD(0.508621927853471,0.0110000000000009,-0.413872438578105))
body_33.SetRot(chrono.ChQuaternionD(0.839965163056495,5.85834293952116e-17,-0.542640327336143,-6.9545609766406e-17))
body_33.SetMass(0.0328349916079305)
body_33.SetInertiaXX(chrono.ChVectorD(1.84319593022117e-05,2.05032007850367e-05,5.66590618069251e-06))
body_33.SetInertiaXY(chrono.ChVectorD(-7.32888042927187e-12,7.22632360732652e-06,1.48625004554306e-11))
body_33.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_33.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_33.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_33.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_33.GetCollisionModel().BuildModel()
body_33.SetCollide(True)

exported_items.append(body_33)



# Rigid body part
body_34= chrono.ChBodyAuxRef()
body_34.SetName('robo_leg_link-7/link2-4')
body_34.SetPos(chrono.ChVectorD(0.430708828511179,0.0110000000000095,-0.378737610025377))
body_34.SetRot(chrono.ChQuaternionD(0.839965293264801,4.23475347985718e-15,-0.542640125783725,1.27480107313584e-14))
body_34.SetMass(0.0328349916079305)
body_34.SetInertiaXX(chrono.ChVectorD(1.84319523662883e-05,2.05032007850367e-05,5.66591311661588e-06))
body_34.SetInertiaXY(chrono.ChVectorD(-7.32888737545044e-12,7.22632973383603e-06,1.48624965049115e-11))
body_34.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_34.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_34.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_34.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_34.GetCollisionModel().BuildModel()
body_34.SetCollide(True)

exported_items.append(body_34)



# Rigid body part
body_35= chrono.ChBodyAuxRef()
body_35.SetName('robo_leg_link-7/link3-3')
body_35.SetPos(chrono.ChVectorD(0.41073261005297,0.0110000000000006,-0.370873690773046))
body_35.SetRot(chrono.ChQuaternionD(0.582813563806824,1.53489597411477e-14,0.812605900694051,3.12529820853752e-15))
body_35.SetMass(0.0328351485462318)
body_35.SetInertiaXX(chrono.ChVectorD(1.97079557115713e-05,2.05022207359738e-05,4.3907727010824e-06))
body_35.SetInertiaXY(chrono.ChVectorD(5.85093081142e-12,5.85655718595822e-06,-1.55016274808771e-11))
body_35.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_35.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_35.GetCollisionModel().BuildModel()
body_35.SetCollide(True)

exported_items.append(body_35)



# Rigid body part
body_36= chrono.ChBodyAuxRef()
body_36.SetName('robo_leg_link-11/leg-1')
body_36.SetPos(chrono.ChVectorD(0.990790894097152,0.017000000000001,-0.615237357615954))
body_36.SetRot(chrono.ChQuaternionD(0.942210334404762,0.00104422978304117,-0.335007417722729,-0.00293690241173605))
body_36.SetMass(0.00100053137131845)
body_36.SetInertiaXX(chrono.ChVectorD(3.50775215933556e-08,5.12119256131344e-08,4.3303206238475e-08))
body_36.SetInertiaXY(chrono.ChVectorD(-1.03448459789502e-10,1.98480215300427e-08,1.26063571500369e-10))
body_36.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_36.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_36.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_36.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_36.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_36.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_36.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_36.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_36.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_36.GetCollisionModel().BuildModel()
body_36.SetCollide(True)

exported_items.append(body_36)



# Rigid body part
body_37= chrono.ChBodyAuxRef()
body_37.SetName('robo_leg_link-11/single_bot1-1')
body_37.SetPos(chrono.ChVectorD(0.911040146344112,-0.114692616844647,-0.691944064394432))
body_37.SetRot(chrono.ChQuaternionD(0.519796597897093,-0.485829012467265,0.471590620946218,0.52094525018974))
body_37.SetMass(0.08424283226466)
body_37.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_37.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_37.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_37.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_37_1 =chrono.ChMarker()
marker_37_1.SetName('My_marker')
body_37.AddMarker(marker_37_1)
marker_37_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.991213865791113,0.0170000000000011,-0.615756967995091),chrono.ChQuaternionD(0.334941147774119,0.0189688610068957,0.942023949784192,0.00674446979727859)))

# Collision shapes 
body_37.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_37.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_37.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_37.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_37.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_37.GetCollisionModel().BuildModel()
body_37.SetCollide(True)

exported_items.append(body_37)



# Rigid body part
body_38= chrono.ChBodyAuxRef()
body_38.SetName('robo_leg_link-11/link2-3')
body_38.SetPos(chrono.ChVectorD(1.01171943077908,0.0110000000000009,-0.644115596465643))
body_38.SetRot(chrono.ChQuaternionD(0.942214742362167,7.37723214096588e-17,-0.335009521171259,-4.74143599748075e-17))
body_38.SetMass(0.0328349916079305)
body_38.SetInertiaXX(chrono.ChVectorD(1.00924716841445e-05,2.05032007850367e-05,1.40053937987597e-05))
body_38.SetInertiaXY(chrono.ChVectorD(-1.32046294156269e-11,9.4411355823548e-06,1.0012201062814e-11))
body_38.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_38.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_38.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_38.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_38.GetCollisionModel().BuildModel()
body_38.SetCollide(True)

exported_items.append(body_38)



# Rigid body part
body_39= chrono.ChBodyAuxRef()
body_39.SetName('robo_leg_link-11/link3-3')
body_39.SetPos(chrono.ChVectorD(0.951796667591943,0.0110000000000218,-0.558781063371062))
body_39.SetRot(chrono.ChQuaternionD(-0.0380977445935581,-1.29317722962249e-14,0.999274017403076,-3.79053752619649e-15))
body_39.SetMass(0.0328351485462318)
body_39.SetInertiaXX(chrono.ChVectorD(2.5199226624826e-06,2.05022207359738e-05,2.15788057501711e-05))
body_39.SetInertiaXY(chrono.ChVectorD(1.64676445341158e-11,-1.46390907472311e-06,1.83043738711841e-12))
body_39.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_39.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_39.GetCollisionModel().BuildModel()
body_39.SetCollide(True)

exported_items.append(body_39)



# Rigid body part
body_40= chrono.ChBodyAuxRef()
body_40.SetName('robo_leg_link-11/link1-1')
body_40.SetPos(chrono.ChVectorD(0.984741167157688,0.0780000000000008,-0.610973476759521))
body_40.SetRot(chrono.ChQuaternionD(-6.99622679232249e-17,0.942214911607173,-4.23455832166887e-17,0.335009045169063))
body_40.SetMass(0.074045467558508)
body_40.SetInertiaXX(chrono.ChVectorD(5.26171940127036e-05,9.69039180113677e-05,6.22679977847917e-05))
body_40.SetInertiaXY(chrono.ChVectorD(-9.12108018233021e-08,-2.32815509227956e-05,-7.42471453684273e-08))
body_40.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_40.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_40)



# Rigid body part
body_41= chrono.ChBodyAuxRef()
body_41.SetName('robo_leg_link-11/link3-2')
body_41.SetPos(chrono.ChVectorD(1.01714561167026,0.0110000000000009,-0.663110884558152))
body_41.SetRot(chrono.ChQuaternionD(0.997993014811501,2.36396837684687e-16,0.0633241058954016,-1.83381370409812e-16))
body_41.SetMass(0.0328351485462318)
body_41.SetInertiaXX(chrono.ChVectorD(2.71618206076054e-06,2.05022207359737e-05,2.13825463518931e-05))
body_41.SetInertiaXY(chrono.ChVectorD(-1.63542026385384e-11,-2.41764285085363e-06,-2.65967845332453e-12))
body_41.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_41.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_41.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_41.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_41.GetCollisionModel().BuildModel()
body_41.SetCollide(True)

exported_items.append(body_41)



# Rigid body part
body_42= chrono.ChBodyAuxRef()
body_42.SetName('robo_leg_link-11/link2-4')
body_42.SetPos(chrono.ChVectorD(0.957762911975438,0.0110000000000009,-0.577831350183785))
body_42.SetRot(chrono.ChQuaternionD(0.942214911967744,8.66590036963435e-17,-0.335009044154953,-3.54680176623829e-17))
body_42.SetMass(0.0328349916079305)
body_42.SetInertiaXX(chrono.ChVectorD(1.00924525650454e-05,2.05032007850367e-05,1.40054129178588e-05))
body_42.SetInertiaXY(chrono.ChVectorD(-1.32046395529504e-11,9.44113162033527e-06,1.00121876922188e-11))
body_42.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_42.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_42.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_42.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_42.GetCollisionModel().BuildModel()
body_42.SetCollide(True)

exported_items.append(body_42)



# Rigid body part
body_43= chrono.ChBodyAuxRef()
body_43.SetName('robo_leg_link-1/link2-4')
body_43.SetPos(chrono.ChVectorD(0.895890888929548,0.011000000000001,-0.773986312207437))
body_43.SetRot(chrono.ChQuaternionD(0.715510947118694,8.16872956286535e-17,0.698601520577581,1.72213578990573e-17))
body_43.SetMass(0.0328349916079305)
body_43.SetInertiaXX(chrono.ChVectorD(2.16796278565385e-05,2.05032007850367e-05,2.41823762636571e-06))
body_43.SetInertiaXY(chrono.ChVectorD(1.75094979202317e-13,-4.60969145841834e-07,-1.65703273927904e-11))
body_43.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_43.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_43.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_43.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_43.GetCollisionModel().BuildModel()
body_43.SetCollide(True)

exported_items.append(body_43)



# Rigid body part
body_44= chrono.ChBodyAuxRef()
body_44.SetName('robo_leg_link-1/single_bot1-1')
body_44.SetPos(chrono.ChVectorD(0.834316324423206,-0.114692616844647,-0.667153280272936))
body_44.SetRot(chrono.ChQuaternionD(-0.194681942453594,0.253949263014978,0.674303461530732,0.665525021967107))
body_44.SetMass(0.08424283226466)
body_44.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_44.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_44.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_44.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_44_1 =chrono.ChMarker()
marker_44_1.SetName('My_marker')
body_44.AddMarker(marker_44_1)
marker_44_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.845422779766053,0.0170000000000011,-0.777193934566912),chrono.ChQuaternionD(0.698460649103432,-0.0144047968956026,-0.715365232656205,0.0140644014142715)))

# Collision shapes 
body_44.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_44.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_44.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_44.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_44.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_44.GetCollisionModel().BuildModel()
body_44.SetCollide(True)

exported_items.append(body_44)



# Rigid body part
body_45= chrono.ChBodyAuxRef()
body_45.SetName('robo_leg_link-1/link3-3')
body_45.SetPos(chrono.ChVectorD(0.917236287619375,0.0109999999999982,-0.775490434770561))
body_45.SetRot(chrono.ChQuaternionD(0.762846935706247,1.14155428065545e-15,-0.64657911556405,-1.46491885975745e-14))
body_45.SetMass(0.0328351485462318)
body_45.SetInertiaXX(chrono.ChVectorD(2.11727874585613e-05,2.05022207359738e-05,2.92594095409234e-06))
body_45.SetInertiaXY(chrono.ChVectorD(-3.27712474935766e-12,3.11711951602568e-06,1.6241745566408e-11))
body_45.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_45.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_45.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_45.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_45.GetCollisionModel().BuildModel()
body_45.SetCollide(True)

exported_items.append(body_45)



# Rigid body part
body_46= chrono.ChBodyAuxRef()
body_46.SetName('robo_leg_link-1/leg-1')
body_46.SetPos(chrono.ChVectorD(0.846092588225907,0.0170000000000011,-0.777177914981794))
body_46.SetRot(chrono.ChQuaternionD(0.715506771602788,-0.00217755689696002,0.698598843385165,-0.00223025949741149))
body_46.SetMass(0.00100053137131845)
body_46.SetInertiaXX(chrono.ChVectorD(5.94366648364527e-08,5.12129068802266e-08,1.89430817282858e-08))
body_46.SetInertiaXY(chrono.ChVectorD(5.11817784290766e-11,-9.6899159238439e-10,-9.72805516185325e-12))
body_46.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_46.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_46.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_46.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_46.GetCollisionModel().BuildModel()
body_46.SetCollide(True)

exported_items.append(body_46)



# Rigid body part
body_47= chrono.ChBodyAuxRef()
body_47.SetName('robo_leg_link-1/link1-1')
body_47.SetPos(chrono.ChVectorD(0.853168731382154,0.078000000000001,-0.775008105305291))
body_47.SetRot(chrono.ChQuaternionD(-7.27336952359237e-17,0.71551024749256,2.66690215865053e-17,-0.698602237137225))
body_47.SetMass(0.074045467558508)
body_47.SetInertiaXX(chrono.ChVectorD(8.11918038818677e-05,9.69039180113677e-05,3.36933879156276e-05))
body_47.SetInertiaXY(chrono.ChVectorD(-2.81202851621043e-09,1.13588595689428e-06,1.17576109223913e-07))
body_47.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_47.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_47)



# Rigid body part
body_48= chrono.ChBodyAuxRef()
body_48.SetName('robo_leg_link-1/link2-3')
body_48.SetPos(chrono.ChVectorD(0.810446573373309,0.0110000000000009,-0.77602987910996))
body_48.SetRot(chrono.ChQuaternionD(0.715510319861456,7.68425201008419e-17,0.698602163016804,2.19515887457824e-17))
body_48.SetMass(0.0328349916079305)
body_48.SetInertiaXX(chrono.ChVectorD(2.16796295120473e-05,2.05032007850367e-05,2.41823597085685e-06))
body_48.SetInertiaXY(chrono.ChVectorD(1.75124735378884e-13,-4.60934557191322e-07,-1.65703270780919e-11))
body_48.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_48.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_48.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_48.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_48.GetCollisionModel().BuildModel()
body_48.SetCollide(True)

exported_items.append(body_48)



# Rigid body part
body_49= chrono.ChBodyAuxRef()
body_49.SetName('robo_leg_link-1/link3-2')
body_49.SetPos(chrono.ChVectorD(0.789130238992871,0.011000000000001,-0.774358912361577))
body_49.SetRot(chrono.ChQuaternionD(0.640548982601932,1.35409119606302e-16,0.767917313835044,-9.20782013322856e-17))
body_49.SetMass(0.0328351485462318)
body_49.SetInertiaXX(chrono.ChVectorD(2.10700401169986e-05,2.05022207359738e-05,3.02868829565506e-06))
body_49.SetInertiaXY(chrono.ChVectorD(3.53263011909193e-12,3.40304158529206e-06,-1.61880930884209e-11))
body_49.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

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
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_49.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_49.GetCollisionModel().BuildModel()
body_49.SetCollide(True)

exported_items.append(body_49)



# Rigid body part
body_50= chrono.ChBodyAuxRef()
body_50.SetName('graound-1')
body_50.SetPos(chrono.ChVectorD(0.25,-1.10171934447595e-17,-0.25))
body_50.SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,0,0))
body_50.SetMass(40.0914755458284)
body_50.SetInertiaXX(chrono.ChVectorD(13.381883573545,26.7554149366551,13.3745647759435))
body_50.SetInertiaXY(chrono.ChVectorD(0.000139998297324689,1.80719073164046e-05,-0.000230019371465711))
body_50.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.750063565667459,0.749895560623561,0.00506512993061473),chrono.ChQuaternionD(1,0,0,0)))
body_50.SetBodyFixed(True)

# Visualization shape 
body_50_1_shape = chrono.ChObjShapeFile() 
body_50_1_shape.SetFilename(shapes_dir +'body_50_1.obj') 
body_50_1_level = chrono.ChAssetLevel() 
body_50_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_50_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_50_1_level.GetAssets().push_back(body_50_1_shape) 
body_50.GetAssets().push_back(body_50_1_level) 

# Collision shapes 
body_50.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_50.GetCollisionModel().AddBox(0.0173210915247496,0.898770232768381,0.05,chrono.ChVectorD(-0.232678908475251,0.769778827599257,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_50.GetCollisionModel().AddBox(0.0238671590667463,0.905244271051395,0.05,chrono.ChVectorD(1.72613284093325,0.762370811199574,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_50.GetCollisionModel().AddBox(1,0.0299331228275488,0.05,chrono.ChVectorD(0.75,1.72006687717245,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=-1.38777878078145E-17; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_50.GetCollisionModel().AddBox(1,0.0407925544906709,0.05,chrono.ChVectorD(0.75,-0.209207445509329,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=8.32667268468867E-17; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_50.GetCollisionModel().AddBox(1,1,0.00500000000005684,chrono.ChVectorD(0.75,0.75,0.00499999999994316),mr)
body_50.GetCollisionModel().BuildModel()
body_50.SetCollide(True)

exported_items.append(body_50)



# Rigid body part
body_51= chrono.ChBodyAuxRef()
body_51.SetName('robo_leg_link-4/link3-2')
body_51.SetPos(chrono.ChVectorD(0.377097966483732,0.0110000000000009,-0.642803485996395))
body_51.SetRot(chrono.ChQuaternionD(0.144301424503413,3.29485616487913e-17,0.989533778547395,1.34704072674031e-16))
body_51.SetMass(0.0328351485462318)
body_51.SetInertiaXX(chrono.ChVectorD(3.9807587924776e-06,2.05022207359738e-05,2.01179696201761e-05))
body_51.SetInertiaXY(chrono.ChVectorD(1.60327355338304e-11,5.27739448896934e-06,-4.18153374963713e-12))
body_51.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_51.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_51.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_51.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_51.GetCollisionModel().BuildModel()
body_51.SetCollide(True)

exported_items.append(body_51)



# Rigid body part
body_52= chrono.ChBodyAuxRef()
body_52.SetName('robo_leg_link-4/link2-3')
body_52.SetPos(chrono.ChVectorD(0.388353279608885,0.0110000000000009,-0.660065665572314))
body_52.SetRot(chrono.ChQuaternionD(0.419584984074723,4.8732693741997e-17,0.907716057552699,6.49769249893293e-17))
body_52.SetMass(0.0328349916079305)
body_52.SetInertiaXX(chrono.ChVectorD(1.35960347408698e-05,2.05032007850367e-05,1.05018307420344e-05))
body_52.SetInertiaXY(chrono.ChVectorD(1.11652348647736e-11,9.51678813237768e-06,-1.22451598004487e-11))
body_52.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_52.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_52.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_52.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_52.GetCollisionModel().BuildModel()
body_52.SetCollide(True)

exported_items.append(body_52)



# Rigid body part
body_53= chrono.ChBodyAuxRef()
body_53.SetName('robo_leg_link-4/link1-1')
body_53.SetPos(chrono.ChVectorD(0.420905254893047,0.0780000000000009,-0.687753130456986))
body_53.SetRot(chrono.ChQuaternionD(5.06437814254925e-17,-0.419585048224306,-7.64434436611208e-17,0.907716027900029))
body_53.SetMass(0.074045467558508)
body_53.SetInertiaXX(chrono.ChVectorD(6.12569819153883e-05,9.69039180113677e-05,5.36282098821069e-05))
body_53.SetInertiaXY(chrono.ChVectorD(7.61989657601164e-08,-2.34683952593401e-05,8.95866428754792e-08))
body_53.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_53.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_53)



# Rigid body part
body_54= chrono.ChBodyAuxRef()
body_54.SetName('robo_leg_link-4/link2-4')
body_54.SetPos(chrono.ChVectorD(0.45345723116167,0.0110000000000073,-0.715440594184202))
body_54.SetRot(chrono.ChQuaternionD(0.419585048248267,-1.33546696077608e-14,0.907716027888954,2.08881709806561e-15))
body_54.SetMass(0.0328349916079305)
body_54.SetInertiaXX(chrono.ChVectorD(1.35960374321343e-05,2.05032007850367e-05,1.05018280507698e-05))
body_54.SetInertiaXY(chrono.ChVectorD(1.11652328356071e-11,9.51678769487036e-06,-1.22451610139345e-11))
body_54.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_54.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_54.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_54.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_54.GetCollisionModel().BuildModel()
body_54.SetCollide(True)

exported_items.append(body_54)



# Rigid body part
body_55= chrono.ChBodyAuxRef()
body_55.SetName('robo_leg_link-4/link3-3')
body_55.SetPos(chrono.ChVectorD(0.472377368628641,0.0110000000000077,-0.722930572831116))
body_55.SetRot(chrono.ChQuaternionD(0.7242403003755,6.59416972008968e-15,-0.689547668629229,1.32915341630429e-14))
body_55.SetMass(0.0328351485462318)
body_55.SetInertiaXX(chrono.ChVectorD(2.1644204536424e-05,2.05022207359738e-05,2.45452387622967e-06))
body_55.SetInertiaXY(chrono.ChVectorD(-1.38278357603657e-12,9.44629534019637e-07,1.65112605754036e-11))
body_55.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_55.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_55.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_55.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_55.GetCollisionModel().BuildModel()
body_55.SetCollide(True)

exported_items.append(body_55)



# Rigid body part
body_56= chrono.ChBodyAuxRef()
body_56.SetName('robo_leg_link-4/leg-1')
body_56.SetPos(chrono.ChVectorD(0.414181386570577,0.017000000000001,-0.684659674334456))
body_56.SetRot(chrono.ChQuaternionD(0.419196808302539,-0.0390395326011632,0.906876122852998,-0.0180457364039547))
body_56.SetMass(0.00100053137131845)
body_56.SetInertiaXX(chrono.ChVectorD(4.25069273479277e-08,5.1148228762541e-08,3.59374973344964e-08))
body_56.SetInertiaXY(chrono.ChVectorD(-7.48530904092899e-10,1.99336994806194e-08,1.71659276316368e-09))
body_56.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_56.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_56.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_56.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_56.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_56.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_56.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_56.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_56.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_56.GetCollisionModel().BuildModel()
body_56.SetCollide(True)

exported_items.append(body_56)



# Rigid body part
body_57= chrono.ChBodyAuxRef()
body_57.SetName('robo_leg_link-4/single_bot1-1')
body_57.SetPos(chrono.ChVectorD(0.478664149251636,-0.114692616844647,-0.59473707456673))
body_57.SetRot(chrono.ChQuaternionD(-0.422193847052941,0.474442000127671,0.560659051884895,0.531336589710349))
body_57.SetMass(0.08424283226466)
body_57.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_57.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_57.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_57.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_57_1 =chrono.ChMarker()
marker_57_1.SetName('My_marker')
body_57.AddMarker(marker_57_1)
marker_57_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.413671028712301,0.0170000000000011,-0.684225583495465),chrono.ChQuaternionD(0.907532058080297,-0.00844717098223103,-0.419500009530148,0.0182743225084345)))

# Collision shapes 
body_57.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_57.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_57.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_57.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_57.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_57.GetCollisionModel().BuildModel()
body_57.SetCollide(True)

exported_items.append(body_57)



# Rigid body part
body_58= chrono.ChBodyAuxRef()
body_58.SetName('robo_leg_link-6/link3-2')
body_58.SetPos(chrono.ChVectorD(0.390373827163711,0.0110000000000009,-0.36398157583418))
body_58.SetRot(chrono.ChQuaternionD(0.812605900694051,1.70781282734489e-16,-0.582813563806824,-2.1134183738393e-16))
body_58.SetMass(0.0328351485462318)
body_58.SetInertiaXX(chrono.ChVectorD(1.97079557115713e-05,2.05022207359738e-05,4.3907727010824e-06))
body_58.SetInertiaXY(chrono.ChVectorD(-5.85093071455708e-12,5.85655718595822e-06,1.55016271094632e-11))
body_58.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_58.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_58.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_58.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_58.GetCollisionModel().BuildModel()
body_58.SetCollide(True)

exported_items.append(body_58)



# Rigid body part
body_59= chrono.ChBodyAuxRef()
body_59.SetName('robo_leg_link-6/link3-3')
body_59.SetPos(chrono.ChVectorD(0.351675387261123,0.0110000000000008,-0.473706053517781))
body_59.SetRot(chrono.ChQuaternionD(0.996462002129973,7.97362627711442e-16,-0.0840445019684548,-3.72010809695748e-17))
body_59.SetMass(0.0328351485462318)
body_59.SetInertiaXX(chrono.ChVectorD(2.94909194291117e-06,2.05022207359737e-05,2.11496364697425e-05))
body_59.SetInertiaXY(chrono.ChVectorD(-1.64209652635354e-11,3.18407453560821e-06,2.21036941650213e-12))
body_59.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_59.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_59.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_59.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_59.GetCollisionModel().BuildModel()
body_59.SetCollide(True)

exported_items.append(body_59)



# Rigid body part
body_60= chrono.ChBodyAuxRef()
body_60.SetName('robo_leg_link-6/link1-1')
body_60.SetPos(chrono.ChVectorD(0.365034893301768,0.0780000000000076,-0.411823259348937))
body_60.SetRot(chrono.ChQuaternionD(-1.23922185585304e-14,0.143203646744052,5.2022778899177e-15,0.989693243161337))
body_60.SetMass(0.074045467558508)
body_60.SetInertiaXX(chrono.ChVectorD(3.74873717627329e-05,9.69039180113677e-05,7.73978200347624e-05))
body_60.SetInertiaXY(chrono.ChVectorD(1.12786019153856e-07,1.29268769010144e-05,-3.33371091791716e-08))
body_60.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_60.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_60)



# Rigid body part
body_61= chrono.ChBodyAuxRef()
body_61.SetName('robo_leg_link-6/leg-1')
body_61.SetPos(chrono.ChVectorD(0.365136825357394,0.0169999999999809,-0.404422618440478))
body_61.SetRot(chrono.ChQuaternionD(-0.143202915724388,-0.00316229614209767,0.989688191018927,0.000457568385699393))
body_61.SetMass(0.00100053137131845)
body_61.SetInertiaXX(chrono.ChVectorD(2.21783316611845e-08,5.12114478265958e-08,5.62028739571847e-08))
body_61.SetInertiaXY(chrono.ChVectorD(-1.82008067903897e-10,-1.10198211439085e-08,-6.93773069632732e-11))
body_61.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_61.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_61.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_61.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_61.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_61.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_61.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_61.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_61.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_61.GetCollisionModel().BuildModel()
body_61.SetCollide(True)

exported_items.append(body_61)



# Rigid body part
body_62= chrono.ChBodyAuxRef()
body_62.SetName('robo_leg_link-6/single_bot1-1')
body_62.SetPos(chrono.ChVectorD(0.473480116571354,-0.114692616844625,-0.426913324985534))
body_62.SetRot(chrono.ChQuaternionD(0.659612893474243,-0.687423142631397,-0.239785420141396,-0.186716914137218))
body_62.SetMass(0.08424283226466)
body_62.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_62.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_62.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_62.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_62_1 =chrono.ChMarker()
marker_62_1.SetName('My_marker')
body_62.AddMarker(marker_62_1)
marker_62_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.365326740450707,0.0170000000000195,-0.403780098201626),chrono.ChQuaternionD(0.98949265874734,0.00288300475539014,0.143174623185724,0.0199247043723708)))

# Collision shapes 
body_62.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_62.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_62.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_62.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_62.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_62.GetCollisionModel().BuildModel()
body_62.SetCollide(True)

exported_items.append(body_62)



# Rigid body part
body_63= chrono.ChBodyAuxRef()
body_63.SetName('robo_leg_link-6/link2-3')
body_63.SetPos(chrono.ChVectorD(0.377148178724954,0.0110000000000082,-0.37084161561786))
body_63.SetRot(chrono.ChQuaternionD(-0.143203533343429,-1.24342852894929e-14,0.989693259569831,-5.20841256522812e-15))
body_63.SetMass(0.0328349916079305)
body_63.SetInertiaXX(chrono.ChVectorD(3.95657584480717e-06,2.05032007850367e-05,2.0141289638097e-05))
body_63.SetInertiaXY(chrono.ChVectorD(1.57202145247745e-11,-5.2418070176696e-06,5.24225598021e-12))
body_63.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_63.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_63.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_63.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_63.GetCollisionModel().BuildModel()
body_63.SetCollide(True)

exported_items.append(body_63)



# Rigid body part
body_64= chrono.ChBodyAuxRef()
body_64.SetName('robo_leg_link-6/link2-4')
body_64.SetPos(chrono.ChVectorD(0.352921605885125,0.0110000000000069,-0.452804902490906))
body_64.SetRot(chrono.ChQuaternionD(-0.143203629054001,-1.24272743073189e-14,0.989693245720998,-5.22594052852908e-15))
body_64.SetMass(0.0328349916079305)
body_64.SetInertiaXX(chrono.ChVectorD(3.95657787249201e-06,2.05032007850367e-05,2.01412876104122e-05))
body_64.SetInertiaXY(chrono.ChVectorD(1.57202135109697e-11,-5.24181014802949e-06,5.24225902015697e-12))
body_64.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_64.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_64.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_64.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_64.GetCollisionModel().BuildModel()
body_64.SetCollide(True)

exported_items.append(body_64)



# Rigid body part
body_65= chrono.ChBodyAuxRef()
body_65.SetName('robo_leg_link-8/link1-1')
body_65.SetPos(chrono.ChVectorD(0.611449207348265,0.0780000000000137,-0.44329406694588))
body_65.SetRot(chrono.ChQuaternionD(-4.93222907964174e-15,0.807882148208768,1.24948371852135e-14,0.589344071494392))
body_65.SetMass(0.074045467558508)
body_65.SetInertiaXX(chrono.ChVectorD(7.67848399755331e-05,9.69039180113677e-05,3.81003518219622e-05))
body_65.SetInertiaXY(chrono.ChVectorD(-3.59117940943723e-08,-1.38272452356433e-05,-1.11992821242796e-07))
body_65.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_65.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_65)



# Rigid body part
body_66= chrono.ChBodyAuxRef()
body_66.SetName('robo_leg_link-8/link2-3')
body_66.SetPos(chrono.ChVectorD(0.652142636595736,0.0110000000000009,-0.456342883854805))
body_66.SetRot(chrono.ChQuaternionD(0.807882093978433,7.2334377624832e-17,-0.589344145834183,-9.32454809088516e-17))
body_66.SetMass(0.0328349916079305)
body_66.SetInertiaXX(chrono.ChVectorD(1.98927270828645e-05,2.05032007850367e-05,4.20513840003966e-06))
body_66.SetInertiaXY(chrono.ChVectorD(-5.60096237536648e-12,5.6069306472252e-06,1.55960132281035e-11))
body_66.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_66.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_66.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_66.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_66.GetCollisionModel().BuildModel()
body_66.SetCollide(True)

exported_items.append(body_66)



# Rigid body part
body_67= chrono.ChBodyAuxRef()
body_67.SetName('robo_leg_link-8/single_bot1-1')
body_67.SetPos(chrono.ChVectorD(0.593879097898218,-0.11469261684465,-0.55136519067844))
body_67.SetRot(chrono.ChQuaternionD(0.632528796652985,-0.614014655831873,0.304127503449717,0.3611091046676))
body_67.SetMass(0.08424283226466)
body_67.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_67.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_67.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_67.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_67_1 =chrono.ChMarker()
marker_67_1.SetName('My_marker')
body_67.AddMarker(marker_67_1)
marker_67_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.619483573295809,0.0169999999999978,-0.443770070959455),chrono.ChQuaternionD(0.589224627175624,0.0162644466676709,0.807718412052741,0.011864793944259)))

# Collision shapes 
body_67.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_67.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_67.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_67.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_67.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_67.GetCollisionModel().BuildModel()
body_67.SetCollide(True)

exported_items.append(body_67)



# Rigid body part
body_68= chrono.ChBodyAuxRef()
body_68.SetName('robo_leg_link-8/link2-4')
body_68.SetPos(chrono.ChVectorD(0.570755780288265,0.0110000000000008,-0.430245243215288))
body_68.SetRot(chrono.ChQuaternionD(0.807882290403776,9.23316556793458e-17,-0.589343876571182,-8.26690405501119e-17))
body_68.SetMass(0.0328349916079305)
body_68.SetInertiaXX(chrono.ChVectorD(1.98927196078124e-05,2.05032007850367e-05,4.20514587509183e-06))
body_68.SetInertiaXY(chrono.ChVectorD(-5.6009727719258e-12,5.60694110440632e-06,1.55960094932125e-11))
body_68.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_68.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_68.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_68.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_68.GetCollisionModel().BuildModel()
body_68.SetCollide(True)

exported_items.append(body_68)



# Rigid body part
body_69= chrono.ChBodyAuxRef()
body_69.SetName('robo_leg_link-8/link3-2')
body_69.SetPos(chrono.ChVectorD(0.6730442357932,0.0109999999999966,-0.460924206804141))
body_69.SetRot(chrono.ChQuaternionD(0.748647628617573,1.04503141098599e-15,-0.662968120021834,-1.47486140375296e-14))
body_69.SetMass(0.0328351485462318)
body_69.SetInertiaXX(chrono.ChVectorD(2.14085272518343e-05,2.05022207359738e-05,2.69020116081934e-06))
body_69.SetInertiaXY(chrono.ChVectorD(-2.5698602274625e-12,2.31502764653212e-06,1.63685571740926e-11))
body_69.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_69.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_69.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_69.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_69.GetCollisionModel().BuildModel()
body_69.SetCollide(True)

exported_items.append(body_69)



# Rigid body part
body_70= chrono.ChBodyAuxRef()
body_70.SetName('robo_leg_link-8/leg-1')
body_70.SetPos(chrono.ChVectorD(0.618845571752761,0.017000000000038,-0.443565488381824))
body_70.SetRot(chrono.ChQuaternionD(0.807878223579798,0.00183699801132201,-0.589341208506497,-0.00251818584660124))
body_70.SetMass(0.00100053137131845)
body_70.SetInertiaXX(chrono.ChVectorD(5.56801534429973e-08,5.12127479438386e-08,2.26997520581291e-08))
body_70.SetInertiaXY(chrono.ChVectorD(2.67252099330214e-11,1.17873990547415e-08,7.69963595321719e-11))
body_70.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_70.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_70.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_70.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_70.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_70.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_70.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_70.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_70.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_70.GetCollisionModel().BuildModel()
body_70.SetCollide(True)

exported_items.append(body_70)



# Rigid body part
body_71= chrono.ChBodyAuxRef()
body_71.SetName('robo_leg_link-8/link3-3')
body_71.SetPos(chrono.ChVectorD(0.549996313524184,0.0109999999999983,-0.424795357038013))
body_71.SetRot(chrono.ChQuaternionD(0.63175708050415,1.45338954507925e-14,0.775166428086816,1.559654754591e-15))
body_71.SetMass(0.0328351485462318)
body_71.SetInertiaXX(chrono.ChVectorD(2.09056126893458e-05,2.05022207359738e-05,3.19311572330784e-06))
body_71.SetInertiaXY(chrono.ChVectorD(3.90061116478084e-12,3.81053129151631e-06,-1.61033871595929e-11))
body_71.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_71.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_71.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_71.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_71.GetCollisionModel().BuildModel()
body_71.SetCollide(True)

exported_items.append(body_71)



# Rigid body part
body_72= chrono.ChBodyAuxRef()
body_72.SetName('robo_leg_link-9/link3-2')
body_72.SetPos(chrono.ChVectorD(0.820169443348337,0.0109999999999946,-0.489360758032558))
body_72.SetRot(chrono.ChQuaternionD(0.783356823424262,2.50014137339952e-15,-0.621572270290953,-1.55312148116364e-14))
body_72.SetMass(0.0328351485462318)
body_72.SetInertiaXX(chrono.ChVectorD(2.06943948908256e-05,2.05022207359737e-05,3.4043335218281e-06))
body_72.SetInertiaXY(chrono.ChVectorD(-4.32016153319893e-12,4.26810618198242e-06,1.59959385532388e-11))
body_72.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_72.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_72.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_72.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_72.GetCollisionModel().BuildModel()
body_72.SetCollide(True)

exported_items.append(body_72)



# Rigid body part
body_73= chrono.ChBodyAuxRef()
body_73.SetName('robo_leg_link-9/leg-1')
body_73.SetPos(chrono.ChVectorD(0.764761463500638,0.017000000000001,-0.475385920542641))
body_73.SetRot(chrono.ChQuaternionD(0.776708015241412,0.00196327406122825,-0.629853112438361,-0.00242102590166306))
body_73.SetMass(0.00100053137131845)
body_73.SetInertiaXX(chrono.ChVectorD(5.77301429663142e-08,5.12128321579141e-08,2.06496783207368e-08))
body_73.SetInertiaXY(chrono.ChVectorD(3.98694364209461e-11,8.19331011996929e-09,5.46869076815227e-11))
body_73.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_73.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_73.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_73.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_73.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_73.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_73.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_73.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_73.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_73.GetCollisionModel().BuildModel()
body_73.SetCollide(True)

exported_items.append(body_73)



# Rigid body part
body_74= chrono.ChBodyAuxRef()
body_74.SetName('robo_leg_link-9/single_bot1-1')
body_74.SetPos(chrono.ChVectorD(0.750927906679107,-0.114692616844647,-0.585170861848969))
body_74.SetRot(chrono.ChQuaternionD(0.647243086688446,-0.631664839753589,0.271408554920152,0.329261770130153))
body_74.SetMass(0.08424283226466)
body_74.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_74.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_74.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_74.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_74_1 =chrono.ChMarker()
marker_74_1.SetName('My_marker')
body_74.AddMarker(marker_74_1)
marker_74_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.765417013897388,0.017000000000001,-0.475524317353732),chrono.ChQuaternionD(0.629728517187027,0.0156369186858448,0.776554369687456,0.0126803917430655)))

# Collision shapes 
body_74.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_74.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_74.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_74.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_74.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_74.GetCollisionModel().BuildModel()
body_74.SetCollide(True)

exported_items.append(body_74)



# Rigid body part
body_75= chrono.ChBodyAuxRef()
body_75.SetName('robo_leg_link-9/link2-3')
body_75.SetPos(chrono.ChVectorD(0.799188757904994,0.0110000000000008,-0.484698137864621))
body_75.SetRot(chrono.ChQuaternionD(0.776711788443225,6.253575399532e-17,-0.629856172227697,-6.14190441025465e-17))
body_75.SetMass(0.0328349916079305)
body_75.SetInertiaXX(chrono.ChVectorD(2.08678666104511e-05,2.05032007850367e-05,3.22999887245309e-06))
body_75.SetInertiaXY(chrono.ChVectorD(-3.97991455542714e-12,3.89733067206123e-06,1.60862266624855e-11))
body_75.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_75.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_75.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_75.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_75.GetCollisionModel().BuildModel()
body_75.SetCollide(True)

exported_items.append(body_75)



# Rigid body part
body_76= chrono.ChBodyAuxRef()
body_76.SetName('robo_leg_link-9/link2-4')
body_76.SetPos(chrono.ChVectorD(0.715563283810987,0.0110000000000009,-0.467043504059211))
body_76.SetRot(chrono.ChQuaternionD(0.776711930810821,9.15701944230496e-17,-0.629855996666005,-9.04534847349636e-17))
body_76.SetMass(0.0328349916079305)
body_76.SetInertiaXX(chrono.ChVectorD(2.08678630867626e-05,2.05032007850367e-05,3.23000239614162e-06))
body_76.SetInertiaXY(chrono.ChVectorD(-3.97992182707101e-12,3.89733864550272e-06,1.60862248638694e-11))
body_76.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_76.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_76.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_76.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_76.GetCollisionModel().BuildModel()
body_76.SetCollide(True)

exported_items.append(body_76)



# Rigid body part
body_77= chrono.ChBodyAuxRef()
body_77.SetName('robo_leg_link-9/link1-1')
body_77.SetPos(chrono.ChVectorD(0.757376022083587,0.0780000000000008,-0.475870823703338))
body_77.SetRot(chrono.ChQuaternionD(-6.25357539953199e-17,0.776711788443227,-6.81193034591878e-17,0.629856172227695))
body_77.SetMass(0.074045467558508)
body_77.SetInertiaXX(chrono.ChVectorD(7.91896616543866e-05,9.69039180113677e-05,3.56955301431086e-05))
body_77.SetInertiaXY(chrono.ChVectorD(-2.4293748963105e-08,-9.61146439167707e-06,-1.15073292843856e-07))
body_77.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_77.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_77)



# Rigid body part
body_78= chrono.ChBodyAuxRef()
body_78.SetName('robo_leg_link-9/link3-3')
body_78.SetPos(chrono.ChVectorD(0.69438020064316,0.0109999999999962,-0.463523801577136))
body_78.SetRot(chrono.ChQuaternionD(0.662968120021834,1.4608487005468e-14,0.748647628617573,9.49826398528548e-16))
body_78.SetMass(0.0328351485462318)
body_78.SetInertiaXX(chrono.ChVectorD(2.14085272518343e-05,2.05022207359738e-05,2.69020116081934e-06))
body_78.SetInertiaXY(chrono.ChVectorD(2.56986022608356e-12,2.31502764653212e-06,-1.63685571724222e-11))
body_78.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_78.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_78.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_78.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_78.GetCollisionModel().BuildModel()
body_78.SetCollide(True)

exported_items.append(body_78)



# Rigid body part
body_79= chrono.ChBodyAuxRef()
body_79.SetName('robo_leg_link-12/link3-3')
body_79.SetPos(chrono.ChVectorD(1.01442892997278,0.0109999999999942,-0.684432257159771))
body_79.SetRot(chrono.ChQuaternionD(-0.0633241058954017,9.6192654393166e-15,0.997993014811501,1.0957688711502e-14))
body_79.SetMass(0.0328351485462318)
body_79.SetInertiaXX(chrono.ChVectorD(2.71618206076054e-06,2.05022207359737e-05,2.13825463518931e-05))
body_79.SetInertiaXY(chrono.ChVectorD(1.6354203004099e-11,-2.41764285085363e-06,2.65967847635065e-12))
body_79.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_79.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_79.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_79.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_79.GetCollisionModel().BuildModel()
body_79.SetCollide(True)

exported_items.append(body_79)



# Rigid body part
body_80= chrono.ChBodyAuxRef()
body_80.SetName('robo_leg_link-12/link3-2')
body_80.SetPos(chrono.ChVectorD(0.938439480679149,0.0109999999999978,-0.779012634812075))
body_80.SetRot(chrono.ChQuaternionD(0.646579115564051,1.45709673746751e-14,0.762846935706246,1.27551090259446e-15))
body_80.SetMass(0.0328351485462318)
body_80.SetInertiaXX(chrono.ChVectorD(2.11727874585613e-05,2.05022207359737e-05,2.92594095409234e-06))
body_80.SetInertiaXY(chrono.ChVectorD(3.27712474743084e-12,3.11711951602566e-06,-1.62417455613963e-11))
body_80.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(8.75352465822206e-07,0.0335000807718491,-1.12353616888784e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_80.GetAssets().push_back(body_2_1_level) 

# Collision shapes 
body_80.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_80.GetCollisionModel().AddBox(0.00872667810228913,0.0403837584966534,0.006746875,chrono.ChVectorD(2.08166817117217E-17,0.0336177373374987,-8.67361737988404E-19),mr)
body_80.GetCollisionModel().BuildModel()
body_80.SetCollide(True)

exported_items.append(body_80)



# Rigid body part
body_81= chrono.ChBodyAuxRef()
body_81.SetName('robo_leg_link-12/link2-3')
body_81.SetPos(chrono.ChVectorD(0.955474338184651,0.011000000000001,-0.772165106865193))
body_81.SetRot(chrono.ChQuaternionD(0.948956126695669,7.76914186599253e-17,0.315408100096927,1.46242670418683e-17))
body_81.SetMass(0.0328349916079305)
body_81.SetInertiaXX(chrono.ChVectorD(9.31728398431457e-06,2.05032007850367e-05,1.47805814985896e-05))
body_81.SetInertiaXY(chrono.ChVectorD(-1.2924297835818e-11,-9.24666837411162e-06,-1.03715444160462e-11))
body_81.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_81.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_81.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_81.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_81.GetCollisionModel().BuildModel()
body_81.SetCollide(True)

exported_items.append(body_81)



# Rigid body part
body_82= chrono.ChBodyAuxRef()
body_82.SetName('robo_leg_link-12/single_bot1-1')
body_82.SetPos(chrono.ChVectorD(0.884585453521218,-0.114692616844647,-0.686149743688261))
body_82.SetRot(chrono.ChQuaternionD(0.119765819397092,-0.0626262843675003,0.691550696228392,0.709571512644419))
body_82.SetMass(0.08424283226466)
body_82.SetInertiaXX(chrono.ChVectorD(5.8719973327761e-05,6.05753275703724e-05,2.65574970697557e-05))
body_82.SetInertiaXY(chrono.ChVectorD(1.15856166753899e-06,-5.64726976369661e-06,-4.35120365620264e-07))
body_82.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658499719061292,-0.0789639831153089,0.173220144636528),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_82.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_82_1 =chrono.ChMarker()
marker_82_1.SetName('My_marker')
body_82.AddMarker(marker_82_1)
marker_82_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.977991097988269,0.0170000000000011,-0.745375452037117),chrono.ChQuaternionD(-0.315343841797194,0.0191045787433515,0.948763909452631,-0.00634985289470594)))

# Collision shapes 
body_82.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.9964103237123; mr[2,0]=0.0148275661003071 
mr[0,1]=-0.0833463261523326; mr[1,1]=0.0148275661003071; mr[2,1]=-0.9964103237123 
mr[0,2]=-0.993053389916909; mr[1,2]=-0.00123582316024147; mr[2,2]=0.0830471398216766 
body_82.GetCollisionModel().AddCylinder(0.028575,0.028575,0.02,chrono.ChVectorD(-0.0658560965901889,-0.0772714083988229,0.171852628941588),mr)
body_82.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_82.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_82.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_82.GetCollisionModel().BuildModel()
body_82.SetCollide(True)

exported_items.append(body_82)



# Rigid body part
body_83= chrono.ChBodyAuxRef()
body_83.SetName('robo_leg_link-12/link1-1')
body_83.SetPos(chrono.ChVectorD(0.9810558391716,0.0780000000000009,-0.73793334363431))
body_83.SetRot(chrono.ChQuaternionD(-8.95736251665284e-17,0.948956237562512,1.46242653333108e-17,-0.315407766535958))
body_83.SetMass(0.074045467558508)
body_83.SetInertiaXX(chrono.ChVectorD(5.0707086407188e-05,9.69039180113677e-05,6.41781053903073e-05))
body_83.SetInertiaXY(chrono.ChVectorD(-9.42096150307947e-08,2.28023689256549e-05,7.04031064681031e-08))
body_83.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.36803983489018e-05,0.0335568978280171,-3.78598232120917e-18),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_83.GetAssets().push_back(body_5_1_level) 

exported_items.append(body_83)



# Rigid body part
body_84= chrono.ChBodyAuxRef()
body_84.SetName('robo_leg_link-12/link2-4')
body_84.SetPos(chrono.ChVectorD(1.00663733415349,0.0110000000000009,-0.703701575915824))
body_84.SetRot(chrono.ChQuaternionD(0.948956236704128,8.31755091584419e-17,0.315407769118547,3.65606633663481e-18))
body_84.SetMass(0.0328349916079305)
body_84.SetInertiaXX(chrono.ChVectorD(9.31727108404901e-06,2.05032007850367e-05,1.47805943988552e-05))
body_84.SetInertiaXY(chrono.ChVectorD(-1.29243050708513e-11,-9.24666456310819e-06,-1.03715354004534e-11))
body_84.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(6.90436931444946e-07,0.0335000057451059,-1.12354153874357e-09),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_84.GetAssets().push_back(body_6_1_level) 

# Collision shapes 
body_84.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_84.GetCollisionModel().AddBox(0.040676139256062,0.00859896697846772,0.006746875,chrono.ChVectorD(-0.000292380759408621,0.0335,0),mr)
body_84.GetCollisionModel().BuildModel()
body_84.SetCollide(True)

exported_items.append(body_84)



# Rigid body part
body_85= chrono.ChBodyAuxRef()
body_85.SetName('robo_leg_link-12/leg-1')
body_85.SetPos(chrono.ChVectorD(0.978392170932625,0.0170000000000011,-0.744838757996433))
body_85.SetRot(chrono.ChQuaternionD(0.94895162761137,-0.000983132204353926,0.315406234311034,-0.00295791523440213))
body_85.SetMass(0.00100053137131845)
body_85.SetInertiaXX(chrono.ChVectorD(3.34478889250526e-08,5.12118611059407e-08,4.49329034139717e-08))
body_85.SetInertiaXY(chrono.ChVectorD(-1.1370184334822e-10,-1.94391938100729e-08,-1.23394334676846e-10))
body_85.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.82140024380883e-07,0.00570453175606253,0.004),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_85.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_85.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_85.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00488083971690958,0.0109625454917112,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_85.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0100640468153451,0.00653566842018034,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_85.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.012,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_85.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0115911099154688,0.00310582854123019,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_85.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00802956727630634,0.0089177379057287,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_85.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00187721358048281,0.0118522600871416,0.00399999999999999),mr)
body_85.GetCollisionModel().BuildModel()
body_85.SetCollide(True)

exported_items.append(body_85)




# Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_50 , SW name: graound-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
cB = chrono.ChVectorD(0.825275365733867,0.010000000000002,-0.804268395850859)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
dB = chrono.ChVectorD(-2.6548208076349e-14,-1,-9.00668428727158e-15)
link_1.Initialize(body_50,body_44,False,cA,cB,dB)
link_1.SetDistance(0)
link_1.SetName("Coincident1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
cB = chrono.ChVectorD(0.825275365733867,0.010000000000002,-0.804268395850859)
dB = chrono.ChVectorD(-2.6548208076349e-14,-1,-9.00668428727158e-15)
link_2.SetFlipped(True)
link_2.Initialize(body_50,body_44,False,cA,cB,dA,dB)
link_2.SetName("Coincident1")
exported_items.append(link_2)


# Mate constraint: Parallel1 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_50 , SW name: graound-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
cB = chrono.ChVectorD(0.825275365733867,0.010000000000002,-0.804268395850859)
dB = chrono.ChVectorD(-2.6548208076349e-14,-1,-9.00668428727158e-15)
link_3.SetFlipped(True)
link_3.Initialize(body_50,body_44,False,cA,cB,dA,dB)
link_3.SetName("Parallel1")
exported_items.append(link_3)


# Mate constraint: Parallel2 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.647397367183373,0.00999999999999869,-0.424802602577964)
dB = chrono.ChVectorD(2.65482080763491e-14,-1,4.96130914129367e-15)
link_4.Initialize(body_74,body_67,False,cA,cB,dA,dB)
link_4.SetName("Parallel2")
exported_items.append(link_4)


# Mate constraint: Parallel3 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.93835338987098,0.0100000000000019,-0.492222167321494)
dB = chrono.ChVectorD(2.78943534937071e-14,-1,2.63677968348475e-16)
link_5.Initialize(body_74,body_14,False,cA,cB,dA,dB)
link_5.SetName("Parallel3")
exported_items.append(link_5)


# Mate constraint: Parallel4 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(1.02495565754505,0.0100000000000019,-0.615096486932298)
dB = chrono.ChVectorD(2.31585584042904e-14,-1,-1.57512891618694e-14)
link_6.Initialize(body_74,body_37,False,cA,cB,dA,dB)
link_6.SetName("Parallel4")
exported_items.append(link_6)


# Mate constraint: Parallel5 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.986842001915674,0.010000000000002,-0.777942402242319)
dB = chrono.ChVectorD(-9.35362898246695e-15,-1,-2.64371857738865e-14)
link_7.Initialize(body_74,body_82,False,cA,cB,dA,dB)
link_7.SetName("Parallel5")
exported_items.append(link_7)


# Mate constraint: Parallel6 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.825275365733867,0.010000000000002,-0.804268395850859)
dB = chrono.ChVectorD(-2.6548208076349e-14,-1,-9.00668428727158e-15)
link_8.Initialize(body_74,body_44,False,cA,cB,dA,dB)
link_8.SetName("Parallel6")
exported_items.append(link_8)


# Mate constraint: Parallel7 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.671310582085609,0.0100000000000019,-0.780339765058569)
dB = chrono.ChVectorD(-2.77416978278211e-14,-1,-2.94902990916057e-15)
link_9.Initialize(body_74,body_15,False,cA,cB,dA,dB)
link_9.SetName("Parallel7")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.523750649580164,0.0100000000000103,-0.757213293157226)
dB = chrono.ChVectorD(-1.60982338570648e-15,-1,3.62904151174348e-15)
link_10.Initialize(body_74,body_28,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Parallel9 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.380611126517858,0.010000000000002,-0.691007000227544)
dB = chrono.ChVectorD(-2.56669685505528e-14,-1,1.07414077632484e-14)
link_11.Initialize(body_74,body_57,False,cA,cB,dA,dB)
link_11.SetName("Parallel9")
exported_items.append(link_11)


# Mate constraint: Parallel10 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.331154294599887,0.0100000000000248,-0.533150136638942)
dB = chrono.ChVectorD(2.73323030874906e-14,-1,6.38378239159465e-15)
link_12.Initialize(body_74,body_4,False,cA,cB,dA,dB)
link_12.SetName("Parallel10")
exported_items.append(link_12)


# Mate constraint: Parallel11 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.345724866309098,0.0100000000000192,-0.376308089244567)
dB = chrono.ChVectorD(2.69159694532561e-14,-1,6.91807722219551e-15)
link_13.Initialize(body_74,body_62,False,cA,cB,dA,dB)
link_13.SetName("Parallel11")
exported_items.append(link_13)


# Mate constraint: Parallel12 [MateParallel] type:3 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)

link_14 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.791249127958436,0.0100000000000019,-0.453806890500228)
dA = chrono.ChVectorD(2.77555756156289e-14,-1,2.44249065417534e-15)
cB = chrono.ChVectorD(0.507474229551605,0.00999999999999744,-0.381997856143715)
dB = chrono.ChVectorD(2.5583701823706e-14,-1,5.53723733531797e-15)
link_14.Initialize(body_74,body_30,False,cA,cB,dA,dB)
link_14.SetName("Parallel12")
exported_items.append(link_14)


# Mate constraint: Coincident26 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-6/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link3-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.406892479516308,0.0110000000000009,-0.364294939392502)
cB = chrono.ChVectorD(0.39742052469723,0.0110000000000006,-0.361088372395645)
dA = chrono.ChVectorD(0.320656699685579,-4.46402174484698e-16,0.947195481907907)
dB = chrono.ChVectorD(0.320656699685579,-2.87038911158296e-14,0.947195481907907)
link_15.Initialize(body_58,body_35,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident26")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.406892479516308,0.0110000000000009,-0.364294939392502)
dA = chrono.ChVectorD(0.320656699685579,-4.46402174484698e-16,0.947195481907907)
cB = chrono.ChVectorD(0.39742052469723,0.0110000000000006,-0.361088372395645)
dB = chrono.ChVectorD(0.320656699685579,-2.87038911158296e-14,0.947195481907907)
link_16.Initialize(body_58,body_35,False,cA,cB,dA,dB)
link_16.SetName("Coincident26")
exported_items.append(link_16)


# Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_58 , SW name: robo_leg_link-6/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link3-3 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.398949935109913,0.0445000000000009,-0.372163610713153)
dA = chrono.ChVectorD(0.320656699685579,-4.46402174484698e-16,0.947195481907907)
cB = chrono.ChVectorD(0.40215650210677,0.0445000000000005,-0.362691655894072)
dB = chrono.ChVectorD(-0.320656699685579,2.87038911158296e-14,-0.947195481907907)
link_17.SetFlipped(True)
link_17.Initialize(body_58,body_35,False,cA,cB,dA,dB)
link_17.SetName("Concentric1")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateGeneric()
link_18.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.398949935109913,0.0445000000000009,-0.372163610713153)
cB = chrono.ChVectorD(0.40215650210677,0.0445000000000005,-0.362691655894072)
dA = chrono.ChVectorD(0.320656699685579,-4.46402174484698e-16,0.947195481907907)
dB = chrono.ChVectorD(-0.320656699685579,2.87038911158296e-14,-0.947195481907907)
link_18.Initialize(body_58,body_35,False,cA,cB,dA,dB)
link_18.SetName("Concentric1")
exported_items.append(link_18)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-7/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_71 , SW name: robo_leg_link-8/link3-3 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.538461631432825,0.0444999999999987,-0.42752417204033)
dA = chrono.ChVectorD(0.201765982465746,-2.48921253979499e-14,0.979433759025914)
cB = chrono.ChVectorD(0.540479291257482,0.0444999999999984,-0.417729834450071)
dB = chrono.ChVectorD(-0.201765982465746,2.46238215003321e-14,-0.979433759025914)
link_19.SetFlipped(True)
link_19.Initialize(body_29,body_71,False,cA,cB,dA,dB)
link_19.SetName("Concentric2")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateGeneric()
link_20.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.538461631432825,0.0444999999999987,-0.42752417204033)
cB = chrono.ChVectorD(0.540479291257482,0.0444999999999984,-0.417729834450071)
dA = chrono.ChVectorD(0.201765982465746,-2.48921253979499e-14,0.979433759025914)
dB = chrono.ChVectorD(-0.201765982465746,2.46238215003321e-14,-0.979433759025914)
link_20.Initialize(body_29,body_71,False,cA,cB,dA,dB)
link_20.SetName("Concentric2")
exported_items.append(link_20)


# Mate constraint: Coincident27 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_29 , SW name: robo_leg_link-7/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_71 , SW name: robo_leg_link-8/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.545376460052611,0.0109999999999983,-0.4187386643624)
cB = chrono.ChVectorD(0.535582122462352,0.0109999999999985,-0.416721004537743)
dA = chrono.ChVectorD(0.201765982465746,-2.48921253979499e-14,0.979433759025914)
dB = chrono.ChVectorD(0.201765982465746,-2.46238215003321e-14,0.979433759025914)
link_21.Initialize(body_29,body_71,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident27")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.545376460052611,0.0109999999999983,-0.4187386643624)
dA = chrono.ChVectorD(0.201765982465746,-2.48921253979499e-14,0.979433759025914)
cB = chrono.ChVectorD(0.535582122462352,0.0109999999999985,-0.416721004537743)
dB = chrono.ChVectorD(0.201765982465746,-2.46238215003321e-14,0.979433759025914)
link_22.Initialize(body_29,body_71,False,cA,cB,dA,dB)
link_22.SetName("Coincident27")
exported_items.append(link_22)


# Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_69 , SW name: robo_leg_link-8/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_78 , SW name: robo_leg_link-9/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.683107485499833,0.0444999999999965,-0.467187299299672)
dA = chrono.ChVectorD(0.12094654366943,-2.33655687390903e-14,0.992659021806793)
cB = chrono.ChVectorD(0.684316950936527,0.0444999999999963,-0.457260709081604)
dB = chrono.ChVectorD(-0.12094654366943,2.32221649317429e-14,-0.992659021806793)
link_23.SetFlipped(True)
link_23.Initialize(body_69,body_78,False,cA,cB,dA,dB)
link_23.SetName("Concentric3")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.683107485499833,0.0444999999999965,-0.467187299299672)
cB = chrono.ChVectorD(0.684316950936527,0.0444999999999963,-0.457260709081604)
dA = chrono.ChVectorD(0.12094654366943,-2.33655687390903e-14,0.992659021806793)
dB = chrono.ChVectorD(-0.12094654366943,2.32221649317429e-14,-0.992659021806793)
link_24.Initialize(body_69,body_78,False,cA,cB,dA,dB)
link_24.SetName("Concentric3")
exported_items.append(link_24)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_69 , SW name: robo_leg_link-8/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_78 , SW name: robo_leg_link-9/link3-3 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.689280246045561,0.0109999999999962,-0.457865441799952)
cB = chrono.ChVectorD(0.679353655827493,0.0109999999999964,-0.456655976363258)
dA = chrono.ChVectorD(0.12094654366943,-2.33655687390903e-14,0.992659021806793)
dB = chrono.ChVectorD(0.12094654366943,-2.32221649317429e-14,0.992659021806793)
link_25.Initialize(body_69,body_78,False,cA,cB,dB)
link_25.SetDistance(0)
link_25.SetName("Coincident28")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.689280246045561,0.0109999999999962,-0.457865441799952)
dA = chrono.ChVectorD(0.12094654366943,-2.33655687390903e-14,0.992659021806793)
cB = chrono.ChVectorD(0.679353655827493,0.0109999999999964,-0.456655976363258)
dB = chrono.ChVectorD(0.12094654366943,-2.32221649317429e-14,0.992659021806793)
link_26.Initialize(body_69,body_78,False,cA,cB,dA,dB)
link_26.SetName("Coincident28")
exported_items.append(link_26)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_72 , SW name: robo_leg_link-9/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: robo_leg_link-10/link3-3 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.836640634965183,0.0109999999999942,-0.488070828194635)
cB = chrono.ChVectorD(0.826902377381508,0.0109999999999942,-0.485797869938528)
dA = chrono.ChVectorD(0.2272958256107,-2.73207382643174e-14,0.973825758367456)
dB = chrono.ChVectorD(0.2272958256107,-2.71912122447778e-14,0.973825758367456)
link_27.Initialize(body_72,body_11,False,cA,cB,dB)
link_27.SetDistance(0)
link_27.SetName("Coincident29")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.836640634965183,0.0109999999999942,-0.488070828194635)
dA = chrono.ChVectorD(0.2272958256107,-2.73207382643174e-14,0.973825758367456)
cB = chrono.ChVectorD(0.826902377381508,0.0109999999999942,-0.485797869938528)
dB = chrono.ChVectorD(0.2272958256107,-2.71912122447778e-14,0.973825758367456)
link_28.Initialize(body_72,body_11,False,cA,cB,dA,dB)
link_28.SetName("Coincident29")
exported_items.append(link_28)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_72 , SW name: robo_leg_link-9/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: robo_leg_link-10/link3-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.82949854791724,0.0444999999999945,-0.496672606650255)
dA = chrono.ChVectorD(0.2272958256107,-2.73207382643174e-14,0.973825758367456)
cB = chrono.ChVectorD(0.831771506173346,0.0444999999999941,-0.486934349066581)
dB = chrono.ChVectorD(-0.2272958256107,2.71912122447778e-14,-0.973825758367456)
link_29.SetFlipped(True)
link_29.Initialize(body_72,body_11,False,cA,cB,dA,dB)
link_29.SetName("Concentric4")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.82949854791724,0.0444999999999945,-0.496672606650255)
cB = chrono.ChVectorD(0.831771506173346,0.0444999999999941,-0.486934349066581)
dA = chrono.ChVectorD(0.2272958256107,-2.73207382643174e-14,0.973825758367456)
dB = chrono.ChVectorD(-0.2272958256107,2.71912122447778e-14,-0.973825758367456)
link_30.Initialize(body_72,body_11,False,cA,cB,dA,dB)
link_30.SetName("Concentric4")
exported_items.append(link_30)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-10/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-11/link3-3 ,  SW ref.type:2 (2)

link_31 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.957219721264854,0.0110000000000008,-0.553431571701159)
cB = chrono.ChVectorD(0.957981122990733,0.011000000000022,-0.543460600464022)
dA = chrono.ChVectorD(0.997097123713768,5.66676335485757e-17,-0.0761401725880024)
dB = chrono.ChVectorD(0.997097123713768,2.54402980163585e-14,-0.0761401725880024)
link_31.Initialize(body_10,body_39,False,cA,cB,dB)
link_31.SetDistance(0)
link_31.SetName("Coincident30")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.957219721264854,0.0110000000000008,-0.553431571701159)
dA = chrono.ChVectorD(0.997097123713768,5.66676335485757e-17,-0.0761401725880024)
cB = chrono.ChVectorD(0.957981122990733,0.011000000000022,-0.543460600464022)
dB = chrono.ChVectorD(0.997097123713768,2.54402980163585e-14,-0.0761401725880024)
link_32.Initialize(body_10,body_39,False,cA,cB,dA,dB)
link_32.SetName("Coincident30")
exported_items.append(link_32)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: robo_leg_link-10/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-11/link3-3 ,  SW ref.type:2 (2)

link_33 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.947629450890657,0.0445000000000008,-0.54768468435671)
dA = chrono.ChVectorD(0.997097123713768,5.66676335485757e-17,-0.0761401725880024)
cB = chrono.ChVectorD(0.957600422127792,0.044500000000022,-0.548446086082591)
dB = chrono.ChVectorD(-0.997097123713768,-2.54402980163585e-14,0.0761401725880024)
link_33.SetFlipped(True)
link_33.Initialize(body_10,body_39,False,cA,cB,dA,dB)
link_33.SetName("Concentric5")
exported_items.append(link_33)

link_34 = chrono.ChLinkMateGeneric()
link_34.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.947629450890657,0.0445000000000008,-0.54768468435671)
cB = chrono.ChVectorD(0.957600422127792,0.044500000000022,-0.548446086082591)
dA = chrono.ChVectorD(0.997097123713768,5.66676335485757e-17,-0.0761401725880024)
dB = chrono.ChVectorD(-0.997097123713768,-2.54402980163585e-14,0.0761401725880024)
link_34.Initialize(body_10,body_39,False,cA,cB,dA,dB)
link_34.SetName("Concentric5")
exported_items.append(link_34)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-11/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_79 , SW name: robo_leg_link-12/link3-3 ,  SW ref.type:2 (2)

link_35 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.01082737024539,0.0445000000000009,-0.673139600705434)
dA = chrono.ChVectorD(0.991980115225096,-2.20888122607714e-16,-0.126394030705589)
cB = chrono.ChVectorD(1.02074717139765,0.0444999999999939,-0.674403541012488)
dB = chrono.ChVectorD(-0.991980115225096,1.79347277769655e-14,0.126394030705589)
link_35.SetFlipped(True)
link_35.Initialize(body_41,body_79,False,cA,cB,dA,dB)
link_35.SetName("Concentric6")
exported_items.append(link_35)

link_36 = chrono.ChLinkMateGeneric()
link_36.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.01082737024539,0.0445000000000009,-0.673139600705434)
cB = chrono.ChVectorD(1.02074717139765,0.0444999999999939,-0.674403541012488)
dA = chrono.ChVectorD(0.991980115225096,-2.20888122607714e-16,-0.126394030705589)
dB = chrono.ChVectorD(-0.991980115225096,1.79347277769655e-14,0.126394030705589)
link_36.Initialize(body_41,body_79,False,cA,cB,dA,dB)
link_36.SetName("Concentric6")
exported_items.append(link_36)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_41 , SW name: robo_leg_link-11/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_79 , SW name: robo_leg_link-12/link3-3 ,  SW ref.type:2 (2)

link_37 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.01019540009187,0.0110000000000009,-0.678099501281559)
cB = chrono.ChVectorD(1.01145934039892,0.010999999999994,-0.668179700129308)
dA = chrono.ChVectorD(-0.991980115225096,2.20888122607714e-16,0.126394030705589)
dB = chrono.ChVectorD(-0.991980115225096,1.79347277769655e-14,0.126394030705589)
link_37.Initialize(body_41,body_79,False,cA,cB,dB)
link_37.SetDistance(0)
link_37.SetName("Coincident31")
exported_items.append(link_37)

link_38 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.01019540009187,0.0110000000000009,-0.678099501281559)
dA = chrono.ChVectorD(-0.991980115225096,2.20888122607714e-16,0.126394030705589)
cB = chrono.ChVectorD(1.01145934039892,0.010999999999994,-0.668179700129308)
dB = chrono.ChVectorD(-0.991980115225096,1.79347277769655e-14,0.126394030705589)
link_38.Initialize(body_41,body_79,False,cA,cB,dA,dB)
link_38.SetName("Coincident31")
exported_items.append(link_38)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-1/link3-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_80 , SW name: robo_leg_link-12/link3-2 ,  SW ref.type:2 (2)

link_39 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.927018529676098,0.0444999999999981,-0.782183943761314)
dA = chrono.ChVectorD(0.163870894632821,-2.37541467977091e-14,0.986481793999382)
cB = chrono.ChVectorD(0.928657238622426,0.0444999999999979,-0.77231912582132)
dB = chrono.ChVectorD(-0.16387089463282,2.3995282395888e-14,-0.986481793999382)
link_39.SetFlipped(True)
link_39.Initialize(body_45,body_80,False,cA,cB,dA,dB)
link_39.SetName("Concentric7")
exported_items.append(link_39)

link_40 = chrono.ChLinkMateGeneric()
link_40.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.927018529676098,0.0444999999999981,-0.782183943761314)
cB = chrono.ChVectorD(0.928657238622426,0.0444999999999979,-0.77231912582132)
dA = chrono.ChVectorD(0.163870894632821,-2.37541467977091e-14,0.986481793999382)
dB = chrono.ChVectorD(-0.16387089463282,2.3995282395888e-14,-0.986481793999382)
link_40.Initialize(body_45,body_80,False,cA,cB,dA,dB)
link_40.SetName("Concentric7")
exported_items.append(link_40)


# Mate constraint: Coincident32 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_45 , SW name: robo_leg_link-1/link3-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_80 , SW name: robo_leg_link-12/link3-2 ,  SW ref.type:2 (2)

link_41 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.933589647592423,0.0109999999999978,-0.773138480294485)
cB = chrono.ChVectorD(0.923724829652429,0.010999999999998,-0.771499771348156)
dA = chrono.ChVectorD(0.163870894632821,-2.37541467977091e-14,0.986481793999382)
dB = chrono.ChVectorD(0.16387089463282,-2.3995282395888e-14,0.986481793999382)
link_41.Initialize(body_45,body_80,False,cA,cB,dB)
link_41.SetDistance(0)
link_41.SetName("Coincident32")
exported_items.append(link_41)

link_42 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.933589647592423,0.0109999999999978,-0.773138480294485)
dA = chrono.ChVectorD(0.163870894632821,-2.37541467977091e-14,0.986481793999382)
cB = chrono.ChVectorD(0.923724829652429,0.010999999999998,-0.771499771348156)
dB = chrono.ChVectorD(0.16387089463282,-2.3995282395888e-14,0.986481793999382)
link_42.Initialize(body_45,body_80,False,cA,cB,dA,dB)
link_42.SetName("Coincident32")
exported_items.append(link_42)


# Mate constraint: Concentric8 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_49 , SW name: robo_leg_link-1/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-2/link3-3 ,  SW ref.type:2 (2)

link_43 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.779454677242697,0.044500000000001,-0.767512100907754)
dA = chrono.ChVectorD(-0.179394001775259,1.85037170770859e-16,-0.983777308198892)
cB = chrono.ChVectorD(0.777660737224946,0.0445000000000011,-0.777349873989742)
dB = chrono.ChVectorD(0.179394001775259,-2.40455803416732e-14,0.983777308198892)
link_43.SetFlipped(True)
link_43.Initialize(body_49,body_20,False,cA,cB,dA,dB)
link_43.SetName("Concentric8")
exported_items.append(link_43)

link_44 = chrono.ChLinkMateGeneric()
link_44.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.779454677242697,0.044500000000001,-0.767512100907754)
cB = chrono.ChVectorD(0.777660737224946,0.0445000000000011,-0.777349873989742)
dA = chrono.ChVectorD(-0.179394001775259,1.85037170770859e-16,-0.983777308198892)
dB = chrono.ChVectorD(0.179394001775259,-2.40455803416732e-14,0.983777308198892)
link_44.Initialize(body_49,body_20,False,cA,cB,dA,dB)
link_44.SetName("Concentric8")
exported_items.append(link_44)


# Mate constraint: Coincident33 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_49 , SW name: robo_leg_link-1/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-2/link3-3 ,  SW ref.type:2 (2)

link_45 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.774535790701702,0.011000000000001,-0.766615130898878)
cB = chrono.ChVectorD(0.784373563783692,0.0110000000000007,-0.76840907091663)
dA = chrono.ChVectorD(0.179394001775259,-1.85037170770859e-16,0.983777308198892)
dB = chrono.ChVectorD(0.179394001775259,-2.40455803416732e-14,0.983777308198892)
link_45.Initialize(body_49,body_20,False,cA,cB,dB)
link_45.SetDistance(0)
link_45.SetName("Coincident33")
exported_items.append(link_45)

link_46 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.774535790701702,0.011000000000001,-0.766615130898878)
dA = chrono.ChVectorD(0.179394001775259,-1.85037170770859e-16,0.983777308198892)
cB = chrono.ChVectorD(0.784373563783692,0.0110000000000007,-0.76840907091663)
dB = chrono.ChVectorD(0.179394001775259,-2.40455803416732e-14,0.983777308198892)
link_46.Initialize(body_49,body_20,False,cA,cB,dA,dB)
link_46.SetName("Coincident33")
exported_items.append(link_46)


# Mate constraint: Concentric9 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-2/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-3/link3-3 ,  SW ref.type:2 (2)

link_47 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.631487032143667,0.0445000000000009,-0.74086873344286)
dA = chrono.ChVectorD(-0.0713737524527569,1.5728159515523e-16,-0.997449641566336)
cB = chrono.ChVectorD(0.630773294619141,0.0445000000000035,-0.750843229858522)
dB = chrono.ChVectorD(0.0713737524527567,-2.20194233217323e-14,0.997449641566336)
link_47.SetFlipped(True)
link_47.Initialize(body_19,body_27,False,cA,cB,dA,dB)
link_47.SetName("Concentric9")
exported_items.append(link_47)

link_48 = chrono.ChLinkMateGeneric()
link_48.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.631487032143667,0.0445000000000009,-0.74086873344286)
cB = chrono.ChVectorD(0.630773294619141,0.0445000000000035,-0.750843229858522)
dA = chrono.ChVectorD(-0.0713737524527569,1.5728159515523e-16,-0.997449641566336)
dB = chrono.ChVectorD(0.0713737524527567,-2.20194233217323e-14,0.997449641566336)
link_48.Initialize(body_19,body_27,False,cA,cB,dA,dB)
link_48.SetName("Concentric9")
exported_items.append(link_48)


# Mate constraint: Coincident34 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_19 , SW name: robo_leg_link-2/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-3/link3-3 ,  SW ref.type:2 (2)

link_49 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.626499783935835,0.0110000000000009,-0.740511864680597)
cB = chrono.ChVectorD(0.6364742803515,0.0110000000000032,-0.741225602205123)
dA = chrono.ChVectorD(0.0713737524527569,-1.5728159515523e-16,0.997449641566336)
dB = chrono.ChVectorD(0.0713737524527567,-2.20194233217323e-14,0.997449641566336)
link_49.Initialize(body_19,body_27,False,cA,cB,dB)
link_49.SetDistance(0)
link_49.SetName("Coincident34")
exported_items.append(link_49)

link_50 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.626499783935835,0.0110000000000009,-0.740511864680597)
dA = chrono.ChVectorD(0.0713737524527569,-1.5728159515523e-16,0.997449641566336)
cB = chrono.ChVectorD(0.6364742803515,0.0110000000000032,-0.741225602205123)
dB = chrono.ChVectorD(0.0713737524527567,-2.20194233217323e-14,0.997449641566336)
link_50.Initialize(body_19,body_27,False,cA,cB,dA,dB)
link_50.SetName("Coincident34")
exported_items.append(link_50)


# Mate constraint: Concentric10 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_25 , SW name: robo_leg_link-3/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_55 , SW name: robo_leg_link-4/link3-3 ,  SW ref.type:2 (2)

link_51 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.483356549045708,0.0445000000000009,-0.718463703722316)
dA = chrono.ChVectorD(-0.0490480253759892,3.23815048849004e-16,-0.998796421302518)
cB = chrono.ChVectorD(0.482866068791946,0.0445000000000079,-0.728451667935342)
dB = chrono.ChVectorD(0.0490480253759894,1.0274188907052e-14,0.998796421302518)
link_51.SetFlipped(True)
link_51.Initialize(body_25,body_55,False,cA,cB,dA,dB)
link_51.SetName("Concentric10")
exported_items.append(link_51)

link_52 = chrono.ChLinkMateGeneric()
link_52.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.483356549045708,0.0445000000000009,-0.718463703722316)
cB = chrono.ChVectorD(0.482866068791946,0.0445000000000079,-0.728451667935342)
dA = chrono.ChVectorD(-0.0490480253759892,3.23815048849004e-16,-0.998796421302518)
dB = chrono.ChVectorD(0.0490480253759894,1.0274188907052e-14,0.998796421302518)
link_52.Initialize(body_25,body_55,False,cA,cB,dA,dB)
link_52.SetName("Concentric10")
exported_items.append(link_52)


# Mate constraint: Coincident35 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_25 , SW name: robo_leg_link-3/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_55 , SW name: robo_leg_link-4/link3-3 ,  SW ref.type:2 (2)

link_53 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.478362566939195,0.0110000000000009,-0.718218463595436)
cB = chrono.ChVectorD(0.48835053115222,0.0110000000000082,-0.718708943849196)
dA = chrono.ChVectorD(0.0490480253759892,-3.23815048849004e-16,0.998796421302518)
dB = chrono.ChVectorD(0.0490480253759894,1.0274188907052e-14,0.998796421302518)
link_53.Initialize(body_25,body_55,False,cA,cB,dB)
link_53.SetDistance(0)
link_53.SetName("Coincident35")
exported_items.append(link_53)

link_54 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.478362566939195,0.0110000000000009,-0.718218463595436)
dA = chrono.ChVectorD(0.0490480253759892,-3.23815048849004e-16,0.998796421302518)
cB = chrono.ChVectorD(0.48835053115222,0.0110000000000082,-0.718708943849196)
dB = chrono.ChVectorD(0.0490480253759894,1.0274188907052e-14,0.998796421302518)
link_54.Initialize(body_25,body_55,False,cA,cB,dA,dB)
link_54.SetName("Coincident35")
exported_items.append(link_54)


# Mate constraint: Concentric11 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-4/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-5/link3-3 ,  SW ref.type:2 (2)

link_55 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.378820620539651,0.0445000000000009,-0.631076261888822)
dA = chrono.ChVectorD(-0.958354197772572,2.19731640290396e-16,-0.285582267677267)
cB = chrono.ChVectorD(0.369237078561927,0.0445000000000064,-0.633932084565593)
dB = chrono.ChVectorD(0.958354197772572,-2.56692815151875e-14,0.285582267677268)
link_55.SetFlipped(True)
link_55.Initialize(body_51,body_3,False,cA,cB,dA,dB)
link_55.SetName("Concentric11")
exported_items.append(link_55)

link_56 = chrono.ChLinkMateGeneric()
link_56.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.378820620539651,0.0445000000000009,-0.631076261888822)
cB = chrono.ChVectorD(0.369237078561927,0.0445000000000064,-0.633932084565593)
dA = chrono.ChVectorD(-0.958354197772572,2.19731640290396e-16,-0.285582267677267)
dB = chrono.ChVectorD(0.958354197772572,-2.56692815151875e-14,0.285582267677268)
link_56.Initialize(body_51,body_3,False,cA,cB,dA,dB)
link_56.SetName("Concentric11")
exported_items.append(link_56)


# Mate constraint: Coincident36 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_51 , SW name: robo_leg_link-4/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-5/link3-3 ,  SW ref.type:2 (2)

link_57 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.377392709201264,0.0110000000000009,-0.626284490899959)
cB = chrono.ChVectorD(0.380248531878038,0.0110000000000062,-0.635868032877684)
dA = chrono.ChVectorD(0.958354197772572,-2.19731640290396e-16,0.285582267677267)
dB = chrono.ChVectorD(0.958354197772572,-2.56692815151875e-14,0.285582267677268)
link_57.Initialize(body_51,body_3,False,cA,cB,dB)
link_57.SetDistance(0)
link_57.SetName("Coincident36")
exported_items.append(link_57)

link_58 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.377392709201264,0.0110000000000009,-0.626284490899959)
dA = chrono.ChVectorD(0.958354197772572,-2.19731640290396e-16,0.285582267677267)
cB = chrono.ChVectorD(0.380248531878038,0.0110000000000062,-0.635868032877684)
dB = chrono.ChVectorD(0.958354197772572,-2.56692815151875e-14,0.285582267677268)
link_58.Initialize(body_51,body_3,False,cA,cB,dA,dB)
link_58.SetName("Coincident36")
exported_items.append(link_58)


# Mate constraint: Coincident38 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: robo_leg_link-5/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_59 , SW name: robo_leg_link-6/link3-3 ,  SW ref.type:2 (2)

link_59 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.347708590880575,0.0110000000000036,-0.480209214190937)
cB = chrono.ChVectorD(0.349383533934564,0.0110000000000009,-0.490067944624715)
dA = chrono.ChVectorD(-0.985873043377749,2.3735643080632e-14,-0.167494305399006)
dB = chrono.ChVectorD(-0.985873043377749,9.25185853854298e-17,-0.167494305399006)
link_59.Initialize(body_2,body_59,False,cA,cB,dB)
link_59.SetDistance(0)
link_59.SetName("Coincident38")
exported_items.append(link_59)

link_60 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.347708590880575,0.0110000000000036,-0.480209214190937)
dA = chrono.ChVectorD(-0.985873043377749,2.3735643080632e-14,-0.167494305399006)
cB = chrono.ChVectorD(0.349383533934564,0.0110000000000009,-0.490067944624715)
dB = chrono.ChVectorD(-0.985873043377749,9.25185853854298e-17,-0.167494305399006)
link_60.Initialize(body_2,body_59,False,cA,cB,dA,dB)
link_60.SetName("Coincident38")
exported_items.append(link_60)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: robo_leg_link-5/link3-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_59 , SW name: robo_leg_link-6/link3-3 ,  SW ref.type:2 (2)

link_61 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.358404792841348,0.0445000000000035,-0.483463636353835)
dA = chrono.ChVectorD(-0.985873043377749,2.3735643080632e-14,-0.167494305399006)
cB = chrono.ChVectorD(0.348546062407569,0.0445000000000008,-0.485138579407826)
dB = chrono.ChVectorD(0.985873043377749,-9.25185853854298e-17,0.167494305399006)
link_61.SetFlipped(True)
link_61.Initialize(body_2,body_59,False,cA,cB,dA,dB)
link_61.SetName("Concentric16")
exported_items.append(link_61)

link_62 = chrono.ChLinkMateGeneric()
link_62.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.358404792841348,0.0445000000000035,-0.483463636353835)
cB = chrono.ChVectorD(0.348546062407569,0.0445000000000008,-0.485138579407826)
dA = chrono.ChVectorD(-0.985873043377749,2.3735643080632e-14,-0.167494305399006)
dB = chrono.ChVectorD(0.985873043377749,-9.25185853854298e-17,0.167494305399006)
link_62.Initialize(body_2,body_59,False,cA,cB,dA,dB)
link_62.SetName("Concentric16")
exported_items.append(link_62)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: robo_leg_link-5/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.359772792995903,0.0169999999999904,-0.551811124167946)
dA = chrono.ChVectorD(0.101111083328197,-3.43267081426291e-14,-0.994875142331036)
cB = chrono.ChVectorD(0.360581681662527,0.0170000000000253,-0.559770125306592)
dB = chrono.ChVectorD(-0.101111083328199,7.27196081129478e-15,0.994875142331036)
link_1.SetFlipped(True)
link_1.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.359772792995903,0.0169999999999904,-0.551811124167946)
cB = chrono.ChVectorD(0.360581681662527,0.0170000000000253,-0.559770125306592)
dA = chrono.ChVectorD(0.101111083328197,-3.43267081426291e-14,-0.994875142331036)
dB = chrono.ChVectorD(-0.101111083328199,7.27196081129478e-15,0.994875142331036)
link_2.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.336934815233763,0.0110000000000245,-0.590027148526007)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dA = chrono.ChVectorD(-2.73531197692023e-14,1,-6.37684349769074e-15)
dB = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
link_3.Initialize(body_4,body_5,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.336934815233763,0.0110000000000245,-0.590027148526007)
dA = chrono.ChVectorD(-2.73531197692023e-14,1,-6.37684349769074e-15)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dB = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: robo_leg_link-5/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.360581681662529,0.0169999999999902,-0.559770125306594)
cB = chrono.ChVectorD(0.358590936502723,0.0149990000000253,-0.559972448584332)
dA = chrono.ChVectorD(0.101111083328197,-3.43267081426291e-14,-0.994875142331036)
dB = chrono.ChVectorD(-0.101111083328199,7.27196081129478e-15,0.994875142331036)
link_5.Initialize(body_1,body_4,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.360581681662529,0.0169999999999902,-0.559770125306594)
dA = chrono.ChVectorD(0.101111083328197,-3.43267081426291e-14,-0.994875142331036)
cB = chrono.ChVectorD(0.358590936502723,0.0149990000000253,-0.559972448584332)
dB = chrono.ChVectorD(-0.101111083328199,7.27196081129478e-15,0.994875142331036)
link_6.SetFlipped(True)
link_6.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.341284618669869,0.0633100000000446,-0.576364069841546)
dA = chrono.ChVectorD(-0.994875142331036,-2.8199664825479e-14,-0.101111083328188)
cB = chrono.ChVectorD(0.389138347893264,0.0610000000000073,-0.548176174128343)
dB = chrono.ChVectorD(0.994875142331037,2.66869859544272e-14,0.101111083328187)
link_7.SetFlipped(True)
link_7.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.336934815233763,0.0110000000000245,-0.590027148526007)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dA = chrono.ChVectorD(-2.73531197692023e-14,1,-6.37684349769074e-15)
dB = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
link_8.Initialize(body_4,body_5,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.336934815233763,0.0110000000000245,-0.590027148526007)
dA = chrono.ChVectorD(-2.73531197692023e-14,1,-6.37684349769074e-15)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dB = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
link_9.SetFlipped(True)
link_9.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.340431739697409,0.0789500000000558,-0.594374830169839)
dA = chrono.ChVectorD(3.82471831983366e-14,-1,-1.92831861589582e-14)
cB = chrono.ChVectorD(0.362483060860357,0.0780000000000065,-0.558698382265537)
dB = chrono.ChVectorD(-2.61457522299224e-14,1,-6.70297151117438e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.362483060860357,0.078950000000056,-0.558698382265533)
dA = chrono.ChVectorD(3.82471831983366e-14,-1,-1.92831861589582e-14)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dB = chrono.ChVectorD(-2.61457522299224e-14,1,-6.70297151117438e-15)
link_11.SetFlipped(True)
link_11.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.362483060860357,0.078950000000056,-0.558698382265533)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dA = chrono.ChVectorD(3.82471831983366e-14,-1,-1.92831861589582e-14)
dB = chrono.ChVectorD(-2.61457522299224e-14,1,-6.70297151117438e-15)
link_12.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: robo_leg_link-5/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.362483060860357,0.0110000000000254,-0.558698382265537)
dA = chrono.ChVectorD(-2.73531197692023e-14,1,-6.37684349769074e-15)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dB = chrono.ChVectorD(-2.61457522299224e-14,1,-6.70297151117438e-15)
link_13.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.362483060860357,0.0110000000000254,-0.558698382265537)
cB = chrono.ChVectorD(0.362483060860359,0.0110000000000065,-0.558698382265536)
dA = chrono.ChVectorD(-2.73531197692023e-14,1,-6.37684349769074e-15)
dB = chrono.ChVectorD(-2.61457522299224e-14,1,-6.70297151117438e-15)
link_14.Initialize(body_4,body_5,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-5/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.353768838954101,0.0395000000000065,-0.522405993355208)
cB = chrono.ChVectorD(0.35975432675149,0.0395000000000009,-0.53184918923459)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.45716771982052e-16)
link_15.Initialize(body_5,body_7,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.353768838954101,0.0395000000000065,-0.522405993355208)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
cB = chrono.ChVectorD(0.35975432675149,0.0395000000000009,-0.53184918923459)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.45716771982052e-16)
link_16.Initialize(body_5,body_7,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-5/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.361248531343304,0.0395000000000062,-0.596001882009147)
cB = chrono.ChVectorD(0.36521179443061,0.0395000000000064,-0.585547575351232)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
dB = chrono.ChVectorD(2.61735078055381e-14,-1,6.74460487459783e-15)
link_17.Initialize(body_5,body_6,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.361248531343304,0.0395000000000062,-0.596001882009147)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
cB = chrono.ChVectorD(0.36521179443061,0.0395000000000064,-0.585547575351232)
dB = chrono.ChVectorD(2.61735078055381e-14,-1,6.74460487459783e-15)
link_18.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-5/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: robo_leg_link-5/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.356569951682034,0.0495000000000009,-0.500516841022566)
cB = chrono.ChVectorD(0.357912979878125,0.049500000000004,-0.510420581823819)
dA = chrono.ChVectorD(4.16333634234434e-17,1,1.45716771982052e-16)
dB = chrono.ChVectorD(2.04281036531029e-14,1,2.07750483482982e-14)
link_19.Initialize(body_7,body_2,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.356569951682034,0.0495000000000009,-0.500516841022566)
dA = chrono.ChVectorD(4.16333634234434e-17,1,1.45716771982052e-16)
cB = chrono.ChVectorD(0.357912979878125,0.049500000000004,-0.510420581823819)
dB = chrono.ChVectorD(2.04281036531029e-14,1,2.07750483482982e-14)
link_20.Initialize(body_7,body_2,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-5/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-5/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.368396172892584,0.0495000000000062,-0.616879923218467)
cB = chrono.ChVectorD(0.366462704346515,0.0495000000000059,-0.60711377669997)
dA = chrono.ChVectorD(-2.61735078055381e-14,1,-6.74460487459783e-15)
dB = chrono.ChVectorD(2.07334149848748e-14,1,2.07403538787787e-14)
link_21.Initialize(body_6,body_3,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.368396172892584,0.0495000000000062,-0.616879923218467)
dA = chrono.ChVectorD(-2.61735078055381e-14,1,-6.74460487459783e-15)
cB = chrono.ChVectorD(0.366462704346515,0.0495000000000059,-0.60711377669997)
dB = chrono.ChVectorD(2.07334149848748e-14,1,2.07403538787787e-14)
link_22.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: robo_leg_link-5/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: robo_leg_link-5/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.367890615684888,0.0495000000000063,-0.61190554768884)
dA = chrono.ChVectorD(2.61735078055381e-14,-1,6.74460487459783e-15)
cB = chrono.ChVectorD(0.367890615684901,0.049500000000006,-0.611905547688833)
dB = chrono.ChVectorD(-2.07334149848748e-14,-1,-2.07403538787787e-14)
link_23.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.367890615684888,0.0495000000000063,-0.61190554768884)
cB = chrono.ChVectorD(0.367890615684901,0.049500000000006,-0.611905547688833)
dA = chrono.ChVectorD(2.61735078055381e-14,-1,6.74460487459783e-15)
dB = chrono.ChVectorD(-2.07334149848748e-14,-1,-2.07403538787787e-14)
link_24.Initialize(body_6,body_3,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_7 , SW name: robo_leg_link-5/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: robo_leg_link-5/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.357075508351128,0.0495000000000009,-0.505491216606932)
dA = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.45716771982052e-16)
cB = chrono.ChVectorD(0.35707550835113,0.049500000000004,-0.50549121660693)
dB = chrono.ChVectorD(-2.04281036531029e-14,-1,-2.07750483482982e-14)
link_25.Initialize(body_7,body_2,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.357075508351128,0.0495000000000009,-0.505491216606932)
cB = chrono.ChVectorD(0.35707550835113,0.049500000000004,-0.50549121660693)
dA = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.45716771982052e-16)
dB = chrono.ChVectorD(-2.04281036531029e-14,-1,-2.07750483482982e-14)
link_26.Initialize(body_7,body_2,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: robo_leg_link-5/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.365717351638318,0.0545000000000064,-0.590521950880851)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
cB = chrono.ChVectorD(0.365717351638305,0.0495000000000064,-0.590521950880859)
dB = chrono.ChVectorD(2.61735078055381e-14,-1,6.74460487459783e-15)
link_27.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.365717351638318,0.0545000000000064,-0.590521950880851)
cB = chrono.ChVectorD(0.365717351638305,0.0495000000000064,-0.590521950880859)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
dB = chrono.ChVectorD(2.61735078055381e-14,-1,6.74460487459783e-15)
link_28.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: robo_leg_link-5/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: robo_leg_link-5/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.359248770082397,0.0545000000000066,-0.526874813650223)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
cB = chrono.ChVectorD(0.359248770082396,0.0495000000000009,-0.526874813650224)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.45716771982052e-16)
link_29.Initialize(body_5,body_7,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.359248770082397,0.0545000000000066,-0.526874813650223)
cB = chrono.ChVectorD(0.359248770082396,0.0495000000000009,-0.526874813650224)
dA = chrono.ChVectorD(2.61457522299224e-14,-1,6.70297151117438e-15)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.45716771982052e-16)
link_30.Initialize(body_5,body_7,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: robo_leg_link-10/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.910309791149956,0.017000000000001,-0.511736456362463)
dA = chrono.ChVectorD(-0.960012070539745,-3.04686831320566e-14,0.279958611973254)
cB = chrono.ChVectorD(0.902629694585638,0.0170000000000007,-0.509496787466677)
dB = chrono.ChVectorD(0.960012070539746,3.04131719808254e-14,-0.279958611973253)
link_1.SetFlipped(True)
link_1.Initialize(body_8,body_14,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.910309791149956,0.017000000000001,-0.511736456362463)
cB = chrono.ChVectorD(0.902629694585638,0.0170000000000007,-0.509496787466677)
dA = chrono.ChVectorD(-0.960012070539745,-3.04686831320566e-14,0.279958611973254)
dB = chrono.ChVectorD(0.960012070539746,3.04131719808254e-14,-0.279958611973253)
link_2.Initialize(body_8,body_14,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.883469499798222,0.0110000000000003,-0.476216933474986)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dA = chrono.ChVectorD(-2.78631284711395e-14,1,-2.22044604925031e-16)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
link_3.Initialize(body_14,body_13,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.883469499798222,0.0110000000000003,-0.476216933474986)
dA = chrono.ChVectorD(-2.78631284711395e-14,1,-2.22044604925031e-16)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: robo_leg_link-10/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.902629694585638,0.0170000000000008,-0.509496787466677)
cB = chrono.ChVectorD(0.903189891768197,0.0149990000000008,-0.507575803313527)
dA = chrono.ChVectorD(-0.960012070539745,-3.04686831320566e-14,0.279958611973254)
dB = chrono.ChVectorD(0.960012070539746,3.04131719808254e-14,-0.279958611973253)
link_5.Initialize(body_8,body_14,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.902629694585638,0.0170000000000008,-0.509496787466677)
dA = chrono.ChVectorD(-0.960012070539745,-3.04686831320566e-14,0.279958611973254)
cB = chrono.ChVectorD(0.903189891768197,0.0149990000000008,-0.507575803313527)
dB = chrono.ChVectorD(0.960012070539746,3.04131719808254e-14,-0.279958611973253)
link_6.SetFlipped(True)
link_6.Initialize(body_8,body_14,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.894498489101888,0.0633100000000204,-0.485380006322571)
dA = chrono.ChVectorD(0.279958611973264,7.70217223333702e-15,0.960012070539742)
cB = chrono.ChVectorD(0.902649149745032,0.0610000000000008,-0.540317275970824)
dB = chrono.ChVectorD(-0.279958611973264,1.80411241501588e-16,-0.960012070539743)
link_7.SetFlipped(True)
link_7.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.883469499798222,0.0110000000000003,-0.476216933474986)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dA = chrono.ChVectorD(-2.78631284711395e-14,1,-2.22044604925031e-16)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
link_8.Initialize(body_14,body_13,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.883469499798222,0.0110000000000003,-0.476216933474986)
dA = chrono.ChVectorD(-2.78631284711395e-14,1,-2.22044604925031e-16)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.878126716790622,0.0789500000000314,-0.477824878161316)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.902908827911348,0.0780000000000008,-0.511661495434621)
dB = chrono.ChVectorD(3.12250225675825e-17,1,1.94289029309402e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.902908827911348,0.0789500000000314,-0.511661495434621)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dB = chrono.ChVectorD(3.12250225675825e-17,1,1.94289029309402e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.902908827911348,0.0789500000000314,-0.511661495434621)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(3.12250225675825e-17,1,1.94289029309402e-16)
link_12.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_14 , SW name: robo_leg_link-10/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.902908827911347,0.0110000000000008,-0.511661495434619)
dA = chrono.ChVectorD(-2.78631284711395e-14,1,-2.22044604925031e-16)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dB = chrono.ChVectorD(3.12250225675825e-17,1,1.94289029309402e-16)
link_13.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.902908827911347,0.0110000000000008,-0.511661495434619)
cB = chrono.ChVectorD(0.902908827911348,0.0110000000000008,-0.511661495434621)
dA = chrono.ChVectorD(-2.78631284711395e-14,1,-2.22044604925031e-16)
dB = chrono.ChVectorD(3.12250225675825e-17,1,1.94289029309402e-16)
link_14.Initialize(body_14,body_13,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-10/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.939817067430303,0.0395000000000009,-0.517216404242283)
cB = chrono.ChVectorD(0.928817153459191,0.0395000000000008,-0.519216879181126)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-2.22044604925031e-16)
link_15.Initialize(body_13,body_9,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.939817067430303,0.0395000000000009,-0.517216404242283)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.928817153459191,0.0395000000000008,-0.519216879181126)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-2.22044604925031e-16)
link_16.Initialize(body_13,body_9,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-10/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.868800174512125,0.0395000000000009,-0.496506465921561)
cB = chrono.ChVectorD(0.877000499348605,0.0395000000000008,-0.504106122026534)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-4.85722573273506e-17,-1,-1.94289029309402e-16)
link_17.Initialize(body_13,body_12,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.868800174512125,0.0395000000000009,-0.496506465921561)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.877000499348605,0.0395000000000008,-0.504106122026534)
dB = chrono.ChVectorD(-4.85722573273506e-17,-1,-1.94289029309402e-16)
link_18.Initialize(body_13,body_12,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_9 , SW name: robo_leg_link-10/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: robo_leg_link-10/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.959051534902335,0.0495000000000008,-0.528033821270817)
cB = chrono.ChVectorD(0.954632175206729,0.0495000000000008,-0.521648543298259)
dA = chrono.ChVectorD(4.16333634234434e-17,1,2.22044604925031e-16)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.94289029309402e-16)
link_19.Initialize(body_9,body_10,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.959051534902335,0.0495000000000008,-0.528033821270817)
dA = chrono.ChVectorD(4.16333634234434e-17,1,2.22044604925031e-16)
cB = chrono.ChVectorD(0.954632175206729,0.0495000000000008,-0.521648543298259)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.94289029309402e-16)
link_20.Initialize(body_9,body_10,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_12 , SW name: robo_leg_link-10/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: robo_leg_link-10/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.846766136895554,0.0495000000000008,-0.495289114817732)
cB = chrono.ChVectorD(0.856435323231041,0.0494999999999938,-0.497825396638191)
dA = chrono.ChVectorD(4.85722573273506e-17,1,1.94289029309402e-16)
dB = chrono.ChVectorD(2.14030182466018e-14,1,2.28428387316626e-14)
link_21.Initialize(body_12,body_11,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.846766136895554,0.0495000000000008,-0.495289114817732)
dA = chrono.ChVectorD(4.85722573273506e-17,1,1.94289029309402e-16)
cB = chrono.ChVectorD(0.856435323231041,0.0494999999999938,-0.497825396638191)
dB = chrono.ChVectorD(2.14030182466018e-14,1,2.28428387316626e-14)
link_22.Initialize(body_12,body_11,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_12 , SW name: robo_leg_link-10/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_11 , SW name: robo_leg_link-10/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.851566194439202,0.0495000000000008,-0.496688917510139)
dA = chrono.ChVectorD(-4.85722573273506e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.851566194439203,0.0494999999999939,-0.496688917510138)
dB = chrono.ChVectorD(-2.14030182466018e-14,-1,-2.28428387316626e-14)
link_23.Initialize(body_12,body_11,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.851566194439202,0.0495000000000008,-0.496688917510139)
cB = chrono.ChVectorD(0.851566194439203,0.0494999999999939,-0.496688917510138)
dA = chrono.ChVectorD(-4.85722573273506e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-2.14030182466018e-14,-1,-2.28428387316626e-14)
link_24.Initialize(body_12,body_11,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_9 , SW name: robo_leg_link-10/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: robo_leg_link-10/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.954251474343789,0.0495000000000008,-0.526634028916827)
dA = chrono.ChVectorD(-4.16333634234434e-17,-1,-2.22044604925031e-16)
cB = chrono.ChVectorD(0.954251474343789,0.0495000000000008,-0.526634028916827)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.94289029309402e-16)
link_25.Initialize(body_9,body_10,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.954251474343789,0.0495000000000008,-0.526634028916827)
cB = chrono.ChVectorD(0.954251474343789,0.0495000000000008,-0.526634028916827)
dA = chrono.ChVectorD(-4.16333634234434e-17,-1,-2.22044604925031e-16)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.94289029309402e-16)
link_26.Initialize(body_9,body_10,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: robo_leg_link-10/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.872200441804958,0.0545000000000008,-0.502706319334126)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.872200441804957,0.0495000000000008,-0.502706319334126)
dB = chrono.ChVectorD(-4.85722573273506e-17,-1,-1.94289029309402e-16)
link_27.Initialize(body_13,body_12,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.872200441804958,0.0545000000000008,-0.502706319334126)
cB = chrono.ChVectorD(0.872200441804957,0.0495000000000008,-0.502706319334126)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-4.85722573273506e-17,-1,-1.94289029309402e-16)
link_28.Initialize(body_13,body_12,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_13 , SW name: robo_leg_link-10/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: robo_leg_link-10/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.933617214017738,0.0545000000000008,-0.520616671535116)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.933617214017737,0.0495000000000008,-0.520616671535116)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-2.22044604925031e-16)
link_29.Initialize(body_13,body_9,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.933617214017738,0.0545000000000008,-0.520616671535116)
cB = chrono.ChVectorD(0.933617214017737,0.0495000000000008,-0.520616671535116)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-2.22044604925031e-16)
link_30.Initialize(body_13,body_9,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.697609916635232,0.0170000000000011,-0.758531603553132)
dA = chrono.ChVectorD(0.980189767119599,-3.01286773307652e-14,-0.198060648373234)
cB = chrono.ChVectorD(0.705451434772189,0.0170000000000008,-0.760116088740117)
dB = chrono.ChVectorD(-0.980189767119599,3.01633718002847e-14,0.198060648373234)
link_1.SetFlipped(True)
link_1.Initialize(body_16,body_15,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.697609916635232,0.0170000000000011,-0.758531603553132)
cB = chrono.ChVectorD(0.705451434772189,0.0170000000000008,-0.760116088740117)
dA = chrono.ChVectorD(0.980189767119599,-3.01286773307652e-14,-0.198060648373234)
dB = chrono.ChVectorD(-0.980189767119599,3.01633718002847e-14,0.198060648373234)
link_2.Initialize(body_16,body_15,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.727348031071837,0.0110000000000004,-0.791662892326063)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dA = chrono.ChVectorD(2.77659839564848e-14,1,2.92821322744885e-15)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
link_3.Initialize(body_15,body_17,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.727348031071837,0.0110000000000004,-0.791662892326063)
dA = chrono.ChVectorD(2.77659839564848e-14,1,2.92821322744885e-15)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_16 , SW name: robo_leg_link-2/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.705451434772189,0.0170000000000008,-0.760116088740117)
cB = chrono.ChVectorD(0.705055115414794,0.0149990000000009,-0.762077448464124)
dA = chrono.ChVectorD(0.980189767119599,-3.01286773307652e-14,-0.198060648373234)
dB = chrono.ChVectorD(-0.980189767119599,3.01633718002847e-14,0.198060648373234)
link_5.Initialize(body_16,body_15,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.705451434772189,0.0170000000000008,-0.760116088740117)
dA = chrono.ChVectorD(0.980189767119599,-3.01286773307652e-14,-0.198060648373234)
cB = chrono.ChVectorD(0.705055115414794,0.0149990000000009,-0.762077448464124)
dB = chrono.ChVectorD(-0.980189767119599,3.01633718002847e-14,0.198060648373234)
link_6.SetFlipped(True)
link_6.Initialize(body_16,body_15,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.71558608412724,0.0633100000000205,-0.783461848908119)
dA = chrono.ChVectorD(-0.198060648373245,7.9658502016855e-15,-0.980189767119597)
cB = chrono.ChVectorD(0.702834746043666,0.0610000000000009,-0.729406874646697)
dB = chrono.ChVectorD(0.198060648373245,-1.80411241501588e-16,0.980189767119597)
link_7.SetFlipped(True)
link_7.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.727348031071837,0.0110000000000004,-0.791662892326063)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dA = chrono.ChVectorD(2.77659839564848e-14,1,2.92821322744885e-15)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
link_8.Initialize(body_15,body_17,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.727348031071837,0.0110000000000004,-0.791662892326063)
dA = chrono.ChVectorD(2.77659839564848e-14,1,2.92821322744885e-15)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.732536304049402,0.0789500000000315,-0.78961042067197)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.704990870212469,0.0780000000000009,-0.7579826041992)
dB = chrono.ChVectorD(2.08166817117217e-17,1,1.66533453693773e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.704990870212469,0.0789500000000315,-0.7579826041992)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dB = chrono.ChVectorD(2.08166817117217e-17,1,1.66533453693773e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.704990870212469,0.0789500000000315,-0.7579826041992)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(2.08166817117217e-17,1,1.66533453693773e-16)
link_12.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_15 , SW name: robo_leg_link-2/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.70499087021247,0.0110000000000009,-0.757982604199202)
dA = chrono.ChVectorD(2.77659839564848e-14,1,2.92821322744885e-15)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dB = chrono.ChVectorD(2.08166817117217e-17,1,1.66533453693773e-16)
link_13.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.70499087021247,0.0110000000000009,-0.757982604199202)
cB = chrono.ChVectorD(0.704990870212469,0.0110000000000009,-0.7579826041992)
dA = chrono.ChVectorD(2.77659839564848e-14,1,2.92821322744885e-15)
dB = chrono.ChVectorD(2.08166817117217e-17,1,1.66533453693773e-16)
link_14.Initialize(body_15,body_17,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: robo_leg_link-2/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.667745797959266,0.0395000000000009,-0.755557784803092)
cB = chrono.ChVectorD(0.678537998782741,0.0395000000000009,-0.752637442894592)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.52655665885959e-16)
link_15.Initialize(body_17,body_18,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.667745797959266,0.0395000000000009,-0.755557784803092)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.678537998782741,0.0395000000000009,-0.752637442894592)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.52655665885959e-16)
link_16.Initialize(body_17,body_18,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: robo_leg_link-2/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.740255335981939,0.0395000000000009,-0.770209321266503)
cB = chrono.ChVectorD(0.731443741664156,0.0395000000000009,-0.763327765395133)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-2.77555756156289e-17,-1,-1.66533453693773e-16)
link_17.Initialize(body_17,body_21,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.740255335981939,0.0395000000000009,-0.770209321266503)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.731443741664156,0.0395000000000009,-0.763327765395133)
dB = chrono.ChVectorD(-2.77555756156289e-17,-1,-1.66533453693773e-16)
link_18.Initialize(body_17,body_21,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_18 , SW name: robo_leg_link-2/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: robo_leg_link-2/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.64766814786881,0.0495000000000009,-0.746399767557243)
cB = chrono.ChVectorD(0.657556344822651,0.0495000000000009,-0.747746940004737)
dA = chrono.ChVectorD(1.38777878078145e-17,1,1.52655665885959e-16)
dB = chrono.ChVectorD(9.0205620750794e-17,1,4.16333634234434e-17)
link_19.Initialize(body_18,body_19,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.64766814786881,0.0495000000000009,-0.746399767557243)
dA = chrono.ChVectorD(1.38777878078145e-17,1,1.52655665885959e-16)
cB = chrono.ChVectorD(0.657556344822651,0.0495000000000009,-0.747746940004737)
dB = chrono.ChVectorD(9.0205620750794e-17,1,4.16333634234434e-17)
link_20.Initialize(body_18,body_19,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_21 , SW name: robo_leg_link-2/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-2/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.762313592439771,0.0495000000000009,-0.769565441416997)
cB = chrono.ChVectorD(0.752493757174728,0.0495000000000014,-0.767678167614214)
dA = chrono.ChVectorD(2.77555756156289e-17,1,1.66533453693773e-16)
dB = chrono.ChVectorD(2.06362704702201e-14,1,2.08028039239139e-14)
link_21.Initialize(body_21,body_20,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.762313592439771,0.0495000000000009,-0.769565441416997)
dA = chrono.ChVectorD(2.77555756156289e-17,1,1.66533453693773e-16)
cB = chrono.ChVectorD(0.752493757174728,0.0495000000000014,-0.767678167614214)
dB = chrono.ChVectorD(2.06362704702201e-14,1,2.08028039239139e-14)
link_22.Initialize(body_21,body_20,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_21 , SW name: robo_leg_link-2/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: robo_leg_link-2/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.75741264371572,0.0495000000000009,-0.768575137623091)
dA = chrono.ChVectorD(-2.77555756156289e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.757412643715722,0.0495000000000013,-0.76857513762309)
dB = chrono.ChVectorD(-2.06362704702201e-14,-1,-2.08028039239139e-14)
link_23.Initialize(body_21,body_20,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.75741264371572,0.0495000000000009,-0.768575137623091)
cB = chrono.ChVectorD(0.757412643715722,0.0495000000000013,-0.76857513762309)
dA = chrono.ChVectorD(-2.77555756156289e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-2.06362704702201e-14,-1,-2.08028039239139e-14)
link_24.Initialize(body_21,body_20,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_18 , SW name: robo_leg_link-2/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: robo_leg_link-2/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.65256909661482,0.0495000000000009,-0.747390071242474)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.52655665885959e-16)
cB = chrono.ChVectorD(0.65256909661482,0.0495000000000009,-0.747390071242474)
dB = chrono.ChVectorD(-9.0205620750794e-17,-1,-4.16333634234434e-17)
link_25.Initialize(body_18,body_19,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.65256909661482,0.0495000000000009,-0.747390071242474)
cB = chrono.ChVectorD(0.65256909661482,0.0495000000000009,-0.747390071242474)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.52655665885959e-16)
dB = chrono.ChVectorD(-9.0205620750794e-17,-1,-4.16333634234434e-17)
link_26.Initialize(body_18,body_19,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: robo_leg_link-2/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.736344690388207,0.0545000000000009,-0.764318069189039)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.736344690388207,0.0495000000000009,-0.764318069189039)
dB = chrono.ChVectorD(-2.77555756156289e-17,-1,-1.66533453693773e-16)
link_27.Initialize(body_17,body_21,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.736344690388207,0.0545000000000009,-0.764318069189039)
cB = chrono.ChVectorD(0.736344690388207,0.0495000000000009,-0.764318069189039)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-2.77555756156289e-17,-1,-1.66533453693773e-16)
link_28.Initialize(body_17,body_21,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_17 , SW name: robo_leg_link-2/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: robo_leg_link-2/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.673637050036731,0.0545000000000009,-0.75164713920936)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.673637050036731,0.0495000000000009,-0.751647139209361)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.52655665885959e-16)
link_29.Initialize(body_17,body_18,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.673637050036731,0.0545000000000009,-0.75164713920936)
cB = chrono.ChVectorD(0.673637050036731,0.0495000000000009,-0.751647139209361)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.52655665885959e-16)
link_30.Initialize(body_17,body_18,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_26 , SW name: robo_leg_link-3/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.549762080751647,0.0170000000000726,-0.73506252896247)
dA = chrono.ChVectorD(0.982699916299036,-6.02295990859147e-15,-0.185204952703397)
cB = chrono.ChVectorD(0.557623680082039,0.0170000000000102,-0.736544168584101)
dB = chrono.ChVectorD(-0.982699916299036,5.9535709695524e-15,0.185204952703396)
link_1.SetFlipped(True)
link_1.Initialize(body_26,body_28,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.549762080751647,0.0170000000000726,-0.73506252896247)
cB = chrono.ChVectorD(0.557623680082039,0.0170000000000102,-0.736544168584101)
dA = chrono.ChVectorD(0.982699916299036,-6.02295990859147e-15,-0.185204952703397)
dB = chrono.ChVectorD(-0.982699916299036,5.9535709695524e-15,0.185204952703396)
link_2.Initialize(body_26,body_28,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.579931603794981,0.0110000000000101,-0.767801460303276)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dA = chrono.ChVectorD(1.6757428777936e-15,1,-3.63598040564739e-15)
dB = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
link_3.Initialize(body_28,body_22,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.579931603794981,0.0110000000000101,-0.767801460303276)
dA = chrono.ChVectorD(1.6757428777936e-15,1,-3.63598040564739e-15)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dB = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_26 , SW name: robo_leg_link-3/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.557623680082039,0.0170000000000725,-0.736544168584097)
cB = chrono.ChVectorD(0.55725308497168,0.0149990000000102,-0.738510551116615)
dA = chrono.ChVectorD(0.982699916299036,-6.02295990859147e-15,-0.185204952703397)
dB = chrono.ChVectorD(-0.982699916299036,5.9535709695524e-15,0.185204952703396)
link_5.Initialize(body_26,body_28,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.557623680082039,0.0170000000000725,-0.736544168584097)
dA = chrono.ChVectorD(0.982699916299036,-6.02295990859147e-15,-0.185204952703397)
cB = chrono.ChVectorD(0.55725308497168,0.0149990000000102,-0.738510551116615)
dB = chrono.ChVectorD(-0.982699916299036,5.9535709695524e-15,0.185204952703396)
link_6.SetFlipped(True)
link_6.Initialize(body_26,body_28,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.568063247079137,0.06331000000003,-0.759755180545771)
dA = chrono.ChVectorD(-0.185204952703407,-3.62210261783957e-15,-0.982699916299034)
cB = chrono.ChVectorD(0.554604980916214,0.0610000000000104,-0.705871862745059)
dB = chrono.ChVectorD(0.185204952703407,1.15046860926782e-14,0.982699916299034)
link_7.SetFlipped(True)
link_7.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.579931603794981,0.0110000000000101,-0.767801460303276)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dA = chrono.ChVectorD(1.6757428777936e-15,1,-3.63598040564739e-15)
dB = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
link_8.Initialize(body_28,body_22,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.579931603794981,0.0110000000000101,-0.767801460303276)
dA = chrono.ChVectorD(1.6757428777936e-15,1,-3.63598040564739e-15)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dB = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
link_9.SetFlipped(True)
link_9.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.585092548047598,0.0789500000000414,-0.765681207770687)
dA = chrono.ChVectorD(2.60381993744119e-14,-1,6.77236045021345e-15)
cB = chrono.ChVectorD(0.557135210260595,0.0780000000000103,-0.734416899622841)
dB = chrono.ChVectorD(-2.60173826927002e-14,1,-6.78623823802127e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.557135210260595,0.0789500000000408,-0.734416899622841)
dA = chrono.ChVectorD(2.60381993744119e-14,-1,6.77236045021345e-15)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dB = chrono.ChVectorD(-2.60173826927002e-14,1,-6.78623823802127e-15)
link_11.SetFlipped(True)
link_11.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.557135210260595,0.0789500000000408,-0.734416899622841)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dA = chrono.ChVectorD(2.60381993744119e-14,-1,6.77236045021345e-15)
dB = chrono.ChVectorD(-2.60173826927002e-14,1,-6.78623823802127e-15)
link_12.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_28 , SW name: robo_leg_link-3/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.557135210260598,0.0110000000000103,-0.734416899622843)
dA = chrono.ChVectorD(1.6757428777936e-15,1,-3.63598040564739e-15)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dB = chrono.ChVectorD(-2.60173826927002e-14,1,-6.78623823802127e-15)
link_13.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.557135210260598,0.0110000000000103,-0.734416899622843)
cB = chrono.ChVectorD(0.557135210260596,0.0110000000000103,-0.73441689962284)
dA = chrono.ChVectorD(1.6757428777936e-15,1,-3.63598040564739e-15)
dB = chrono.ChVectorD(-2.60173826927002e-14,1,-6.78623823802127e-15)
link_14.Initialize(body_28,body_22,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: robo_leg_link-3/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.519861572342968,0.0395000000000093,-0.732480131016218)
cB = chrono.ChVectorD(0.530614596470191,0.0395000000000009,-0.729418679896756)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.52655665885959e-16)
link_15.Initialize(body_22,body_23,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.519861572342968,0.0395000000000093,-0.732480131016218)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
cB = chrono.ChVectorD(0.530614596470191,0.0395000000000009,-0.729418679896756)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.52655665885959e-16)
link_16.Initialize(body_22,body_23,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-3/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.592556798651189,0.0395000000000112,-0.746180667392453)
cB = chrono.ChVectorD(0.583655824254315,0.0395000000000009,-0.73941511827013)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
dB = chrono.ChVectorD(-4.5102810375397e-17,-1,-1.2490009027033e-16)
link_17.Initialize(body_22,body_24,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.592556798651189,0.0395000000000112,-0.746180667392453)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
cB = chrono.ChVectorD(0.583655824254315,0.0395000000000009,-0.73941511827013)
dB = chrono.ChVectorD(-4.5102810375397e-17,-1,-1.2490009027033e-16)
link_18.Initialize(body_22,body_24,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-3/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: robo_leg_link-3/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.499665689716988,0.0495000000000009,-0.72358588812574)
cB = chrono.ChVectorD(0.509573171605712,0.0495000000000009,-0.724757151951134)
dA = chrono.ChVectorD(3.46944695195361e-17,1,1.52655665885959e-16)
dB = chrono.ChVectorD(2.08166817117217e-17,1,2.77555756156289e-16)
link_19.Initialize(body_23,body_25,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.499665689716988,0.0495000000000009,-0.72358588812574)
dA = chrono.ChVectorD(3.46944695195361e-17,1,1.52655665885959e-16)
cB = chrono.ChVectorD(0.509573171605712,0.0495000000000009,-0.724757151951134)
dB = chrono.ChVectorD(2.08166817117217e-17,1,2.77555756156289e-16)
link_20.Initialize(body_23,body_25,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-3/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-3/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.614604729726882,0.0495000000000009,-0.745247916836222)
cB = chrono.ChVectorD(0.604703981940157,0.0495000000000039,-0.743965023296645)
dA = chrono.ChVectorD(4.5102810375397e-17,1,1.2490009027033e-16)
dB = chrono.ChVectorD(2.07021899623072e-14,1,2.07334149848748e-14)
link_21.Initialize(body_24,body_27,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.614604729726882,0.0495000000000009,-0.745247916836222)
dA = chrono.ChVectorD(4.5102810375397e-17,1,1.2490009027033e-16)
cB = chrono.ChVectorD(0.604703981940157,0.0495000000000039,-0.743965023296645)
dB = chrono.ChVectorD(2.07021899623072e-14,1,2.07334149848748e-14)
link_22.Initialize(body_24,body_27,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_24 , SW name: robo_leg_link-3/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_27 , SW name: robo_leg_link-3/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.609691230147987,0.0495000000000009,-0.744321892058911)
dA = chrono.ChVectorD(-4.5102810375397e-17,-1,-1.2490009027033e-16)
cB = chrono.ChVectorD(0.609691230147988,0.0495000000000038,-0.744321892058909)
dB = chrono.ChVectorD(-2.07021899623072e-14,-1,-2.07334149848748e-14)
link_23.Initialize(body_24,body_27,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.609691230147987,0.0495000000000009,-0.744321892058911)
cB = chrono.ChVectorD(0.609691230147988,0.0495000000000038,-0.744321892058909)
dA = chrono.ChVectorD(-4.5102810375397e-17,-1,-1.2490009027033e-16)
dB = chrono.ChVectorD(-2.07021899623072e-14,-1,-2.07334149848748e-14)
link_24.Initialize(body_24,body_27,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_23 , SW name: robo_leg_link-3/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: robo_leg_link-3/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.504579189499199,0.0495000000000009,-0.724511911824254)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.52655665885959e-16)
cB = chrono.ChVectorD(0.504579189499199,0.0495000000000009,-0.724511911824254)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.77555756156289e-16)
link_25.Initialize(body_23,body_25,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.504579189499199,0.0495000000000009,-0.724511911824254)
cB = chrono.ChVectorD(0.504579189499199,0.0495000000000009,-0.724511911824254)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.52655665885959e-16)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.77555756156289e-16)
link_26.Initialize(body_23,body_25,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: robo_leg_link-3/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.58856932383321,0.0545000000000111,-0.740341143047441)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
cB = chrono.ChVectorD(0.58856932383321,0.0495000000000009,-0.740341143047442)
dB = chrono.ChVectorD(-4.5102810375397e-17,-1,-1.2490009027033e-16)
link_27.Initialize(body_22,body_24,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.58856932383321,0.0545000000000111,-0.740341143047441)
cB = chrono.ChVectorD(0.58856932383321,0.0495000000000009,-0.740341143047442)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
dB = chrono.ChVectorD(-4.5102810375397e-17,-1,-1.2490009027033e-16)
link_28.Initialize(body_22,body_24,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_22 , SW name: robo_leg_link-3/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: robo_leg_link-3/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.52570109668798,0.0545000000000095,-0.72849265619824)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
cB = chrono.ChVectorD(0.52570109668798,0.0495000000000009,-0.728492656198242)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.52655665885959e-16)
link_29.Initialize(body_22,body_23,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.52570109668798,0.0545000000000095,-0.72849265619824)
cB = chrono.ChVectorD(0.52570109668798,0.0495000000000009,-0.728492656198242)
dA = chrono.ChVectorD(2.60173826927002e-14,-1,6.78623823802127e-15)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.52655665885959e-16)
link_30.Initialize(body_22,body_23,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_31 , SW name: robo_leg_link-7/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.476983588479524,0.0170000000000234,-0.397411217652927)
dA = chrono.ChVectorD(-0.911597392658731,-5.12714870559705e-14,0.411084168629495)
cB = chrono.ChVectorD(0.469690809338254,0.0169999999999962,-0.394122544303895)
dB = chrono.ChVectorD(0.911597392658731,2.47857290247566e-14,-0.411084168629496)
link_1.SetFlipped(True)
link_1.Initialize(body_31,body_30,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.476983588479524,0.0170000000000234,-0.397411217652927)
cB = chrono.ChVectorD(0.469690809338254,0.0169999999999962,-0.394122544303895)
dA = chrono.ChVectorD(-0.911597392658731,-5.12714870559705e-14,0.411084168629495)
dB = chrono.ChVectorD(0.911597392658731,2.47857290247566e-14,-0.411084168629496)
link_2.Initialize(body_31,body_30,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.455358206613304,0.0109999999999961,-0.358496174223169)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dA = chrono.ChVectorD(-2.55880386323959e-14,1,-5.41233724504764e-15)
dB = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
link_3.Initialize(body_30,body_32,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.455358206613304,0.0109999999999961,-0.358496174223169)
dA = chrono.ChVectorD(-2.55880386323959e-14,1,-5.41233724504764e-15)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dB = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_31 , SW name: robo_leg_link-7/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.469690809338254,0.017000000000023,-0.394122544303891)
cB = chrono.ChVectorD(0.470513388759682,0.0149989999999963,-0.392298437921184)
dA = chrono.ChVectorD(-0.911597392658731,-5.12714870559705e-14,0.411084168629495)
dB = chrono.ChVectorD(0.911597392658731,2.47857290247566e-14,-0.411084168629496)
link_5.Initialize(body_31,body_30,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.469690809338254,0.017000000000023,-0.394122544303891)
dA = chrono.ChVectorD(-0.911597392658731,-5.12714870559705e-14,0.411084168629495)
cB = chrono.ChVectorD(0.470513388759682,0.0149989999999963,-0.392298437921184)
dB = chrono.ChVectorD(0.911597392658731,2.47857290247566e-14,-0.411084168629496)
link_6.SetFlipped(True)
link_6.Initialize(body_31,body_30,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.465001790042032,0.0633100000000162,-0.369107574117798)
dA = chrono.ChVectorD(0.411084168629506,1.52100554373646e-14,0.911597392658726)
cB = chrono.ChVectorD(0.465412581928121,0.06100000000001,-0.424644661864193)
dB = chrono.ChVectorD(-0.411084168629505,-1.68753899743024e-14,-0.911597392658727)
link_7.SetFlipped(True)
link_7.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.455358206613304,0.0109999999999961,-0.358496174223169)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dA = chrono.ChVectorD(-2.55880386323959e-14,1,-5.41233724504764e-15)
dB = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
link_8.Initialize(body_30,body_32,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.455358206613304,0.0109999999999961,-0.358496174223169)
dA = chrono.ChVectorD(-2.55880386323959e-14,1,-5.41233724504764e-15)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dB = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
link_9.SetFlipped(True)
link_9.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.449843411231993,0.0789500000000272,-0.359343433500402)
dA = chrono.ChVectorD(-2.08773970333809e-15,-1,8.86790640919344e-15)
cB = chrono.ChVectorD(0.46966537712218,0.0780000000000103,-0.396305026652595)
dB = chrono.ChVectorD(-2.603039311877e-14,1,-6.74460487459783e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.469665377122183,0.0789500000000268,-0.396305026652596)
dA = chrono.ChVectorD(-2.08773970333809e-15,-1,8.86790640919344e-15)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dB = chrono.ChVectorD(-2.603039311877e-14,1,-6.74460487459783e-15)
link_11.SetFlipped(True)
link_11.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.469665377122183,0.0789500000000268,-0.396305026652596)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dA = chrono.ChVectorD(-2.08773970333809e-15,-1,8.86790640919344e-15)
dB = chrono.ChVectorD(-2.603039311877e-14,1,-6.74460487459783e-15)
link_12.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_30 , SW name: robo_leg_link-7/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.469665377122182,0.0109999999999963,-0.396305026652593)
dA = chrono.ChVectorD(-2.55880386323959e-14,1,-5.41233724504764e-15)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dB = chrono.ChVectorD(-2.603039311877e-14,1,-6.74460487459783e-15)
link_13.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.469665377122182,0.0109999999999963,-0.396305026652593)
cB = chrono.ChVectorD(0.469665377122182,0.0110000000000104,-0.396305026652594)
dA = chrono.ChVectorD(-2.55880386323959e-14,1,-5.41233724504764e-15)
dB = chrono.ChVectorD(-2.603039311877e-14,1,-6.74460487459783e-15)
link_14.Initialize(body_30,body_32,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: robo_leg_link-7/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.505438506526293,0.0395000000000112,-0.406952015376485)
cB = chrono.ChVectorD(0.494267109009534,0.0395000000000009,-0.407399166745146)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
dB = chrono.ChVectorD(-6.24500451351651e-17,-1,-1.94289029309402e-16)
link_15.Initialize(body_32,body_33,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.505438506526293,0.0395000000000112,-0.406952015376485)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
cB = chrono.ChVectorD(0.494267109009534,0.0395000000000009,-0.407399166745146)
dB = chrono.ChVectorD(-6.24500451351651e-17,-1,-1.94289029309402e-16)
link_16.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_34 , SW name: robo_leg_link-7/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.438003089404364,0.0395000000000097,-0.376542064002117)
cB = chrono.ChVectorD(0.445063644248548,0.0395000000000098,-0.385210888747307)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
dB = chrono.ChVectorD(2.60026375431543e-14,-1,6.68909372336657e-15)
link_17.Initialize(body_32,body_34,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.438003089404364,0.0395000000000097,-0.376542064002117)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
cB = chrono.ChVectorD(0.445063644248548,0.0395000000000098,-0.385210888747307)
dB = chrono.ChVectorD(2.60026375431543e-14,-1,6.68909372336657e-15)
link_18.Initialize(body_32,body_34,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_33 , SW name: robo_leg_link-7/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_29 , SW name: robo_leg_link-7/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.522976746697408,0.0495000000000009,-0.420345710411064)
cB = chrono.ChVectorD(0.513521588191961,0.0494999999999989,-0.417281465747248)
dA = chrono.ChVectorD(6.24500451351651e-17,1,1.94289029309402e-16)
dB = chrono.ChVectorD(2.07368844318268e-14,1,2.13301598606108e-14)
link_19.Initialize(body_33,body_29,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.522976746697408,0.0495000000000009,-0.420345710411064)
dA = chrono.ChVectorD(6.24500451351651e-17,1,1.94289029309402e-16)
cB = chrono.ChVectorD(0.513521588191961,0.0494999999999989,-0.417281465747248)
dB = chrono.ChVectorD(2.07368844318268e-14,1,2.13301598606108e-14)
link_20.Initialize(body_33,body_29,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_34 , SW name: robo_leg_link-7/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.416354012773809,0.0495000000000091,-0.372264331303447)
cB = chrono.ChVectorD(0.42564797890714,0.0495000000000003,-0.375923031740907)
dA = chrono.ChVectorD(-2.60026375431543e-14,1,-6.68909372336657e-15)
dB = chrono.ChVectorD(2.1318884158017e-14,1,2.30510055487798e-14)
link_21.Initialize(body_34,body_35,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.416354012773809,0.0495000000000091,-0.372264331303447)
dA = chrono.ChVectorD(-2.60026375431543e-14,1,-6.68909372336657e-15)
cB = chrono.ChVectorD(0.42564797890714,0.0495000000000003,-0.375923031740907)
dB = chrono.ChVectorD(2.1318884158017e-14,1,2.30510055487798e-14)
link_22.Initialize(body_34,body_35,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_34 , SW name: robo_leg_link-7/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_35 , SW name: robo_leg_link-7/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.420912001497721,0.0495000000000093,-0.374319748242341)
dA = chrono.ChVectorD(2.60026375431543e-14,-1,6.68909372336657e-15)
cB = chrono.ChVectorD(0.4209120014976,0.0495000000000004,-0.374319748242479)
dB = chrono.ChVectorD(-2.1318884158017e-14,-1,-2.30510055487798e-14)
link_23.Initialize(body_34,body_35,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.420912001497721,0.0495000000000093,-0.374319748242341)
cB = chrono.ChVectorD(0.4209120014976,0.0495000000000004,-0.374319748242479)
dA = chrono.ChVectorD(2.60026375431543e-14,-1,6.68909372336657e-15)
dB = chrono.ChVectorD(-2.1318884158017e-14,-1,-2.30510055487798e-14)
link_24.Initialize(body_34,body_35,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_33 , SW name: robo_leg_link-7/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_29 , SW name: robo_leg_link-7/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.518418756987088,0.0495000000000009,-0.418290295659578)
dA = chrono.ChVectorD(-6.24500451351651e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.51841875698709,0.0494999999999989,-0.418290295659577)
dB = chrono.ChVectorD(-2.07368844318268e-14,-1,-2.13301598606108e-14)
link_25.Initialize(body_33,body_29,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.518418756987088,0.0495000000000009,-0.418290295659578)
cB = chrono.ChVectorD(0.51841875698709,0.0494999999999989,-0.418290295659577)
dA = chrono.ChVectorD(-6.24500451351651e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-2.07368844318268e-14,-1,-2.13301598606108e-14)
link_26.Initialize(body_33,body_29,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_34 , SW name: robo_leg_link-7/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.44050565552451,0.0545000000000097,-0.383155471808558)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
cB = chrono.ChVectorD(0.440505655524636,0.0495000000000097,-0.383155471808413)
dB = chrono.ChVectorD(2.60026375431543e-14,-1,6.68909372336657e-15)
link_27.Initialize(body_32,body_34,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.44050565552451,0.0545000000000097,-0.383155471808558)
cB = chrono.ChVectorD(0.440505655524636,0.0495000000000097,-0.383155471808413)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
dB = chrono.ChVectorD(2.60026375431543e-14,-1,6.68909372336657e-15)
link_28.Initialize(body_32,body_34,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_32 , SW name: robo_leg_link-7/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_33 , SW name: robo_leg_link-7/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.498825098719852,0.054500000000011,-0.409454581496631)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
cB = chrono.ChVectorD(0.498825098719853,0.0495000000000009,-0.409454581496631)
dB = chrono.ChVectorD(-6.24500451351651e-17,-1,-1.94289029309402e-16)
link_29.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.498825098719852,0.054500000000011,-0.409454581496631)
cB = chrono.ChVectorD(0.498825098719853,0.0495000000000009,-0.409454581496631)
dA = chrono.ChVectorD(2.603039311877e-14,-1,6.74460487459783e-15)
dB = chrono.ChVectorD(-6.24500451351651e-17,-1,-1.94289029309402e-16)
link_30.Initialize(body_32,body_33,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-11/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.990790894097152,0.017000000000001,-0.615237357615954)
dA = chrono.ChVectorD(-0.631301035763153,-3.05103164954801e-14,0.775537879309819)
cB = chrono.ChVectorD(0.985740485811047,0.0170000000000008,-0.609033054581475)
dB = chrono.ChVectorD(0.631301035763153,3.05241942832879e-14,-0.775537879309819)
link_1.SetFlipped(True)
link_1.Initialize(body_36,body_37,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.990790894097152,0.017000000000001,-0.615237357615954)
cB = chrono.ChVectorD(0.985740485811047,0.0170000000000008,-0.609033054581475)
dA = chrono.ChVectorD(-0.631301035763153,-3.05103164954801e-14,0.775537879309819)
dB = chrono.ChVectorD(0.631301035763153,3.05241942832879e-14,-0.775537879309819)
link_2.Initialize(body_36,body_37,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.988864177330467,0.0110000000000004,-0.570758986372157)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dA = chrono.ChVectorD(-2.31550889573384e-14,1,1.57235335862538e-14)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
link_3.Initialize(body_37,body_40,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.988864177330467,0.0110000000000004,-0.570758986372157)
dA = chrono.ChVectorD(-2.31550889573384e-14,1,1.57235335862538e-14)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_36 , SW name: robo_leg_link-11/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.985740485811047,0.0170000000000008,-0.609033054581475)
cB = chrono.ChVectorD(0.987292337107546,0.0149990000000008,-0.607769821208913)
dA = chrono.ChVectorD(-0.631301035763153,-3.05103164954801e-14,0.775537879309819)
dB = chrono.ChVectorD(0.631301035763153,3.05241942832879e-14,-0.775537879309819)
link_5.Initialize(body_36,body_37,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.985740485811047,0.0170000000000008,-0.609033054581475)
dA = chrono.ChVectorD(-0.631301035763153,-3.05103164954801e-14,0.775537879309819)
cB = chrono.ChVectorD(0.987292337107546,0.0149990000000008,-0.607769821208913)
dB = chrono.ChVectorD(0.631301035763153,3.05241942832879e-14,-0.775537879309819)
link_6.SetFlipped(True)
link_6.Initialize(body_36,body_37,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.992740288146666,0.0633100000000205,-0.584563922953362)
dA = chrono.ChVectorD(0.775537879309826,7.67441665772139e-15,0.631301035763144)
cB = chrono.ChVectorD(0.968257010737885,0.0610000000000009,-0.634414760385136)
dB = chrono.ChVectorD(-0.775537879309826,1.2490009027033e-16,-0.631301035763144)
link_7.SetFlipped(True)
link_7.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.988864177330467,0.0110000000000004,-0.570758986372157)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dA = chrono.ChVectorD(-2.31550889573384e-14,1,1.57235335862538e-14)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
link_8.Initialize(body_37,body_40,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.988864177330467,0.0110000000000004,-0.570758986372157)
dA = chrono.ChVectorD(-2.31550889573384e-14,1,1.57235335862538e-14)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.983553160276785,0.0789500000000315,-0.569049040689956)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.35922392732846e-16)
cB = chrono.ChVectorD(0.984741167157688,0.0780000000000008,-0.610973476759521)
dB = chrono.ChVectorD(3.46944695195361e-17,1,1.80411241501588e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.984741167157688,0.0789500000000314,-0.610973476759521)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.35922392732846e-16)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dB = chrono.ChVectorD(3.46944695195361e-17,1,1.80411241501588e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.984741167157688,0.0789500000000314,-0.610973476759521)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.35922392732846e-16)
dB = chrono.ChVectorD(3.46944695195361e-17,1,1.80411241501588e-16)
link_12.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_37 , SW name: robo_leg_link-11/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759518)
dA = chrono.ChVectorD(-2.31550889573384e-14,1,1.57235335862538e-14)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dB = chrono.ChVectorD(3.46944695195361e-17,1,1.80411241501588e-16)
link_13.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759518)
cB = chrono.ChVectorD(0.984741167157688,0.0110000000000009,-0.610973476759521)
dA = chrono.ChVectorD(-2.31550889573384e-14,1,1.57235335862538e-14)
dB = chrono.ChVectorD(3.46944695195361e-17,1,1.80411241501588e-16)
link_14.Initialize(body_37,body_40,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_38 , SW name: robo_leg_link-11/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.01196910361453,0.0395000000000009,-0.636502178891677)
cB = chrono.ChVectorD(1.00177839994237,0.0395000000000009,-0.631903308466698)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.66533453693773e-16)
link_15.Initialize(body_40,body_38,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.01196910361453,0.0395000000000009,-0.636502178891677)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(1.00177839994237,0.0395000000000009,-0.631903308466698)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.66533453693773e-16)
link_16.Initialize(body_40,body_38,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_42 , SW name: robo_leg_link-11/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.965268609493947,0.0395000000000009,-0.579131764269733)
cB = chrono.ChVectorD(0.967703930446683,0.0395000000000009,-0.590043648248441)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.08166817117217e-16)
link_17.Initialize(body_40,body_42,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.965268609493947,0.0395000000000009,-0.579131764269733)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.967703930446683,0.0395000000000009,-0.590043648248441)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.08166817117217e-16)
link_18.Initialize(body_40,body_42,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_38 , SW name: robo_leg_link-11/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_41 , SW name: robo_leg_link-11/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.02166046161579,0.0495000000000009,-0.656327884464589)
cB = chrono.ChVectorD(1.01913592267253,0.0495000000000009,-0.647490297681217)
dA = chrono.ChVectorD(3.46944695195361e-17,1,1.66533453693773e-16)
dB = chrono.ChVectorD(3.95516952522712e-16,1,4.57966997657877e-16)
link_19.Initialize(body_38,body_41,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.02166046161579,0.0495000000000009,-0.656327884464589)
dA = chrono.ChVectorD(3.46944695195361e-17,1,1.66533453693773e-16)
cB = chrono.ChVectorD(1.01913592267253,0.0495000000000009,-0.647490297681217)
dB = chrono.ChVectorD(3.95516952522712e-16,1,4.57966997657877e-16)
link_20.Initialize(body_38,body_41,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_42 , SW name: robo_leg_link-11/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-11/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.947821893504194,0.0495000000000009,-0.565619052119129)
cB = chrono.ChVectorD(0.95059769781172,0.0495000000000216,-0.574482227141042)
dA = chrono.ChVectorD(3.46944695195361e-17,1,2.08166817117217e-16)
dB = chrono.ChVectorD(-2.61110577604029e-14,1,-6.59194920871187e-15)
link_21.Initialize(body_42,body_39,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.947821893504194,0.0495000000000009,-0.565619052119129)
dA = chrono.ChVectorD(3.46944695195361e-17,1,2.08166817117217e-16)
cB = chrono.ChVectorD(0.95059769781172,0.0495000000000216,-0.574482227141042)
dB = chrono.ChVectorD(-2.61110577604029e-14,1,-6.59194920871187e-15)
link_22.Initialize(body_42,body_39,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_42 , SW name: robo_leg_link-11/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_39 , SW name: robo_leg_link-11/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.950978398674662,0.0495000000000009,-0.569496741522473)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.08166817117217e-16)
cB = chrono.ChVectorD(0.95097839867466,0.0495000000000217,-0.569496741522474)
dB = chrono.ChVectorD(2.61110577604029e-14,-1,6.59194920871187e-15)
link_23.Initialize(body_42,body_39,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.950978398674662,0.0495000000000009,-0.569496741522473)
cB = chrono.ChVectorD(0.95097839867466,0.0495000000000217,-0.569496741522474)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.08166817117217e-16)
dB = chrono.ChVectorD(2.61110577604029e-14,-1,6.59194920871187e-15)
link_24.Initialize(body_42,body_39,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_38 , SW name: robo_leg_link-11/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_41 , SW name: robo_leg_link-11/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.018503952519,0.0495000000000009,-0.652450198257343)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(1.018503952519,0.0495000000000009,-0.652450198257343)
dB = chrono.ChVectorD(-3.95516952522712e-16,-1,-4.57966997657877e-16)
link_25.Initialize(body_38,body_41,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.018503952519,0.0495000000000009,-0.652450198257343)
cB = chrono.ChVectorD(1.018503952519,0.0495000000000009,-0.652450198257343)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-3.95516952522712e-16,-1,-4.57966997657877e-16)
link_26.Initialize(body_38,body_41,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_42 , SW name: robo_leg_link-11/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.964547425276214,0.0545000000000009,-0.586165958845097)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.964547425276214,0.0495000000000009,-0.586165958845097)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.08166817117217e-16)
link_27.Initialize(body_40,body_42,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.964547425276214,0.0545000000000009,-0.586165958845097)
cB = chrono.ChVectorD(0.964547425276214,0.0495000000000009,-0.586165958845097)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-2.08166817117217e-16)
link_28.Initialize(body_40,body_42,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_40 , SW name: robo_leg_link-11/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_38 , SW name: robo_leg_link-11/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.00493490903916,0.0545000000000009,-0.635780994673944)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(1.00493490903916,0.0495000000000009,-0.635780994673944)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.66533453693773e-16)
link_29.Initialize(body_40,body_38,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.00493490903916,0.0545000000000009,-0.635780994673944)
cB = chrono.ChVectorD(1.00493490903916,0.0495000000000009,-0.635780994673944)
dA = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-3.46944695195361e-17,-1,-1.66533453693773e-16)
link_30.Initialize(body_40,body_38,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-1/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.846092588225907,0.0170000000000011,-0.777177914981794)
dA = chrono.ChVectorD(0.999714119185824,-3.04964387076723e-14,0.0239098285337404)
cB = chrono.ChVectorD(0.854090301179394,0.0170000000000008,-0.776986636353524)
dB = chrono.ChVectorD(-0.999714119185824,3.04548053442488e-14,-0.0239098285337404)
link_1.SetFlipped(True)
link_1.Initialize(body_46,body_44,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.846092588225907,0.0170000000000011,-0.777177914981794)
cB = chrono.ChVectorD(0.854090301179394,0.0170000000000008,-0.776986636353524)
dA = chrono.ChVectorD(0.999714119185824,-3.04964387076723e-14,0.0239098285337404)
dB = chrono.ChVectorD(-0.999714119185824,3.04548053442488e-14,-0.0239098285337404)
link_2.Initialize(body_46,body_44,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.882429021927721,0.0110000000000004,-0.802901470953582)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dA = chrono.ChVectorD(2.65620858641569e-14,1,9.00668428727158e-15)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_3.Initialize(body_44,body_47,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.882429021927721,0.0110000000000004,-0.802901470953582)
dA = chrono.ChVectorD(2.65620858641569e-14,1,9.00668428727158e-15)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_46 , SW name: robo_leg_link-1/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.854090301179394,0.0170000000000008,-0.776986636353524)
cB = chrono.ChVectorD(0.85413814474629,0.0149990000000009,-0.778987064306015)
dA = chrono.ChVectorD(0.999714119185824,-3.04964387076723e-14,0.0239098285337404)
dB = chrono.ChVectorD(-0.999714119185824,3.04548053442488e-14,-0.0239098285337404)
link_5.Initialize(body_46,body_44,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.854090301179394,0.0170000000000008,-0.776986636353524)
dA = chrono.ChVectorD(0.999714119185824,-3.04964387076723e-14,0.0239098285337404)
cB = chrono.ChVectorD(0.85413814474629,0.0149990000000009,-0.778987064306015)
dB = chrono.ChVectorD(-0.999714119185824,3.04548053442488e-14,-0.0239098285337404)
link_6.SetFlipped(True)
link_6.Initialize(body_46,body_44,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.869143036963536,0.0633100000000205,-0.797508594856173)
dA = chrono.ChVectorD(0.0239098285337298,8.00748356510894e-15,-0.999714119185824)
cB = chrono.ChVectorD(0.84473832010218,0.0610000000000009,-0.747619250700917)
dB = chrono.ChVectorD(-0.0239098285337298,-1.38777878078145e-16,0.999714119185824)
link_7.SetFlipped(True)
link_7.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.882429021927721,0.0110000000000004,-0.802901470953582)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dA = chrono.ChVectorD(2.65620858641569e-14,1,9.00668428727158e-15)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_8.Initialize(body_44,body_47,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.882429021927721,0.0110000000000004,-0.802901470953582)
dA = chrono.ChVectorD(2.65620858641569e-14,1,9.00668428727158e-15)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.887033990874087,0.0789500000000315,-0.799751061871639)
dA = chrono.ChVectorD(-9.71445146547012e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.853168731382154,0.078000000000001,-0.775008105305291)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.853168731382154,0.0789500000000315,-0.775008105305292)
dA = chrono.ChVectorD(-9.71445146547012e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.853168731382154,0.0789500000000315,-0.775008105305292)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dA = chrono.ChVectorD(-9.71445146547012e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
link_12.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_44 , SW name: robo_leg_link-1/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.853168731382155,0.0110000000000009,-0.775008105305294)
dA = chrono.ChVectorD(2.65620858641569e-14,1,9.00668428727158e-15)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
link_13.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.853168731382155,0.0110000000000009,-0.775008105305294)
cB = chrono.ChVectorD(0.853168731382153,0.0110000000000009,-0.775008105305292)
dA = chrono.ChVectorD(2.65620858641569e-14,1,9.00668428727158e-15)
dB = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
link_14.Initialize(body_44,body_47,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-1/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.816311354541436,0.039500000000001,-0.780891040684112)
cB = chrono.ChVectorD(0.826188946565858,0.0395000000000009,-0.775653370767232)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_15.Initialize(body_47,body_48,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.816311354541436,0.039500000000001,-0.780891040684112)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
cB = chrono.ChVectorD(0.826188946565858,0.0395000000000009,-0.775653370767232)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_16.Initialize(body_47,body_48,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-1/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.890265206508208,0.039500000000001,-0.779122311118329)
cB = chrono.ChVectorD(0.88014851641314,0.039500000000001,-0.774362848819536)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
dB = chrono.ChVectorD(-1.04083408558608e-16,-1,-1.11022302462516e-16)
link_17.Initialize(body_47,body_43,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.890265206508208,0.039500000000001,-0.779122311118329)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
cB = chrono.ChVectorD(0.88014851641314,0.039500000000001,-0.774362848819536)
dB = chrono.ChVectorD(-1.04083408558608e-16,-1,-1.11022302462516e-16)
link_18.Initialize(body_47,body_43,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_48 , SW name: robo_leg_link-1/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_49 , SW name: robo_leg_link-1/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.79470420018076,0.0495000000000009,-0.776406387452688)
cB = chrono.ChVectorD(0.804621657292915,0.0495000000000009,-0.777183807283282)
dA = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
dB = chrono.ChVectorD(3.05311331771918e-16,1,0)
link_19.Initialize(body_48,body_49,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.79470420018076,0.0495000000000009,-0.776406387452688)
dA = chrono.ChVectorD(7.63278329429795e-17,1,1.2490009027033e-16)
cB = chrono.ChVectorD(0.804621657292915,0.0495000000000009,-0.777183807283282)
dB = chrono.ChVectorD(3.05311331771918e-16,1,0)
link_20.Initialize(body_48,body_49,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_43 , SW name: robo_leg_link-1/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_45 , SW name: robo_leg_link-1/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.911633261445957,0.0495000000000009,-0.773609775595338)
cB = chrono.ChVectorD(0.901702282119491,0.0494999999999984,-0.772909980276639)
dA = chrono.ChVectorD(1.04083408558608e-16,1,1.11022302462516e-16)
dB = chrono.ChVectorD(2.08374983934334e-14,1,2.06917816214514e-14)
link_21.Initialize(body_43,body_45,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.911633261445957,0.0495000000000009,-0.773609775595338)
dA = chrono.ChVectorD(1.04083408558608e-16,1,1.11022302462516e-16)
cB = chrono.ChVectorD(0.901702282119491,0.0494999999999984,-0.772909980276639)
dB = chrono.ChVectorD(2.08374983934334e-14,1,2.06917816214514e-14)
link_22.Initialize(body_43,body_45,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_43 , SW name: robo_leg_link-1/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_45 , SW name: robo_leg_link-1/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.906634691089487,0.0495000000000009,-0.773729334749804)
dA = chrono.ChVectorD(-1.04083408558608e-16,-1,-1.11022302462516e-16)
cB = chrono.ChVectorD(0.906634691089488,0.0494999999999983,-0.773729334749803)
dB = chrono.ChVectorD(-2.08374983934334e-14,-1,-2.06917816214514e-14)
link_23.Initialize(body_43,body_45,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.906634691089487,0.0495000000000009,-0.773729334749804)
cB = chrono.ChVectorD(0.906634691089488,0.0494999999999983,-0.773729334749803)
dA = chrono.ChVectorD(-1.04083408558608e-16,-1,-1.11022302462516e-16)
dB = chrono.ChVectorD(-2.08374983934334e-14,-1,-2.06917816214514e-14)
link_24.Initialize(body_43,body_45,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_48 , SW name: robo_leg_link-1/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_49 , SW name: robo_leg_link-1/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.79970277075192,0.0495000000000009,-0.776286837274406)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
cB = chrono.ChVectorD(0.799702770751921,0.049500000000001,-0.776286837274406)
dB = chrono.ChVectorD(-3.05311331771918e-16,-1,0)
link_25.Initialize(body_48,body_49,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.79970277075192,0.0495000000000009,-0.776286837274406)
cB = chrono.ChVectorD(0.799702770751921,0.049500000000001,-0.776286837274406)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
dB = chrono.ChVectorD(-3.05311331771918e-16,-1,0)
link_26.Initialize(body_48,body_49,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_43 , SW name: robo_leg_link-1/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.88514708676961,0.054500000000001,-0.774243289665069)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
cB = chrono.ChVectorD(0.88514708676961,0.049500000000001,-0.774243289665069)
dB = chrono.ChVectorD(-1.04083408558608e-16,-1,-1.11022302462516e-16)
link_27.Initialize(body_47,body_43,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.88514708676961,0.054500000000001,-0.774243289665069)
cB = chrono.ChVectorD(0.88514708676961,0.049500000000001,-0.774243289665069)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
dB = chrono.ChVectorD(-1.04083408558608e-16,-1,-1.11022302462516e-16)
link_28.Initialize(body_47,body_43,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_47 , SW name: robo_leg_link-1/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_48 , SW name: robo_leg_link-1/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.821190375994697,0.054500000000001,-0.775772920945514)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
cB = chrono.ChVectorD(0.821190375994697,0.0495000000000009,-0.775772920945514)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_29.Initialize(body_47,body_48,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.821190375994697,0.054500000000001,-0.775772920945514)
cB = chrono.ChVectorD(0.821190375994697,0.0495000000000009,-0.775772920945514)
dA = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
dB = chrono.ChVectorD(-7.63278329429795e-17,-1,-1.2490009027033e-16)
link_30.Initialize(body_47,body_48,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-4/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.414181386570577,0.017000000000001,-0.684659674334456)
dA = chrono.ChVectorD(0.761728146680825,-3.00939828612456e-14,-0.647896774613206)
cB = chrono.ChVectorD(0.420275211744024,0.0170000000000008,-0.689842848531362)
dB = chrono.ChVectorD(-0.761728146680826,3.0218882951516e-14,0.647896774613206)
link_1.SetFlipped(True)
link_1.Initialize(body_56,body_57,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.414181386570577,0.017000000000001,-0.684659674334456)
cB = chrono.ChVectorD(0.420275211744024,0.0170000000000008,-0.689842848531362)
dA = chrono.ChVectorD(0.761728146680825,-3.00939828612456e-14,-0.647896774613206)
dB = chrono.ChVectorD(-0.761728146680826,3.0218882951516e-14,0.647896774613206)
link_2.Initialize(body_56,body_57,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.424159124663602,0.0110000000000004,-0.728047258832178)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dA = chrono.ChVectorD(2.56704379975048e-14,1,-1.07969189144796e-14)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
link_3.Initialize(body_57,body_53,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.424159124663602,0.0110000000000004,-0.728047258832178)
dA = chrono.ChVectorD(2.56704379975048e-14,1,-1.07969189144796e-14)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_56 , SW name: robo_leg_link-4/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.420275211744024,0.0170000000000008,-0.689842848531362)
cB = chrono.ChVectorD(0.418978770298023,0.0149990000000009,-0.69136706655287)
dA = chrono.ChVectorD(0.761728146680825,-3.00939828612456e-14,-0.647896774613206)
dB = chrono.ChVectorD(-0.761728146680826,3.0218882951516e-14,0.647896774613206)
link_5.Initialize(body_56,body_57,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.420275211744024,0.0170000000000008,-0.689842848531362)
dA = chrono.ChVectorD(0.761728146680825,-3.00939828612456e-14,-0.647896774613206)
cB = chrono.ChVectorD(0.418978770298023,0.0149990000000009,-0.69136706655287)
dB = chrono.ChVectorD(-0.761728146680826,3.0218882951516e-14,0.647896774613206)
link_6.SetFlipped(True)
link_6.Initialize(body_56,body_57,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.417838771164856,0.0633100000000205,-0.715176608470934)
dA = chrono.ChVectorD(-0.647896774613214,8.11850586757146e-15,-0.761728146680818)
cB = chrono.ChVectorD(0.432854910519983,0.0610000000000009,-0.661706500707886)
dB = chrono.ChVectorD(0.647896774613214,-1.52655665885959e-16,0.761728146680818)
link_7.SetFlipped(True)
link_7.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.424159124663602,0.0110000000000004,-0.728047258832178)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dA = chrono.ChVectorD(2.56704379975048e-14,1,-1.07969189144796e-14)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
link_8.Initialize(body_57,body_53,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.424159124663602,0.0110000000000004,-0.728047258832178)
dA = chrono.ChVectorD(2.56704379975048e-14,1,-1.07969189144796e-14)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.429692454389811,0.0789500000000315,-0.728763553828419)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-2.4980018054066e-16)
cB = chrono.ChVectorD(0.420905254893047,0.0780000000000009,-0.687753130456986)
dB = chrono.ChVectorD(3.81639164714898e-17,1,1.80411241501588e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.420905254893047,0.0789500000000315,-0.687753130456986)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-2.4980018054066e-16)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dB = chrono.ChVectorD(3.81639164714898e-17,1,1.80411241501588e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.420905254893047,0.0789500000000315,-0.687753130456986)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-2.4980018054066e-16)
dB = chrono.ChVectorD(3.81639164714898e-17,1,1.80411241501588e-16)
link_12.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_57 , SW name: robo_leg_link-4/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456989)
dA = chrono.ChVectorD(2.56704379975048e-14,1,-1.07969189144796e-14)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dB = chrono.ChVectorD(3.81639164714898e-17,1,1.80411241501588e-16)
link_13.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456989)
cB = chrono.ChVectorD(0.420905254893047,0.0110000000000009,-0.687753130456986)
dA = chrono.ChVectorD(2.56704379975048e-14,1,-1.07969189144796e-14)
dB = chrono.ChVectorD(3.81639164714898e-17,1,1.80411241501588e-16)
link_14.Initialize(body_57,body_53,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-4/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.389491351194624,0.0395000000000009,-0.667597689239384)
cB = chrono.ChVectorD(0.400348116076621,0.0395000000000009,-0.670268016790436)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.66533453693773e-16)
link_15.Initialize(body_53,body_52,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.389491351194624,0.0395000000000009,-0.667597689239384)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.400348116076621,0.0395000000000009,-0.670268016790436)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.66533453693773e-16)
link_16.Initialize(body_53,body_52,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_54 , SW name: robo_leg_link-4/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.445840190845338,0.0395000000000009,-0.715525853141396)
cB = chrono.ChVectorD(0.441462393251366,0.0395000000000071,-0.705238244662098)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(2.60035049048923e-14,-1,7.41073868937292e-15)
link_17.Initialize(body_53,body_54,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.445840190845338,0.0395000000000009,-0.715525853141396)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.441462393251366,0.0395000000000071,-0.705238244662098)
dB = chrono.ChVectorD(2.60035049048923e-14,-1,7.41073868937292e-15)
link_18.Initialize(body_53,body_54,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_52 , SW name: robo_leg_link-4/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-4/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.37635844314115,0.0495000000000009,-0.649863314354192)
cB = chrono.ChVectorD(0.381594994755063,0.0495000000000008,-0.657894569754445)
dA = chrono.ChVectorD(3.81639164714898e-17,1,1.66533453693773e-16)
dB = chrono.ChVectorD(3.12250225675825e-17,1,2.77555756156289e-16)
link_19.Initialize(body_52,body_51,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.37635844314115,0.0495000000000009,-0.649863314354192)
dA = chrono.ChVectorD(3.81639164714898e-17,1,1.66533453693773e-16)
cB = chrono.ChVectorD(0.381594994755063,0.0495000000000008,-0.657894569754445)
dB = chrono.ChVectorD(3.12250225675825e-17,1,2.77555756156289e-16)
link_20.Initialize(body_52,body_51,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_54 , SW name: robo_leg_link-4/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_55 , SW name: robo_leg_link-4/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.465452069071972,0.0495000000000075,-0.725642943706307)
cB = chrono.ChVectorD(0.456649446231942,0.0495000000000073,-0.722158219706524)
dA = chrono.ChVectorD(-2.60035049048923e-14,1,-7.41073868937292e-15)
dB = chrono.ChVectorD(-2.8345381597461e-14,1,-8.85402862138562e-15)
link_21.Initialize(body_54,body_55,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.465452069071972,0.0495000000000075,-0.725642943706307)
dA = chrono.ChVectorD(-2.60035049048923e-14,1,-7.41073868937292e-15)
cB = chrono.ChVectorD(0.456649446231942,0.0495000000000073,-0.722158219706524)
dB = chrono.ChVectorD(-2.8345381597461e-14,1,-8.85402862138562e-15)
link_22.Initialize(body_54,body_55,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_54 , SW name: robo_leg_link-4/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_55 , SW name: robo_leg_link-4/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.461643428338397,0.0495000000000075,-0.722403459833441)
dA = chrono.ChVectorD(2.60035049048923e-14,-1,7.41073868937292e-15)
cB = chrono.ChVectorD(0.461643428338455,0.0495000000000074,-0.722403459833404)
dB = chrono.ChVectorD(2.8345381597461e-14,-1,8.85402862138562e-15)
link_23.Initialize(body_54,body_55,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.461643428338397,0.0495000000000075,-0.722403459833441)
cB = chrono.ChVectorD(0.461643428338455,0.0495000000000074,-0.722403459833404)
dA = chrono.ChVectorD(2.60035049048923e-14,-1,7.41073868937292e-15)
dB = chrono.ChVectorD(2.8345381597461e-14,-1,8.85402862138562e-15)
link_24.Initialize(body_54,body_55,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_52 , SW name: robo_leg_link-4/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_51 , SW name: robo_leg_link-4/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.380167083416676,0.0495000000000009,-0.653102798765583)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.66533453693773e-16)
cB = chrono.ChVectorD(0.380167083416676,0.0495000000000008,-0.653102798765583)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-2.77555756156289e-16)
link_25.Initialize(body_52,body_51,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.380167083416676,0.0495000000000009,-0.653102798765583)
cB = chrono.ChVectorD(0.380167083416676,0.0495000000000008,-0.653102798765583)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.66533453693773e-16)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-2.77555756156289e-16)
link_26.Initialize(body_52,body_51,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_54 , SW name: robo_leg_link-4/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.445271033985,0.0545000000000009,-0.708477728534926)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.445271033984941,0.0495000000000071,-0.708477728534963)
dB = chrono.ChVectorD(2.60035049048923e-14,-1,7.41073868937292e-15)
link_27.Initialize(body_53,body_54,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.445271033985,0.0545000000000009,-0.708477728534926)
cB = chrono.ChVectorD(0.445271033984941,0.0495000000000071,-0.708477728534963)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(2.60035049048923e-14,-1,7.41073868937292e-15)
link_28.Initialize(body_53,body_54,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_53 , SW name: robo_leg_link-4/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_52 , SW name: robo_leg_link-4/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.396539475801095,0.0545000000000009,-0.667028532379046)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.396539475801095,0.0495000000000009,-0.667028532379046)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.66533453693773e-16)
link_29.Initialize(body_53,body_52,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.396539475801095,0.0545000000000009,-0.667028532379046)
cB = chrono.ChVectorD(0.396539475801095,0.0495000000000009,-0.667028532379046)
dA = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-3.81639164714898e-17,-1,-1.66533453693773e-16)
link_30.Initialize(body_53,body_52,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-6/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.365136825357394,0.0169999999999809,-0.404422618440478)
dA = chrono.ChVectorD(-0.283455363157293,-4.41660596983695e-14,-0.958985431118413)
cB = chrono.ChVectorD(0.362869182452134,0.0170000000000193,-0.412094501889423)
dB = chrono.ChVectorD(0.283455363157292,1.79925518928314e-14,0.958985431118413)
link_1.SetFlipped(True)
link_1.Initialize(body_61,body_62,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.365136825357394,0.0169999999999809,-0.404422618440478)
cB = chrono.ChVectorD(0.362869182452134,0.0170000000000193,-0.412094501889423)
dA = chrono.ChVectorD(-0.283455363157293,-4.41660596983695e-14,-0.958985431118413)
dB = chrono.ChVectorD(0.283455363157292,1.79925518928314e-14,0.958985431118413)
link_2.Initialize(body_61,body_62,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.329519723197398,0.0110000000000184,-0.431133286341607)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dA = chrono.ChVectorD(-2.69090305593522e-14,1,-6.92154666914746e-15)
dB = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
link_3.Initialize(body_62,body_60,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.329519723197398,0.0110000000000184,-0.431133286341607)
dA = chrono.ChVectorD(-2.69090305593522e-14,1,-6.92154666914746e-15)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dB = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_61 , SW name: robo_leg_link-6/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.362869182452136,0.0169999999999805,-0.412094501889425)
cB = chrono.ChVectorD(0.360950252604466,0.0149990000000193,-0.411527307707745)
dA = chrono.ChVectorD(-0.283455363157293,-4.41660596983695e-14,-0.958985431118413)
dB = chrono.ChVectorD(0.283455363157292,1.79925518928314e-14,0.958985431118413)
link_5.Initialize(body_61,body_62,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.362869182452136,0.0169999999999805,-0.412094501889425)
dA = chrono.ChVectorD(-0.283455363157293,-4.41660596983695e-14,-0.958985431118413)
cB = chrono.ChVectorD(0.360950252604466,0.0149990000000193,-0.411527307707745)
dB = chrono.ChVectorD(0.283455363157292,1.79925518928314e-14,0.958985431118413)
link_6.SetFlipped(True)
link_6.Initialize(body_61,body_62,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.338722928582102,0.0633100000000385,-0.420137763629272)
dA = chrono.ChVectorD(-0.95898543111841,-2.42028619368284e-14,0.283455363157302)
cB = chrono.ChVectorD(0.393689537190073,0.0610000000000083,-0.412187367194964)
dB = chrono.ChVectorD(0.95898543111841,2.30510055487798e-14,-0.283455363157303)
link_7.SetFlipped(True)
link_7.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.329519723197398,0.0110000000000184,-0.431133286341607)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dA = chrono.ChVectorD(-2.69090305593522e-14,1,-6.92154666914746e-15)
dB = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
link_8.Initialize(body_62,body_60,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.329519723197398,0.0110000000000184,-0.431133286341607)
dA = chrono.ChVectorD(-2.69090305593522e-14,1,-6.92154666914746e-15)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dB = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
link_9.SetFlipped(True)
link_9.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.331108186288715,0.0789500000000496,-0.436481893766731)
dA = chrono.ChVectorD(2.72420974667398e-14,-1,-2.09728068245596e-14)
cB = chrono.ChVectorD(0.365034893301768,0.0780000000000076,-0.411823259348937)
dB = chrono.ChVectorD(-2.60347299274599e-14,1,-6.75501321545369e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.365034893301768,0.07895000000005,-0.411823259348934)
dA = chrono.ChVectorD(2.72420974667398e-14,-1,-2.09728068245596e-14)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dB = chrono.ChVectorD(-2.60347299274599e-14,1,-6.75501321545369e-15)
link_11.SetFlipped(True)
link_11.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.365034893301768,0.07895000000005,-0.411823259348934)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dA = chrono.ChVectorD(2.72420974667398e-14,-1,-2.09728068245596e-14)
dB = chrono.ChVectorD(-2.60347299274599e-14,1,-6.75501321545369e-15)
link_12.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_62 , SW name: robo_leg_link-6/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.365034893301768,0.0110000000000195,-0.411823259348937)
dA = chrono.ChVectorD(-2.69090305593522e-14,1,-6.92154666914746e-15)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dB = chrono.ChVectorD(-2.60347299274599e-14,1,-6.75501321545369e-15)
link_13.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.365034893301768,0.0110000000000195,-0.411823259348937)
cB = chrono.ChVectorD(0.36503489330177,0.0110000000000076,-0.411823259348937)
dA = chrono.ChVectorD(-2.69090305593522e-14,1,-6.92154666914746e-15)
dB = chrono.ChVectorD(-2.60347299274599e-14,1,-6.75501321545369e-15)
link_14.Initialize(body_62,body_60,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_63 , SW name: robo_leg_link-6/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.370724271390958,0.039500000000008,-0.374935508899659)
cB = chrono.ChVectorD(0.372684646013834,0.039500000000008,-0.385942640351381)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.75501321545369e-15)
link_15.Initialize(body_60,body_63,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.370724271390958,0.039500000000008,-0.374935508899659)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
cB = chrono.ChVectorD(0.372684646013834,0.039500000000008,-0.385942640351381)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.75501321545369e-15)
link_16.Initialize(body_60,body_63,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_64 , SW name: robo_leg_link-6/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.349755660901397,0.039500000000007,-0.445876456166643)
cB = chrono.ChVectorD(0.357385141517002,0.0395000000000071,-0.437703878620699)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.77582989716541e-15)
link_17.Initialize(body_60,body_64,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.349755660901397,0.039500000000007,-0.445876456166643)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
cB = chrono.ChVectorD(0.357385141517002,0.0395000000000071,-0.437703878620699)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.77582989716541e-15)
link_18.Initialize(body_60,body_64,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_63 , SW name: robo_leg_link-6/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_58 , SW name: robo_leg_link-6/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.381611711436072,0.0495000000000084,-0.35574059088434)
cB = chrono.ChVectorD(0.375458458309543,0.0495000000000009,-0.358932234866319)
dA = chrono.ChVectorD(-2.60902410786912e-14,1,-6.75501321545369e-15)
dB = chrono.ChVectorD(1.2490009027033e-16,1,5.13478148889135e-16)
link_19.Initialize(body_63,body_58,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.381611711436072,0.0495000000000084,-0.35574059088434)
dA = chrono.ChVectorD(-2.60902410786912e-14,1,-6.75501321545369e-15)
cB = chrono.ChVectorD(0.375458458309543,0.0495000000000009,-0.358932234866319)
dB = chrono.ChVectorD(1.2490009027033e-16,1,5.13478148889135e-16)
link_20.Initialize(body_63,body_58,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_64 , SW name: robo_leg_link-6/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_59 , SW name: robo_leg_link-6/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.348458070253246,0.0495000000000066,-0.467905926361114)
cB = chrono.ChVectorD(0.349037875370793,0.0495000000000008,-0.458181633937842)
dA = chrono.ChVectorD(-2.60902410786912e-14,1,-6.77582989716541e-15)
dB = chrono.ChVectorD(-9.71445146547012e-17,1,1.57859836313889e-15)
link_21.Initialize(body_64,body_59,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.348458070253246,0.0495000000000066,-0.467905926361114)
dA = chrono.ChVectorD(-2.60902410786912e-14,1,-6.77582989716541e-15)
cB = chrono.ChVectorD(0.349037875370793,0.0495000000000008,-0.458181633937842)
dB = chrono.ChVectorD(-9.71445146547012e-17,1,1.57859836313889e-15)
link_22.Initialize(body_64,body_59,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_64 , SW name: robo_leg_link-6/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_59 , SW name: robo_leg_link-6/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.349875346897621,0.0495000000000067,-0.463110999154857)
dA = chrono.ChVectorD(2.60902410786912e-14,-1,6.77582989716541e-15)
cB = chrono.ChVectorD(0.349875346897788,0.0495000000000008,-0.463110999154731)
dB = chrono.ChVectorD(9.71445146547012e-17,-1,-1.57859836313889e-15)
link_23.Initialize(body_64,body_59,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.349875346897621,0.0495000000000067,-0.463110999154857)
cB = chrono.ChVectorD(0.349875346897788,0.0495000000000008,-0.463110999154731)
dA = chrono.ChVectorD(2.60902410786912e-14,-1,6.77582989716541e-15)
dB = chrono.ChVectorD(9.71445146547012e-17,-1,-1.57859836313889e-15)
link_24.Initialize(body_64,body_59,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_63 , SW name: robo_leg_link-6/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_58 , SW name: robo_leg_link-6/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.380194435719106,0.0495000000000083,-0.36053551836472)
dA = chrono.ChVectorD(2.60902410786912e-14,-1,6.75501321545369e-15)
cB = chrono.ChVectorD(0.380194435719082,0.0495000000000009,-0.360535518364747)
dB = chrono.ChVectorD(-1.2490009027033e-16,-1,-5.13478148889135e-16)
link_25.Initialize(body_63,body_58,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.380194435719106,0.0495000000000083,-0.36053551836472)
cB = chrono.ChVectorD(0.380194435719082,0.0495000000000009,-0.360535518364747)
dA = chrono.ChVectorD(2.60902410786912e-14,-1,6.75501321545369e-15)
dB = chrono.ChVectorD(-1.2490009027033e-16,-1,-5.13478148889135e-16)
link_26.Initialize(body_63,body_58,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_64 , SW name: robo_leg_link-6/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.355967864872775,0.0545000000000072,-0.442498805826837)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
cB = chrono.ChVectorD(0.355967864872627,0.049500000000007,-0.442498805826957)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.77582989716541e-15)
link_27.Initialize(body_60,body_64,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.355967864872775,0.0545000000000072,-0.442498805826837)
cB = chrono.ChVectorD(0.355967864872627,0.049500000000007,-0.442498805826957)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.77582989716541e-15)
link_28.Initialize(body_60,body_64,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_60 , SW name: robo_leg_link-6/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_63 , SW name: robo_leg_link-6/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.374101921730763,0.054500000000008,-0.381147712871037)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
cB = chrono.ChVectorD(0.3741019217308,0.049500000000008,-0.381147712871001)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.75501321545369e-15)
link_29.Initialize(body_60,body_63,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.374101921730763,0.054500000000008,-0.381147712871037)
cB = chrono.ChVectorD(0.3741019217308,0.049500000000008,-0.381147712871001)
dA = chrono.ChVectorD(2.60347299274599e-14,-1,6.75501321545369e-15)
dB = chrono.ChVectorD(2.60902410786912e-14,-1,6.75501321545369e-15)
link_30.Initialize(body_60,body_63,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_70 , SW name: robo_leg_link-8/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.618845571752761,0.017000000000038,-0.443565488381824)
dA = chrono.ChVectorD(-0.952241109025985,-5.30617216831786e-14,0.305347130788817)
cB = chrono.ChVectorD(0.611227642880554,0.0169999999999976,-0.441122711335516)
dB = chrono.ChVectorD(0.952241109025985,2.74641420716648e-14,-0.305347130788814)
link_1.SetFlipped(True)
link_1.Initialize(body_70,body_67,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.618845571752761,0.017000000000038,-0.443565488381824)
cB = chrono.ChVectorD(0.611227642880554,0.0169999999999976,-0.441122711335516)
dA = chrono.ChVectorD(-0.952241109025985,-5.30617216831786e-14,0.305347130788817)
dB = chrono.ChVectorD(0.952241109025985,2.74641420716648e-14,-0.305347130788814)
link_2.Initialize(body_70,body_67,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.592957742980357,0.0109999999999973,-0.40734590711077)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dA = chrono.ChVectorD(-2.65586164172049e-14,1,-4.98212582300539e-15)
dB = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
link_3.Initialize(body_67,body_65,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.592957742980357,0.0109999999999973,-0.40734590711077)
dA = chrono.ChVectorD(-2.65586164172049e-14,1,-4.98212582300539e-15)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dB = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_70 , SW name: robo_leg_link-8/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.611227642880553,0.0170000000000375,-0.441122711335513)
cB = chrono.ChVectorD(0.611838642489263,0.0149989999999976,-0.439217276876355)
dA = chrono.ChVectorD(-0.952241109025985,-5.30617216831786e-14,0.305347130788817)
dB = chrono.ChVectorD(0.952241109025985,2.74641420716648e-14,-0.305347130788814)
link_5.Initialize(body_70,body_67,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.611227642880553,0.0170000000000375,-0.441122711335513)
dA = chrono.ChVectorD(-0.952241109025985,-5.30617216831786e-14,0.305347130788817)
cB = chrono.ChVectorD(0.611838642489263,0.0149989999999976,-0.439217276876355)
dB = chrono.ChVectorD(0.952241109025985,2.74641420716648e-14,-0.305347130788814)
link_6.SetFlipped(True)
link_6.Initialize(body_70,body_67,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.603739575877097,0.0633100000000174,-0.416798556922188)
dA = chrono.ChVectorD(0.305347130788825,1.25038868148408e-14,0.952241109025982)
cB = chrono.ChVectorD(0.61042884320734,0.0610000000000135,-0.471932852672739)
dB = chrono.ChVectorD(-0.305347130788828,-1.43773881688958e-14,-0.952241109025981)
link_7.SetFlipped(True)
link_7.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.592957742980357,0.0109999999999973,-0.40734590711077)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dA = chrono.ChVectorD(-2.65586164172049e-14,1,-4.98212582300539e-15)
dB = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
link_8.Initialize(body_67,body_65,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.592957742980357,0.0109999999999973,-0.40734590711077)
dA = chrono.ChVectorD(-2.65586164172049e-14,1,-4.98212582300539e-15)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dB = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
link_9.SetFlipped(True)
link_9.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.587574154156022,0.0789500000000284,-0.408811440375858)
dA = chrono.ChVectorD(-1.3392065234541e-15,-1,5.30131494258512e-15)
cB = chrono.ChVectorD(0.611449207348265,0.0780000000000137,-0.44329406694588)
dB = chrono.ChVectorD(-2.60243215866041e-14,1,-6.77236045021345e-15)
link_10.SetFlipped(True)
link_10.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.611449207348269,0.0789500000000282,-0.443294066945878)
dA = chrono.ChVectorD(-1.3392065234541e-15,-1,5.30131494258512e-15)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dB = chrono.ChVectorD(-2.60243215866041e-14,1,-6.77236045021345e-15)
link_11.SetFlipped(True)
link_11.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.611449207348269,0.0789500000000282,-0.443294066945878)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dA = chrono.ChVectorD(-1.3392065234541e-15,-1,5.30131494258512e-15)
dB = chrono.ChVectorD(-2.60243215866041e-14,1,-6.77236045021345e-15)
link_12.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_67 , SW name: robo_leg_link-8/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.611449207348268,0.0109999999999977,-0.443294066945875)
dA = chrono.ChVectorD(-2.65586164172049e-14,1,-4.98212582300539e-15)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dB = chrono.ChVectorD(-2.60243215866041e-14,1,-6.77236045021345e-15)
link_13.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.611449207348268,0.0109999999999977,-0.443294066945875)
cB = chrono.ChVectorD(0.611449207348267,0.0110000000000137,-0.443294066945879)
dA = chrono.ChVectorD(-2.65586164172049e-14,1,-4.98212582300539e-15)
dB = chrono.ChVectorD(-2.60243215866041e-14,1,-6.77236045021345e-15)
link_14.Initialize(body_67,body_65,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_66 , SW name: robo_leg_link-8/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.648196961022309,0.0395000000000146,-0.449826888400801)
cB = chrono.ChVectorD(0.637147813997148,0.0395000000000008,-0.451534623514255)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.38777878078145e-16)
link_15.Initialize(body_65,body_66,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.648196961022309,0.0395000000000146,-0.449826888400801)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
cB = chrono.ChVectorD(0.637147813997148,0.0395000000000008,-0.451534623514255)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.38777878078145e-16)
link_16.Initialize(body_65,body_66,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_68 , SW name: robo_leg_link-8/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.577754924982112,0.039500000000013,-0.427238834400698)
cB = chrono.ChVectorD(0.585750599681712,0.0395000000000008,-0.435053513551232)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-2.22044604925031e-16)
link_17.Initialize(body_65,body_68,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.577754924982112,0.039500000000013,-0.427238834400698)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
cB = chrono.ChVectorD(0.585750599681712,0.0395000000000008,-0.435053513551232)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-2.22044604925031e-16)
link_18.Initialize(body_65,body_68,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_66 , SW name: robo_leg_link-8/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_69 , SW name: robo_leg_link-8/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.667137459194324,0.0495000000000008,-0.461151144195354)
cB = chrono.ChVectorD(0.657412958259187,0.0494999999999969,-0.459019676699296)
dA = chrono.ChVectorD(3.12250225675825e-17,1,1.38777878078145e-16)
dB = chrono.ChVectorD(2.06848427275474e-14,1,2.11081152556858e-14)
link_19.Initialize(body_66,body_69,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.667137459194324,0.0495000000000008,-0.461151144195354)
dA = chrono.ChVectorD(3.12250225675825e-17,1,1.38777878078145e-16)
cB = chrono.ChVectorD(0.657412958259187,0.0494999999999969,-0.459019676699296)
dB = chrono.ChVectorD(2.06848427275474e-14,1,2.11081152556858e-14)
link_20.Initialize(body_66,body_69,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_68 , SW name: robo_leg_link-8/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_71 , SW name: robo_leg_link-8/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.555760960894817,0.0495000000000008,-0.425436972879344)
cB = chrono.ChVectorD(0.565419334498346,0.0494999999999981,-0.427972540743152)
dA = chrono.ChVectorD(1.73472347597681e-17,1,2.22044604925031e-16)
dB = chrono.ChVectorD(2.0566881531181e-14,1,2.07889261361061e-14)
link_21.Initialize(body_68,body_71,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.555760960894817,0.0495000000000008,-0.425436972879344)
dA = chrono.ChVectorD(1.73472347597681e-17,1,2.22044604925031e-16)
cB = chrono.ChVectorD(0.565419334498346,0.0494999999999981,-0.427972540743152)
dB = chrono.ChVectorD(2.0566881531181e-14,1,2.07889261361061e-14)
link_22.Initialize(body_68,body_71,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_68 , SW name: robo_leg_link-8/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_71 , SW name: robo_leg_link-8/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.560522165703215,0.0495000000000008,-0.426963710830825)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-2.22044604925031e-16)
cB = chrono.ChVectorD(0.560522165703216,0.0494999999999982,-0.426963710830823)
dB = chrono.ChVectorD(-2.0566881531181e-14,-1,-2.07889261361061e-14)
link_23.Initialize(body_68,body_71,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.560522165703215,0.0495000000000008,-0.426963710830825)
cB = chrono.ChVectorD(0.560522165703216,0.0494999999999982,-0.426963710830823)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-2.22044604925031e-16)
dB = chrono.ChVectorD(-2.0566881531181e-14,-1,-2.07889261361061e-14)
link_24.Initialize(body_68,body_71,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_66 , SW name: robo_leg_link-8/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_69 , SW name: robo_leg_link-8/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.662376253368219,0.0495000000000008,-0.459624409417645)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.38777878078145e-16)
cB = chrono.ChVectorD(0.662376253368221,0.0494999999999968,-0.459624409417643)
dB = chrono.ChVectorD(-2.06848427275474e-14,-1,-2.11081152556858e-14)
link_25.Initialize(body_66,body_69,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.662376253368219,0.0495000000000008,-0.459624409417645)
cB = chrono.ChVectorD(0.662376253368221,0.0494999999999968,-0.459624409417643)
dA = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.38777878078145e-16)
dB = chrono.ChVectorD(-2.06848427275474e-14,-1,-2.11081152556858e-14)
link_26.Initialize(body_66,body_69,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_68 , SW name: robo_leg_link-8/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.580989394873297,0.054500000000013,-0.433526775599772)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
cB = chrono.ChVectorD(0.580989394873314,0.0495000000000008,-0.433526775599752)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-2.22044604925031e-16)
link_27.Initialize(body_65,body_68,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.580989394873297,0.054500000000013,-0.433526775599772)
cB = chrono.ChVectorD(0.580989394873314,0.0495000000000008,-0.433526775599752)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-2.22044604925031e-16)
link_28.Initialize(body_65,body_68,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_65 , SW name: robo_leg_link-8/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_66 , SW name: robo_leg_link-8/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.641909019823234,0.0545000000000145,-0.453061358291987)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
cB = chrono.ChVectorD(0.641909019823252,0.0495000000000008,-0.453061358291965)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.38777878078145e-16)
link_29.Initialize(body_65,body_66,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.641909019823234,0.0545000000000145,-0.453061358291987)
cB = chrono.ChVectorD(0.641909019823252,0.0495000000000008,-0.453061358291965)
dA = chrono.ChVectorD(2.60243215866041e-14,-1,6.77236045021345e-15)
dB = chrono.ChVectorD(-3.12250225675825e-17,-1,-1.38777878078145e-16)
link_30.Initialize(body_65,body_66,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_73 , SW name: robo_leg_link-9/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.764761463500638,0.017000000000001,-0.475385920542641)
dA = chrono.ChVectorD(-0.978433427985959,-3.03368441478824e-14,0.206562404613341)
cB = chrono.ChVectorD(0.75693399607675,0.0170000000000008,-0.473733421305734)
dB = chrono.ChVectorD(0.978433427985959,3.03229663600746e-14,-0.206562404613341)
link_1.SetFlipped(True)
link_1.Initialize(body_73,body_74,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.764761463500638,0.017000000000001,-0.475385920542641)
cB = chrono.ChVectorD(0.75693399607675,0.0170000000000008,-0.473733421305734)
dA = chrono.ChVectorD(-0.978433427985959,-3.03368441478824e-14,0.206562404613341)
dB = chrono.ChVectorD(0.978433427985959,3.03229663600746e-14,-0.206562404613341)
link_2.Initialize(body_73,body_74,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.735312088880478,0.0110000000000003,-0.441997717828486)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dA = chrono.ChVectorD(-2.77694534034367e-14,1,-2.42861286636753e-15)
dB = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
link_3.Initialize(body_74,body_77,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.735312088880478,0.0110000000000003,-0.441997717828486)
dA = chrono.ChVectorD(-2.77694534034367e-14,1,-2.42861286636753e-15)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dB = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_73 , SW name: robo_leg_link-9/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.75693399607675,0.0170000000000007,-0.473733421305734)
cB = chrono.ChVectorD(0.757347327448381,0.0149990000000008,-0.471775576016334)
dA = chrono.ChVectorD(-0.978433427985959,-3.03368441478824e-14,0.206562404613341)
dB = chrono.ChVectorD(0.978433427985959,3.03229663600746e-14,-0.206562404613341)
link_5.Initialize(body_73,body_74,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.75693399607675,0.0170000000000007,-0.473733421305734)
dA = chrono.ChVectorD(-0.978433427985959,-3.03368441478824e-14,0.206562404613341)
cB = chrono.ChVectorD(0.757347327448381,0.0149990000000008,-0.471775576016334)
dB = chrono.ChVectorD(0.978433427985959,3.03229663600746e-14,-0.206562404613341)
link_6.SetFlipped(True)
link_6.Initialize(body_73,body_74,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.747002397741506,0.0633100000000204,-0.450300559980683)
dA = chrono.ChVectorD(0.206562404613352,7.79931674799172e-15,0.978433427985957)
cB = chrono.ChVectorD(0.759283993491693,0.0610000000000008,-0.504464194193627)
dB = chrono.ChVectorD(-0.206562404613352,1.80411241501588e-16,-0.978433427985957)
link_7.SetFlipped(True)
link_7.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.735312088880478,0.0110000000000003,-0.441997717828486)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dA = chrono.ChVectorD(-2.77694534034367e-14,1,-2.42861286636753e-15)
dB = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
link_8.Initialize(body_74,body_77,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.735312088880478,0.0110000000000003,-0.441997717828486)
dA = chrono.ChVectorD(-2.77694534034367e-14,1,-2.42861286636753e-15)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dB = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.730106193499942,0.0789500000000314,-0.444005071725412)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-8.32667268468867e-17)
cB = chrono.ChVectorD(0.757376022083587,0.0780000000000008,-0.475870823703338)
dB = chrono.ChVectorD(2.42861286636753e-17,1,1.94289029309402e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.757376022083587,0.0789500000000314,-0.475870823703338)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-8.32667268468867e-17)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dB = chrono.ChVectorD(2.42861286636753e-17,1,1.94289029309402e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.757376022083587,0.0789500000000314,-0.475870823703338)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-8.32667268468867e-17)
dB = chrono.ChVectorD(2.42861286636753e-17,1,1.94289029309402e-16)
link_12.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_74 , SW name: robo_leg_link-9/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.757376022083586,0.0110000000000009,-0.475870823703336)
dA = chrono.ChVectorD(-2.77694534034367e-14,1,-2.42861286636753e-15)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dB = chrono.ChVectorD(2.42861286636753e-17,1,1.94289029309402e-16)
link_13.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.757376022083586,0.0110000000000009,-0.475870823703336)
cB = chrono.ChVectorD(0.757376022083587,0.0110000000000008,-0.475870823703338)
dA = chrono.ChVectorD(-2.77694534034367e-14,1,-2.42861286636753e-15)
dB = chrono.ChVectorD(2.42861286636753e-17,1,1.94289029309402e-16)
link_14.Initialize(body_74,body_77,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_75 , SW name: robo_leg_link-9/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.794598640524284,0.0395000000000009,-0.478618883504044)
cB = chrono.ChVectorD(0.783781489018678,0.0395000000000008,-0.481445425499475)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.80411241501588e-16)
link_15.Initialize(body_77,body_75,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.794598640524284,0.0395000000000009,-0.478618883504044)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.783781489018678,0.0395000000000008,-0.481445425499475)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.80411241501588e-16)
link_16.Initialize(body_77,body_75,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_76 , SW name: robo_leg_link-9/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.722219027689023,0.0395000000000009,-0.463338429622772)
cB = chrono.ChVectorD(0.730970551226868,0.0395000000000009,-0.470296223389426)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.4980018054066e-16)
link_17.Initialize(body_77,body_76,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.722219027689023,0.0395000000000009,-0.463338429622772)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.730970551226868,0.0395000000000009,-0.470296223389426)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.4980018054066e-16)
link_18.Initialize(body_77,body_76,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_75 , SW name: robo_leg_link-9/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_72 , SW name: robo_leg_link-9/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.814596026791311,0.0495000000000008,-0.487950850229766)
cB = chrono.ChVectorD(0.804834730859546,0.0494999999999948,-0.485781559078644)
dA = chrono.ChVectorD(1.73472347597681e-17,1,1.80411241501588e-16)
dB = chrono.ChVectorD(2.12226070051003e-14,1,2.32175390024736e-14)
link_19.Initialize(body_75,body_72,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.814596026791311,0.0495000000000008,-0.487950850229766)
dA = chrono.ChVectorD(1.73472347597681e-17,1,1.80411241501588e-16)
cB = chrono.ChVectorD(0.804834730859546,0.0494999999999948,-0.485781559078644)
dB = chrono.ChVectorD(2.12226070051003e-14,1,2.32175390024736e-14)
link_20.Initialize(body_75,body_72,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_76 , SW name: robo_leg_link-9/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_78 , SW name: robo_leg_link-9/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.700156016395106,0.0495000000000008,-0.463790784728995)
cB = chrono.ChVectorD(0.710011478177174,0.0494999999999959,-0.46542833168198)
dA = chrono.ChVectorD(2.08166817117217e-17,1,2.4980018054066e-16)
dB = chrono.ChVectorD(2.05876982128927e-14,1,2.08166817117217e-14)
link_21.Initialize(body_76,body_78,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.700156016395106,0.0495000000000008,-0.463790784728995)
dA = chrono.ChVectorD(2.08166817117217e-17,1,2.4980018054066e-16)
cB = chrono.ChVectorD(0.710011478177174,0.0494999999999959,-0.46542833168198)
dB = chrono.ChVectorD(2.05876982128927e-14,1,2.08166817117217e-14)
link_22.Initialize(body_76,body_78,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_76 , SW name: robo_leg_link-9/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_78 , SW name: robo_leg_link-9/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.705048183068139,0.0495000000000009,-0.464823598963634)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.4980018054066e-16)
cB = chrono.ChVectorD(0.70504818306814,0.049499999999996,-0.464823598963633)
dB = chrono.ChVectorD(-2.05876982128927e-14,-1,-2.08166817117217e-14)
link_23.Initialize(body_76,body_78,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.705048183068139,0.0495000000000009,-0.464823598963634)
cB = chrono.ChVectorD(0.70504818306814,0.049499999999996,-0.464823598963633)
dA = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.4980018054066e-16)
dB = chrono.ChVectorD(-2.05876982128927e-14,-1,-2.08166817117217e-14)
link_24.Initialize(body_76,body_78,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_75 , SW name: robo_leg_link-9/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_72 , SW name: robo_leg_link-9/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.809703859651381,0.0495000000000008,-0.4869180382067)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.80411241501588e-16)
cB = chrono.ChVectorD(0.809703859651383,0.0494999999999948,-0.486918038206698)
dB = chrono.ChVectorD(-2.12226070051003e-14,-1,-2.32175390024736e-14)
link_25.Initialize(body_75,body_72,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.809703859651381,0.0495000000000008,-0.4869180382067)
cB = chrono.ChVectorD(0.809703859651383,0.0494999999999948,-0.486918038206698)
dA = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.80411241501588e-16)
dB = chrono.ChVectorD(-2.12226070051003e-14,-1,-2.32175390024736e-14)
link_26.Initialize(body_75,body_72,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_76 , SW name: robo_leg_link-9/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.726078382805886,0.0545000000000008,-0.469263408785768)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.726078384553836,0.0495000000000009,-0.469263409154787)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.4980018054066e-16)
link_27.Initialize(body_77,body_76,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.726078382805886,0.0545000000000008,-0.469263408785768)
cB = chrono.ChVectorD(0.726078384553836,0.0495000000000009,-0.469263409154787)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-2.08166817117217e-17,-1,-2.4980018054066e-16)
link_28.Initialize(body_77,body_76,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_77 , SW name: robo_leg_link-9/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_75 , SW name: robo_leg_link-9/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.788673661361288,0.0545000000000009,-0.482478238620907)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
cB = chrono.ChVectorD(0.788673656158608,0.0495000000000008,-0.482478237522541)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.80411241501588e-16)
link_29.Initialize(body_77,body_75,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.788673661361288,0.0545000000000009,-0.482478238620907)
cB = chrono.ChVectorD(0.788673656158608,0.0495000000000008,-0.482478237522541)
dA = chrono.ChVectorD(-2.42861286636753e-17,-1,-1.94289029309402e-16)
dB = chrono.ChVectorD(-1.73472347597681e-17,-1,-1.80411241501588e-16)
link_30.Initialize(body_77,body_75,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_85 , SW name: robo_leg_link-12/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.978392170932625,0.0170000000000011,-0.744838757996433)
dA = chrono.ChVectorD(0.598616334859908,-3.04756220259605e-14,0.801035881617603)
cB = chrono.ChVectorD(0.983181101611505,0.0170000000000008,-0.738430470943492)
dB = chrono.ChVectorD(-0.598616334859908,3.05311331771918e-14,-0.801035881617604)
link_1.SetFlipped(True)
link_1.Initialize(body_85,body_82,False,cA,cB,dA,dB)
link_1.SetName("Concentric2")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.978392170932625,0.0170000000000011,-0.744838757996433)
cB = chrono.ChVectorD(0.983181101611505,0.0170000000000008,-0.738430470943492)
dA = chrono.ChVectorD(0.598616334859908,-3.04756220259605e-14,0.801035881617603)
dB = chrono.ChVectorD(-0.598616334859908,3.05311331771918e-14,-0.801035881617604)
link_2.Initialize(body_85,body_82,False,cA,cB,dA,dB)
link_2.SetName("Concentric2")
exported_items.append(link_2)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.02106489777961,0.0110000000000004,-0.732147180890239)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dA = chrono.ChVectorD(9.4091401336982e-15,1,2.64337163269346e-14)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
link_3.Initialize(body_82,body_83,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident4")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.02106489777961,0.0110000000000004,-0.732147180890239)
dA = chrono.ChVectorD(9.4091401336982e-15,1,2.64337163269346e-14)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
link_4.SetFlipped(True)
link_4.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_4.SetName("Coincident4")
exported_items.append(link_4)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_85 , SW name: robo_leg_link-12/leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.983181101611505,0.0170000000000008,-0.738430470943492)
cB = chrono.ChVectorD(0.984783974410622,0.0149990000000009,-0.739628302229547)
dA = chrono.ChVectorD(0.598616334859908,-3.04756220259605e-14,0.801035881617603)
dB = chrono.ChVectorD(-0.598616334859908,3.05311331771918e-14,-0.801035881617604)
link_5.Initialize(body_85,body_82,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident7")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.983181101611505,0.0170000000000008,-0.738430470943492)
dA = chrono.ChVectorD(0.598616334859908,-3.04756220259605e-14,0.801035881617603)
cB = chrono.ChVectorD(0.984783974410622,0.0149990000000009,-0.739628302229547)
dB = chrono.ChVectorD(-0.598616334859908,3.05311331771918e-14,-0.801035881617604)
link_6.SetFlipped(True)
link_6.Initialize(body_85,body_82,False,cA,cB,dA,dB)
link_6.SetName("Coincident7")
exported_items.append(link_6)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.00861803720877,0.0633100000000205,-0.739265900648767)
dA = chrono.ChVectorD(0.801035881617597,7.9658502016855e-15,-0.598616334859916)
cB = chrono.ChVectorD(0.954308063792265,0.0610000000000009,-0.727648514848875)
dB = chrono.ChVectorD(-0.801035881617597,-6.93889390390723e-17,0.598616334859916)
link_7.SetFlipped(True)
link_7.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_7.SetName("Parallel6")
exported_items.append(link_7)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_8 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.02106489777961,0.0110000000000004,-0.732147180890239)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dA = chrono.ChVectorD(9.4091401336982e-15,1,2.64337163269346e-14)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
link_8.Initialize(body_82,body_83,False,cA,cB,dB)
link_8.SetDistance(0)
link_8.SetName("Coincident25")
exported_items.append(link_8)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.02106489777961,0.0110000000000004,-0.732147180890239)
dA = chrono.ChVectorD(9.4091401336982e-15,1,2.64337163269346e-14)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_9.SetName("Coincident25")
exported_items.append(link_9)


# Mate constraint: Parallel8 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.02143113856841,0.0789500000000315,-0.726579714315108)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.56125112837913e-16)
cB = chrono.ChVectorD(0.9810558391716,0.0780000000000009,-0.73793334363431)
dB = chrono.ChVectorD(1.38777878078145e-17,1,1.70002900645727e-16)
link_10.SetFlipped(True)
link_10.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_10.SetName("Parallel8")
exported_items.append(link_10)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.9810558391716,0.0789500000000315,-0.73793334363431)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.56125112837913e-16)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dB = chrono.ChVectorD(1.38777878078145e-17,1,1.70002900645727e-16)
link_11.SetFlipped(True)
link_11.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_11.SetName("Concentric15")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.9810558391716,0.0789500000000315,-0.73793334363431)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.56125112837913e-16)
dB = chrono.ChVectorD(1.38777878078145e-17,1,1.70002900645727e-16)
link_12.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_12.SetName("Concentric15")
exported_items.append(link_12)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_82 , SW name: robo_leg_link-12/single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.981055839171603,0.0110000000000009,-0.73793334363431)
dA = chrono.ChVectorD(9.4091401336982e-15,1,2.64337163269346e-14)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dB = chrono.ChVectorD(1.38777878078145e-17,1,1.70002900645727e-16)
link_13.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_13.SetName("Concentric16")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.981055839171603,0.0110000000000009,-0.73793334363431)
cB = chrono.ChVectorD(0.9810558391716,0.0110000000000009,-0.73793334363431)
dA = chrono.ChVectorD(9.4091401336982e-15,1,2.64337163269346e-14)
dB = chrono.ChVectorD(1.38777878078145e-17,1,1.70002900645727e-16)
link_14.Initialize(body_82,body_83,False,cA,cB,dA,dB)
link_14.SetName("Concentric16")
exported_items.append(link_14)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_81 , SW name: robo_leg_link-12/link2-3 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.962919696894057,0.0395000000000009,-0.770554739979941)
cB = chrono.ChVectorD(0.964900683650233,0.0395000000000009,-0.75955130159362)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.59594559789866e-16)
link_15.Initialize(body_83,body_81,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident28")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.962919696894057,0.0395000000000009,-0.770554739979941)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
cB = chrono.ChVectorD(0.964900683650233,0.0395000000000009,-0.75955130159362)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.59594559789866e-16)
link_16.Initialize(body_83,body_81,False,cA,cB,dA,dB)
link_16.SetName("Coincident28")
exported_items.append(link_16)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_84 , SW name: robo_leg_link-12/link2-4 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.00720234026532,0.0395000000000009,-0.711298110637279)
cB = chrono.ChVectorD(0.997210997486832,0.0395000000000009,-0.716315387762864)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.59594559789866e-16)
link_17.Initialize(body_83,body_84,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident29")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.00720234026532,0.0395000000000009,-0.711298110637279)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
cB = chrono.ChVectorD(0.997210997486832,0.0395000000000009,-0.716315387762864)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.59594559789866e-16)
link_18.Initialize(body_83,body_84,False,cA,cB,dA,dB)
link_18.SetName("Coincident29")
exported_items.append(link_18)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_81 , SW name: robo_leg_link-12/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_80 , SW name: robo_leg_link-12/link3-2 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.94604799271907,0.0495000000000009,-0.784778912136765)
cB = chrono.ChVectorD(0.953973486179033,0.0494999999999976,-0.781593089305995)
dA = chrono.ChVectorD(1.38777878078145e-17,1,1.59594559789866e-16)
dB = chrono.ChVectorD(2.05807593189888e-14,1,2.07819872422021e-14)
link_19.Initialize(body_81,body_80,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident30")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.94604799271907,0.0495000000000009,-0.784778912136765)
dA = chrono.ChVectorD(1.38777878078145e-17,1,1.59594559789866e-16)
cB = chrono.ChVectorD(0.953973486179033,0.0494999999999976,-0.781593089305995)
dB = chrono.ChVectorD(2.05807593189888e-14,1,2.07819872422021e-14)
link_20.Initialize(body_81,body_80,False,cA,cB,dA,dB)
link_20.SetName("Coincident30")
exported_items.append(link_20)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_84 , SW name: robo_leg_link-12/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_79 , SW name: robo_leg_link-12/link3-3 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.01606367082014,0.0495000000000009,-0.691087764068785)
cB = chrono.ChVectorD(1.01243861897051,0.0494999999999946,-0.700052844036705)
dA = chrono.ChVectorD(4.16333634234434e-17,1,1.59594559789866e-16)
dB = chrono.ChVectorD(2.05807593189888e-14,1,2.0646678811076e-14)
link_21.Initialize(body_84,body_79,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident31")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.01606367082014,0.0495000000000009,-0.691087764068785)
dA = chrono.ChVectorD(4.16333634234434e-17,1,1.59594559789866e-16)
cB = chrono.ChVectorD(1.01243861897051,0.0494999999999946,-0.700052844036705)
dB = chrono.ChVectorD(2.05807593189888e-14,1,2.0646678811076e-14)
link_22.Initialize(body_84,body_79,False,cA,cB,dA,dB)
link_22.SetName("Coincident31")
exported_items.append(link_22)


# Mate constraint: Concentric21 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_84 , SW name: robo_leg_link-12/link2-4 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_79 , SW name: robo_leg_link-12/link3-3 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.01307058912404,0.0495000000000009,-0.695092943460581)
dA = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.59594559789866e-16)
cB = chrono.ChVectorD(1.01307058912404,0.0494999999999945,-0.69509294346058)
dB = chrono.ChVectorD(-2.05807593189888e-14,-1,-2.0646678811076e-14)
link_23.Initialize(body_84,body_79,False,cA,cB,dA,dB)
link_23.SetName("Concentric21")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.01307058912404,0.0495000000000009,-0.695092943460581)
cB = chrono.ChVectorD(1.01307058912404,0.0494999999999945,-0.69509294346058)
dA = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.59594559789866e-16)
dB = chrono.ChVectorD(-2.05807593189888e-14,-1,-2.0646678811076e-14)
link_24.Initialize(body_84,body_79,False,cA,cB,dA,dB)
link_24.SetName("Concentric21")
exported_items.append(link_24)


# Mate constraint: Concentric22 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_81 , SW name: robo_leg_link-12/link2-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_80 , SW name: robo_leg_link-12/link3-2 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.949041077209035,0.0495000000000009,-0.780773734832832)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.59594559789866e-16)
cB = chrono.ChVectorD(0.949041077209037,0.0494999999999977,-0.780773734832831)
dB = chrono.ChVectorD(-2.05807593189888e-14,-1,-2.07819872422021e-14)
link_25.Initialize(body_81,body_80,False,cA,cB,dA,dB)
link_25.SetName("Concentric22")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.949041077209035,0.0495000000000009,-0.780773734832832)
cB = chrono.ChVectorD(0.949041077209037,0.0494999999999977,-0.780773734832831)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.59594559789866e-16)
dB = chrono.ChVectorD(-2.05807593189888e-14,-1,-2.07819872422021e-14)
link_26.Initialize(body_81,body_80,False,cA,cB,dA,dB)
link_26.SetName("Concentric22")
exported_items.append(link_26)


# Mate constraint: Concentric23 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_84 , SW name: robo_leg_link-12/link2-4 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.00020407918293,0.0545000000000009,-0.712310208371067)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
cB = chrono.ChVectorD(1.00020407918293,0.0495000000000009,-0.712310208371067)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.59594559789866e-16)
link_27.Initialize(body_83,body_84,False,cA,cB,dA,dB)
link_27.SetName("Concentric23")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.00020407918293,0.0545000000000009,-0.712310208371067)
cB = chrono.ChVectorD(1.00020407918293,0.0495000000000009,-0.712310208371067)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
dB = chrono.ChVectorD(-4.16333634234434e-17,-1,-1.59594559789866e-16)
link_28.Initialize(body_83,body_84,False,cA,cB,dA,dB)
link_28.SetName("Concentric23")
exported_items.append(link_28)


# Mate constraint: Concentric24 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_83 , SW name: robo_leg_link-12/link1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_81 , SW name: robo_leg_link-12/link2-3 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.961907599160269,0.0545000000000009,-0.763556478897553)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
cB = chrono.ChVectorD(0.961907599160268,0.0495000000000009,-0.763556478897553)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.59594559789866e-16)
link_29.Initialize(body_83,body_81,False,cA,cB,dA,dB)
link_29.SetName("Concentric24")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.961907599160269,0.0545000000000009,-0.763556478897553)
cB = chrono.ChVectorD(0.961907599160268,0.0495000000000009,-0.763556478897553)
dA = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.70002900645727e-16)
dB = chrono.ChVectorD(-1.38777878078145e-17,-1,-1.59594559789866e-16)
link_30.Initialize(body_83,body_81,False,cA,cB,dA,dB)
link_30.SetName("Concentric24")
exported_items.append(link_30)

