# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\Amin\Documents\SolidCAD\sim2\SinlgeBotAssem.SLDASM


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
body_1.SetName('leg-1')
body_1.SetPos(chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079))
body_1.SetRot(chrono.ChQuaternionD(0.881215075431449,-1.96188308486009e-15,0.472715549598641,1.24441122555222e-17))
body_1.SetMass(0.01)
body_1.SetInertiaXX(chrono.ChVectorD(1.89110723602526e-07,5.11889399343246e-07,5.94333456279105e-07))
body_1.SetInertiaXY(chrono.ChVectorD(-3.88054984756532e-11,-6.97451787573893e-24,-2.11470064202402e-24))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-5.02229192063373e-07,0.00570468434844058,0.004),chrono.ChQuaternionD(1,0,0,0)))

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
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0125,0,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.0120740728286134,0.00323523806378145,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00836413257948577,0.0092893103184674,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(0.00195543081300292,0.0123461042574392,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.00508420803844748,0.0114193182205325,0.00399999999999999),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.00178981093410736,0.00178981093410736,0.00400000000000001,chrono.ChVectorD(-0.0104833820993178,0.00680798793768785,0.00399999999999999),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('graound-1')
body_2.SetPos(chrono.ChVectorD(0.25,-1.10171934447595e-17,-0.25))
body_2.SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,0,0))
body_2.SetMass(160.310974449907)
body_2.SetInertiaXX(chrono.ChVectorD(213.929220761236,427.825165923698,213.899655627765))
body_2.SetInertiaXY(chrono.ChVectorD(0.000941966655763467,0.00121521831061318,0.000370669095243679))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.75010692404897,1.75004207520538,0.00504630746441585),chrono.ChQuaternionD(1,0,0,0)))
body_2.SetBodyFixed(True)

# Visualization shape 
# body_2_1_shape = chrono.ChObjShapeFile()
# body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
# body_2_1_level = chrono.ChAssetLevel()
# body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
# body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
# body_2_1_level.GetAssets().push_back(body_2_1_shape)
# body_2.GetAssets().push_back(body_2_1_level)

# Auxiliary marker (coordinate system feature)
marker_2_1 =chrono.ChMarker()
marker_2_1.SetName('ground_marker')
body_2.AddMarker(marker_2_1)
marker_2_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(4,0.00999999999999983,-4),chrono.ChQuaternionD(0.707106781186548,1.02176270012416E-16,-0.707106781186547,1.55806643888669E-17)))

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=5.89786600208644E-17; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(0.0305042045565396,1.88241479923824,0.05,chrono.ChVectorD(-0.219495795443461,1.75342339406911,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(0.0413523486386163,1.87726618574267,0.05,chrono.ChVectorD(3.70864765136138,1.73439272589085,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(2,0.0450620362723377,0.05,chrono.ChVectorD(1.75,3.70493796372766,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=6.93889390390723E-18; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(2,0.040792554490671,0.05,chrono.ChVectorD(1.75,-0.209207445509329,0.06),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=9.71445146547012E-17; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(2,2,0.00500000000005684,chrono.ChVectorD(1.75,1.75,0.00499999999994316),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('single_bot1-1')
body_3.SetPos(chrono.ChVectorD(1.47825488624892,-0.106920788463152,-1.7169521749262))
body_3.SetRot(chrono.ChQuaternionD(0.701844866726147,-0.709850830967959,-1.48299736156966e-17,0.0593766016563365))
body_3.SetMass(0.35)
body_3.SetInertiaXX(chrono.ChVectorD(0.000243287100418674,0.000250960901009867,0.000110709858882453))
body_3.SetInertiaXY(chrono.ChVectorD(4.8045401853377e-06,-2.33417013594839e-05,-1.75548754038152e-06))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0658557785731179,-0.0789654441773446,0.173168884203731),chrono.ChQuaternionD(1,0,0,0)))

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
marker_3_1.SetName('My_marker')
body_3.AddMarker(marker_3_1)
marker_3_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(1.41020567518852,0.0247718283814951,-1.62976499799182),chrono.ChQuaternionD(0.881036476647703,0.00951680497270276,0.472619742769445,0.0177407999779333)))

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0.996410323712302; mr[2,0]=0.0148275661003368 
mr[0,1]=-0.0833463261523019; mr[1,1]=0.0148275661003368; mr[2,1]=-0.996410323712302 
mr[0,2]=-0.993053389916914; mr[1,2]=-0.00123582316024348; mr[2,2]=0.0830471398216463 
body_3.GetCollisionModel().AddCylinder(0.038575,0.038575,0.0163750000000019,chrono.ChVectorD(-0.0652414174348167,-0.0773807616988142,0.179201155078968),mr)
body_3.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0490106583570424,-0.0813717222701078,0.130238444426058))
body_3.GetCollisionModel().AddSphere(0.00448981850569662, chrono.ChVectorD(-0.0783561237483722,-0.0580446528925914,0.133040222294353))
body_3.GetCollisionModel().AddSphere(0.00450000000000001, chrono.ChVectorD(-0.0817763692558815,-0.0926100044370168,0.13281194772533))
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)




# Mate constraint: Parallel22 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: graound-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: single_bot1-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.25,0.00999999999999999,-0.25)
dA = chrono.ChVectorD(0,1,2.88998133608509e-16)
cB = chrono.ChVectorD(1.41282194374937,0.0177718283814951,-1.59611830595801)
dB = chrono.ChVectorD(2.77555756156289e-17,-1,-4.16333634234434e-16)
link_1.SetFlipped(True)
link_1.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_1.SetName("Parallel22")
exported_items.append(link_1)


# Mate constraint: Concentric31 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: leg-1 ,  SW ref.type:2 (2)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079)
dA = chrono.ChVectorD(0.833128137394371,3.46944695195361e-15,0.553080018335309)
cB = chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079)
dB = chrono.ChVectorD(0.833128137394371,3.46944695195361e-15,0.553080018335309)
link_2.Initialize(body_3,body_1,False,cA,cB,dA,dB)
link_2.SetName("Concentric31")
exported_items.append(link_2)

link_3 = chrono.ChLinkMateGeneric()
link_3.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079)
cB = chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079)
dA = chrono.ChVectorD(0.833128137394371,3.46944695195361e-15,0.553080018335309)
dB = chrono.ChVectorD(0.833128137394371,3.46944695195361e-15,0.553080018335309)
link_3.Initialize(body_3,body_1,False,cA,cB,dA,dB)
link_3.SetName("Concentric31")
exported_items.append(link_3)


# Mate constraint: Coincident81 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: single_bot1-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: leg-1 ,  SW ref.type:2 (2)

link_4 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(1.40187574112063,0.0227708283814951,-1.63289311234786)
cB = chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079)
dA = chrono.ChVectorD(0.833128137394371,3.46944695195361e-15,0.553080018335309)
dB = chrono.ChVectorD(-0.833128137394371,-3.46944695195361e-15,-0.553080018335309)
link_4.Initialize(body_3,body_1,False,cA,cB,dB)
link_4.SetDistance(0)
link_4.SetName("Coincident81")
exported_items.append(link_4)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(1.40187574112063,0.0227708283814951,-1.63289311234786)
dA = chrono.ChVectorD(0.833128137394371,3.46944695195361e-15,0.553080018335309)
cB = chrono.ChVectorD(1.40298245423732,0.0247718283814951,-1.63456020175079)
dB = chrono.ChVectorD(-0.833128137394371,-3.46944695195361e-15,-0.553080018335309)
link_5.SetFlipped(True)
link_5.Initialize(body_3,body_1,False,cA,cB,dA,dB)
link_5.SetName("Coincident81")
exported_items.append(link_5)

