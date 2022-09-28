#general code for comprssion testing of rings of regular 2D nodes (jammed or unjammed)

import pychrono as chrono 
import builtins
import numpy as np
import sys
import pychrono.irrlicht as chronoirr
import time
import os

def main():

    m_timestep = 0.0001
    #t_delay = 0.05
    m_length = 7.0
    video_time_interval = int(1.0 / m_timestep / 120.0)  # create video at 20 fps
    m_visualization = "irrlicht"

    angs = [45.00]#, 45.00]#22.50, 33.75, 45.00, 56.25, 67.50]
    ks = [200, 100, 50] #100, 200]
    mus = [4.0, 8.0, 12.0]

    r_i = 0.025
    h=0.01
    r_o = r_i / np.cos(np.pi / 8)
    R_o = r_o / np.sin(np.pi / 8)
    cam_y = 5 * R_o
    fixed_texture = chrono.ChTexture()
    fixed_texture.SetTextureFilename('.\data\octA.png')
    ground_texture = chrono.ChTexture()
    ground_texture.SetTextureFilename('.\data\ground.png')
    moving_texture = chrono.ChTexture()
    moving_texture.SetTextureFilename('.\data\octB.png')

    for mu in mus:
        for k in ks:
            for ang in angs:
                my_system = chrono.ChSystemNSC()

                ground = chrono.ChBodyEasyBox(6 * R_o, 2 * r_i, 6 * R_o, 10000, True, True)
                ground.SetPos(chrono.ChVectorD(0, -r_i, 0))
                ground.SetBodyFixed(True)
                ground.AddAsset(ground_texture)
                ground.GetMaterialSurfaceNSC().SetFriction(0.05)
                ground.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
                my_system.Add(ground)
                #print('grounded')

                fixed_body = get_polygon_body(ang)
                fixed_body.SetPos(chrono.ChVectorD(0.0, h/2, 0.0))
                fixed_body.SetBodyFixed(True)
                fixed_body.AddAsset(fixed_texture)
                fixed_body.GetMaterialSurfaceNSC().SetFriction(mu/10.0)
                fixed_body.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
                my_system.Add(fixed_body)
                #print('made fixed')


                moving_body = get_polygon_body(ang)
                moving_body.SetPos(chrono.ChVectorD(-2*r_i*np.sin(np.pi*ang/180), h/2, -2*r_i*np.cos(np.pi*ang/180)))
                moving_body.AddAsset(moving_texture)
                moving_body.GetMaterialSurfaceNSC().SetFriction(mu/10.0)
                moving_body.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
                my_system.Add(moving_body)
                #print('made moving')
                '''
                spring = chrono.ChLinkSpring()
                spring.Initialize(fixed_body, moving_body, True, chrono.ChVectorD(0.0, 0, 0.0),
                                  chrono.ChVectorD(0.0, 0, 0.0), True)
                spring.Set_SpringRestLength(0.049)
                #spring.Set_SpringR(0.1)
                spring.Set_SpringK(k)
                my_system.Add(spring)
                '''
                my_system.SetSolverTolerance(1e-9)
                solver = chrono.ChSolverBB()
                my_system.SetSolver(solver)
                solver.SetMaxIterations(3000)
                chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.001)
                my_system.SetStep(m_timestep)
                my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

                cwd = os.getcwd()

                csv_filename = 'results/NoSpring_Angle_%d_Spring_%d_Friction_%d.csv' % (ang, k, mu)
                f = open(csv_filename, 'w')

                if m_visualization == "irrlicht":
                    #  Create an Irrlicht application to visualize the system
                    myapplication = chronoirr.ChIrrApp(my_system, 'Two_Nodes',# % (N_s, N_n),
                                                       chronoirr.dimension2du(1280, 720))
                    myapplication.AddTypicalSky('./data/skybox/')
                    myapplication.AddTypicalCamera(chronoirr.vector3df(0, cam_y, 0), chronoirr.vector3df(0.0, 0.0, 0.0))
                    myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, 2),
                                                     chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
                    myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, -2),
                                                     chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
                    myapplication.SetSymbolscale(0.002)
                    myapplication.SetShowInfos(False)
                    # myapplication.SetContactsDrawMode(2)
                    myapplication.AssetBindAll()
                    myapplication.AssetUpdateAll()
                    myapplication.SetTimestep(m_timestep)

                    t = 0
                    i = 0
                    #print(moving_body.GetMass())
                    #print(spring.Get_SpringR())
                    print(mu)
                    print(k)
                    myapplication.BeginScene()
                    myapplication.DrawAll()
                    myapplication.DoStep()
                    myapplication.EndScene()
                    #time.sleep(1)
                    while myapplication.GetDevice().run():
                        my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, t))
                        fixed_contact_force = fixed_body.GetContactForce()
                        moving_contact_force = moving_body.GetContactForce()
                        ground_contact_force = ground.GetContactForce()
                        #applied_force = moving_body.GetAppliedForce()
                        if i % 50 == 0:
                            """
                            print(t,fixed_body.GetPos().x, fixed_body.GetPos().z,
                                                     moving_body.GetPos().x, moving_body.GetPos().z,
                                                     accumulated_force.x, accumulated_force.z,
                                                     contact_force.x,contact_force.z)
                            """
                            print("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" %(t,
                                                    moving_body.GetPos().x,moving_body.GetPos().y,moving_body.GetPos().z,
                                                    moving_contact_force.x,moving_contact_force.y,moving_contact_force.z,
                                                    fixed_contact_force.x,fixed_contact_force.y,fixed_contact_force.z,
                                                    ground_contact_force.x,ground_contact_force.y,ground_contact_force.z), file = f)
                                                    #applied_force.x,applied_force.z),file = f)
                        myapplication.BeginScene()
                        myapplication.DrawAll()
                        myapplication.DoStep()
                        myapplication.EndScene()
                        t += m_timestep
                        i += 1
                        if (t >= m_length):
                            #forces = chrono.ChForce()
                            #force_list = moving_body.forcelist()
                            #print(force_list)
                            print('here')
                            myapplication.GetDevice().closeDevice()
                            #return
                f.close()
                os.chdir(cwd)
                time.sleep(1)



def get_polygon_body(shape = 45.00):
    pt_vect = chrono.vector_ChVectorD()

    if shape == 67.50:#22.50:
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0.01, -0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0.0270598050073099, 0.01, -3.46944695195361E-18))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0.01, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545, 0.01, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-0.0270598050073098, 0.01, 0))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545, 0.01, -0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0, -0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0.0270598050073099, 0, -3.46944695195361E-18))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545, 0, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-0.0270598050073098, 0, 0))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545, 0, -0.0191341716182545))

    if shape == 56.25:#33.75:
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0.01, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0.01, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0.01, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0.01, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0.01, -0.00527910607256975))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0.01, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0.01, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0.01, -0.00527910607256975))
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0, -0.00527910607256975))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0, -0.00527910607256975))

    if shape == 45.00:
        pt_vect.push_back(chrono.ChVectorD(0.0103553390593274, 0.01, -0.025))
        pt_vect.push_back(chrono.ChVectorD(0.025, 0.01, -0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(0.025, 0.01, 0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(0.0103553390593274, 0.01, 0.025))
        pt_vect.push_back(chrono.ChVectorD(-0.0103553390593274, 0.01, 0.025))
        pt_vect.push_back(chrono.ChVectorD(-0.025, 0.01, 0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(-0.025, 0.01, -0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(-0.0103553390593274, 0.01, -0.025))
        pt_vect.push_back(chrono.ChVectorD(0.0103553390593274, 0, -0.025))
        pt_vect.push_back(chrono.ChVectorD(0.025, 0, -0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(0.025, 0, 0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(0.0103553390593274, 0, 0.025))
        pt_vect.push_back(chrono.ChVectorD(-0.0103553390593274, 0, 0.025))
        pt_vect.push_back(chrono.ChVectorD(-0.025, 0, 0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(-0.025, 0, -0.0103553390593274))
        pt_vect.push_back(chrono.ChVectorD(-0.0103553390593274, 0, -0.025))

    if shape == 33.75:#56.25:
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0.01, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0.01, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0.01, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0.01, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0.01, -0.00527910607256975))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0.01, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0.01, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0.01, -0.00527910607256975))
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0, 0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0, 0.00527910607256974))
        pt_vect.push_back(chrono.ChVectorD(-0.0265398584417511, 0, -0.00527910607256975))
        pt_vect.push_back(chrono.ChVectorD(-0.0150336221733761, 0, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0150336221733761, 0, -0.0224994055784104))
        pt_vect.push_back(chrono.ChVectorD(0.0265398584417511, 0, -0.00527910607256975))

    if shape == 22.50:#67.50:
        pt_vect.push_back(chrono.ChVectorD(-3.29597460435593E-17, 0.01, -0.0270598050073099))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0.01, -0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0.01, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0, 0.01, 0.0270598050073098))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182544, 0.01, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545, 0.01, -0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-3.29597460435593E-17, 0, -0.0270598050073099))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0, -0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0.0191341716182545, 0, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(0, 0, 0.0270598050073098))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182544, 0, 0.0191341716182545))
        pt_vect.push_back(chrono.ChVectorD(-0.0191341716182545, 0, -0.0191341716182545))


    body = chrono.ChBodyEasyConvexHull(pt_vect, 2500, True, True)
    return body

if __name__ == '__main__':
    main()

sys.exit()

