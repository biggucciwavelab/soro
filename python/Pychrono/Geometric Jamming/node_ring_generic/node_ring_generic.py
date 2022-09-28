#general code for comprssion testing of rings of regular 2D nodes (jammed or unjammed)

import pychrono as chrono 
import builtins
import numpy as np
import sys
import pychrono.irrlicht as chronoirr
import time
import os

def main(
    r_i=0.025,
    h=0.01,
    N_s=8,
    N_n=10,
    body_friction=1.2,
    ground_friction=0.1,
    block_friction=0.1,
    r=0.1,
    k=100,
    jammed=True,
    m_timestep=0.001,
    t_delay=0.05,
    csv_time_interval = 10.0,
    strain_rate = 0.0,#0.1,
    strain_max = -0.2,   # Since we are doing compression, sim ends after strain is below this value
):

    r_o = r_i / np.cos(np.pi / N_s)
    R_o = r_o / np.sin(np.pi / N_n)
    x_o = 0.049 if(jammed) else 2*r_o
    cam_y = 5*R_o

    #m_length = 15.0
    #m_length = (strain_max*2*R_o)/strain_rate +2*t_delay
    video_time_interval = int(1.0 / m_timestep / 240.0)  # create video at 20 fps
    m_visualization = "irrlicht"

    polygon_texture = chrono.ChTexture()
    polygon_texture.SetTextureFilename('.\data\octB.png')
    ground_texture = chrono.ChTexture()
    ground_texture.SetTextureFilename('.\data\ground.png')
    block_texture = chrono.ChTexture()
    block_texture.SetTextureFilename('.\data\octA.png')

    bodies = get_ring(N_s=N_s, r_i=r_i, h=h, density=2414.3, N_n=N_n, texture=polygon_texture)
    #springs = get_ring_springs(bodies, x_o, r, k)
    ground = get_ground(6*R_o, 2*r_i, 6*R_o, 10000, ground_texture)
    block_fixed = get_block_fixed(R_o, 3*R_o, h, r_o, 10000, block_texture)
    block_moving = get_block_moving(R_o, 3*R_o, h, r_o, 10000, block_texture)

    #d = {'k': 1}
    #d['k']
    #d['a'] = 2
    #forces[bodyA][bodyB]
    #forces = {}
    #forces[bodyA] = {}
    #forces[bodyA][bodyB]
    force_funcs = {}
    for (bodyA, bodyB) in zip(bodies, bodies[1:] + [bodies[0]]):
        funcs_AB = [chrono.ChFunction_Const(0.0), chrono.ChFunction_Const(0.0), chrono.ChFunction_Const(0.0)]
        funcs_BA = [chrono.ChFunction_Const(0.0), chrono.ChFunction_Const(0.0), chrono.ChFunction_Const(0.0)]
        force_funcs.setdefault(bodyA.GetName(), {})[bodyB.GetName()] = funcs_AB
        force_funcs.setdefault(bodyB.GetName(), {})[bodyA.GetName()] = funcs_BA
        force_A = chrono.ChForce()
        force_A.SetF_x(funcs_AB[0])
        force_A.SetF_y(funcs_AB[1])
        force_A.SetF_z(funcs_AB[2])
        bodyA.AddForce(force_A)
        force_B = chrono.ChForce()
        force_B.SetF_x(funcs_BA[0])
        force_B.SetF_y(funcs_BA[1])
        force_B.SetF_z(funcs_BA[2])
        bodyB.AddForce(force_B)

    #print(force_funcs['Body_0'])
    #constrain moving block to be parallel to fixed block (redundant bc I define moving block position at each time step later)
    link = chrono.ChLinkMateParallel()
    cA = chrono.ChVectorD(0, 0, 0)
    dA = chrono.ChVectorD(1, 0, 0)
    cB = chrono.ChVectorD(0, 0, 0)
    dB = chrono.ChVectorD(1, 0, 0)
    link.Initialize(block_moving, block_fixed, False, cA, cB, dA, dB)

    for body in bodies:
        body.GetMaterialSurfaceNSC().SetFriction(body_friction)
        body.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
    ground.GetMaterialSurfaceNSC().SetFriction(ground_friction)
    ground.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
    block_fixed.GetMaterialSurfaceNSC().SetFriction(block_friction)
    block_fixed.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
    block_moving.GetMaterialSurfaceNSC().SetFriction(block_friction)
    block_moving.GetMaterialSurfaceNSC().SetRollingFriction(0.0)

    my_system = chrono.ChSystemNSC()
    for body in ([block_fixed, block_moving, link, ground] + bodies
                # + springs
        ):
        my_system.Add(body)
    my_system.SetSolverTolerance(1e-9)
    solver = chrono.ChSolverBB()
    #solver = chrono.ChSolverPSOR()
    my_system.SetSolver(solver)
    solver.SetMaxIterations(300)
    chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.01)
    chrono.ChCollisionModel_SetDefaultSuggestedMargin(0.01)
    my_system.SetStep(m_timestep)
    cwd = os.getcwd()

    j = 'jammed' if(jammed) else 'unjammed'
    #csv_filename = 'results/Nsides_%d_Nnodes_%d_%s_%i.csv' %(N_s,N_n,j,int(time.time()))
    csv_filename = 'results/Nsides_%d_Nnodes_%d_strain_%f_rate_%f_%i.csv' % (N_s, N_n, strain_max, strain_rate, int(time.time()))
    if not os.path.exists('results/'):
        os.mkdir('results/')
    f = open(csv_filename, 'w')
    snapshots_dir = csv_filename.replace('.csv', '_snapshots')

    if m_visualization == "irrlicht":
        #  Create an Irrlicht application to visualize the system
        myapplication = chronoirr.ChIrrApp(my_system, 'Ring of %d Polygons with %d Sides' %(N_s,N_n), chronoirr.dimension2du(1280, 720))
        myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
        myapplication.AddTypicalCamera(chronoirr.vector3df(0, cam_y, 0), chronoirr.vector3df(0.0, 0.0, 0.0))
        myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
        myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, -2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
        myapplication.SetSymbolscale(0.05)
        myapplication.SetShowInfos(False)
        #myapplication.SetContactsDrawMode(2)
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

        ####################################################################################
        # video capture code
        # uncomment next line to activate, but if timestep different than Irrlicht default (0.01) simulation is unstable
        #myapplication.SetTimeStep(m_timestep)
        if video_time_interval is not None and isinstance(video_time_interval, int):
            if not os.path.exists(snapshots_dir):
                os.makedirs(snapshots_dir)
            os.chdir(snapshots_dir)
            myapplication.SetVideoframeSave(True)
            myapplication.SetVideoframeSaveInterval(video_time_interval)
        #####################################################################################

        t = 0
        i = 0
        z_max = 0.0
        z_min = 0.0
        z_diff = 0.0
        z_stop = 0.0
        x_0 = block_moving.GetPos().x
        y_0 = block_moving.GetPos().y
        z_0 = 0.0
        #t_strain = m_length
        strain_vel = strain_rate*R_o
        
        export_int = 1 # Export data every export_int-th timestep
        
        # Write the .csv file header
        print("time, z_pos, strain, force moving, force fixed", file=f)

        # CHANGE
        #frc = chrono.ChForce()
        #my_const_func = chrono.ChFunction_Const(0.0)
        #frc.SetF_z(my_const_func)
        #block_moving.AddForce(frc)
        while (myapplication.GetDevice().run()):
            myapplication.BeginScene()
            myapplication.DrawAll()
            strain = (block_fixed.GetPos().z-block_moving.GetPos().z-0.5*r_o-(2*R_o+2*r_o))/(2*(R_o+r_o))
            
            # after the ring jams (t_delay) move blocks to be in contact with ring
            
            # Where's the actual command to jam the springs??? - Qiyuan
            
            # if (t > t_delay - m_timestep) and (t <= t_delay):
            #     for body in bodies:
            #         if body.GetPos().z > z_max:
            #             z_max = body.GetPos().z
            #         if body.GetPos().z < z_min:
            #             z_min = body.GetPos().z
            #     z_0 = z_min - 1.5 * r_o
            #     z_diff = z_max - z_min + 2*r_o;
            #     t_strain = (strain_max * z_diff) / strain_rate#+ t_delay - m_timestep
            #     print(z_0)
            #     print(z_diff)
            #     print(t_strain)
            # compress ring by moving left block at constant rate

            #CHANGE
            '''
            if (t > t_delay):
                block_moving.SetPos_dt(chrono.ChVectorD(0,0,strain_vel))
                z_stop = block_moving.GetPos().z
            if (strain<=strain_max):
                block_moving.SetPos_dt(chrono.ChVectorD(0,0,0))
                myapplication.GetDevice().closeDevice()
            '''
            #if (t > t_delay):
                #my_const_func.Set_yconst((t-t_delay)**2)
            #force_funcs['Body_0']['Body_1'][2].Set_yconst(t**2)
            #force_funcs['Body_0']['Body_3'][2].Set_yconst(-(t ** 2))

            update_spring_forces(bodies, force_funcs, x_o, k, r)
            # outputs
            moving_app_force = block_moving.Get_Xforce()
            moving_contact_force = block_moving.GetContactForce()
            fixed_contact_force = block_fixed.GetContactForce()
            if i%export_int==0 and strain<=0:
                print(t, block_moving.GetPos().z, strain, moving_app_force.z)
                print("%f,%f,%f,%f,%f" % (t, block_moving.GetPos().z, strain,
                                             -1*moving_contact_force.z, fixed_contact_force.z), file=f)
            
            myapplication.DoStep()
            myapplication.EndScene()
            t += m_timestep
            i += 1
    f.close()
    # change back to original directory so relative paths are correct for next run
    os.chdir(cwd)
    time.sleep(1)

def update_spring_forces(bodies, force_funcs, x_o=0.05, k=100.0, r=0.2):
    for bodyA, bodyB in zip(bodies, bodies[1:] + [bodies[0]]):
        posA = bodyA.GetPos()
        posB = bodyB.GetPos()
        velA = bodyA.GetPos_dt()
        velB = bodyB.GetPos_dt()
        funcs_AB = force_funcs[bodyA.GetName()][bodyB.GetName()]
        funcs_BA = force_funcs[bodyB.GetName()][bodyA.GetName()]

        dist = (posA-posB).Length()
        u_BA = (posA-posB)/dist
        funcs_AB[0].Set_yconst(-(k * (dist-x_o) + r*(velA-velB).Dot(u_BA)) * u_BA.x)
        funcs_AB[1].Set_yconst(-(k * (dist-x_o) + r*(velA-velB).Dot(u_BA)) * u_BA.y)
        funcs_AB[2].Set_yconst(-(k * (dist-x_o) + r*(velA-velB).Dot(u_BA)) * u_BA.z)
        funcs_BA[0].Set_yconst((k * (dist-x_o) + r*(velA-velB).Dot(u_BA)) * u_BA.x)
        funcs_BA[1].Set_yconst((k * (dist-x_o) + r*(velA-velB).Dot(u_BA)) * u_BA.y)
        funcs_BA[2].Set_yconst((k * (dist-x_o) + r*(velA-velB).Dot(u_BA)) *u_BA.z)



def get_polygon_body(N_s=8, r_i=0.025, h=0.01, density=2414.3, texture=None):
    pt_vect = chrono.vector_ChVectorD()
    r_o = r_i/np.cos(np.pi/N_s)
    eta = np.random.rand() * 2*np.pi/N_s
    for i in range(N_s):
        pt_vect.push_back(chrono.ChVectorD(
            r_o*np.cos(eta + i*2*np.pi/N_s),
            0,
            r_o*np.sin(eta + i*2*np.pi/N_s)
        ))
        pt_vect.push_back(chrono.ChVectorD(
            r_o * np.cos(eta + i*2*np.pi / N_s),
            h,
            r_o * np.sin(eta + i*2*np.pi / N_s)
        ))
    body = chrono.ChBodyEasyConvexHull(pt_vect, density, True, True)
    if texture is not None:
        body.AddAsset(texture)
    return body

def get_ring(N_s=8, r_i=0.025, h=0.01, density=2414.3, N_n=8, texture=None):
    ring_bodies = []
    r_o = r_i/np.cos(np.pi/N_s)
    R_o = r_o/np.sin(np.pi/N_n)
    eta = np.random.rand() * 2*np.pi/N_n
    for i in range(N_n):
        body = get_polygon_body(N_s, r_i, h, density, texture)
        body.SetPos(chrono.ChVectorD(R_o*np.cos(eta + i*2*np.pi/N_n), h/2, R_o*np.sin(eta + i*2*np.pi/N_n)))
        body.SetName('Body_%d' % i)
        ring_bodies.append(body)
    return ring_bodies

def get_ring_springs(bodies, d=0.049, r=0.1, k=100):
    springs = []
    for body_A, body_B in zip(bodies, (bodies[1:] + [bodies[0]])):
        spring = chrono.ChLinkSpring()
        spring.Initialize(body_A, body_B, True, chrono.ChVectorD(0.0, 0, 0.0), chrono.ChVectorD(0.0, 0, 0.0), True)
        spring.Set_SpringRestLength(d)
        spring.Set_SpringR(r)
        spring.Set_SpringK(k)
        springs.append(spring)
    return springs

def get_ground(x_s, y_s, z_s, density=2000, texture=None):
    ground = chrono.ChBodyEasyBox(x_s, y_s, z_s, density, True, True)
    ground.SetPos(chrono.ChVectorD(0, -y_s/2, 0))
    ground.SetBodyFixed(True)
    if texture is not None:
        ground.AddAsset(texture)
    return ground

def get_block_fixed(R_o, x_s, y_s, z_s, density=7000, texture=None):
    block_fixed = chrono.ChBodyEasyBox(x_s, y_s, z_s, density, True, True)
    block_fixed.SetPos(chrono.ChVectorD(0, y_s/2, R_o+1.5*z_s))
    block_fixed.SetBodyFixed(True)
    if texture is not None:
        block_fixed.AddAsset(texture)
    return block_fixed

def get_block_moving(R_o, x_s, y_s, z_s, density=7000, texture=None):
    block_moving = chrono.ChBodyEasyBox(x_s, y_s, z_s, density, True, True)
    block_moving.SetPos(chrono.ChVectorD(0, y_s/2, -(R_o+1.5*z_s)))
    block_moving.SetBodyFixed(False)
    if texture is not None:
        block_moving.AddAsset(texture)
    return block_moving

if __name__ == '__main__':
    main()

sys.exit()
