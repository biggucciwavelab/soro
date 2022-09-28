#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#!/usr/bin/env python

import os
import math
import time
import sys, getopt
import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
import csv
import logging
logger = logging.getLogger("two_node_study")

def set_G(my_system, exported_items, t):
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, t))


def _write_data_for_body_to_file(f, body):
    pos = body.GetPos()
    app_force = body.Get_Xforce()
    cont_force = body.GetContactForce()
    #spring_force = -spring.GetC_force()
    app_torque = body.Get_Xtorque()
    cont_torque = body.GetContactTorque()

    def write_chvec_to_file(f, vec):
        #f.write(',%f,%f,%f' % (vec.x, vec.y, vec.z))
        f.write(',%f,%f,%f' % (vec.z))

    write_chvec_to_file(f, pos)
    write_chvec_to_file(f, app_force)
    write_chvec_to_file(f, cont_force)
    write_chvec_to_file(f, app_torque)
    write_chvec_to_file(f, cont_torque)

def write_header_info_to_file(my_system, exported_items, f, body_names):
    f.write('Time')
    bodies=[]
    for name in body_names:
        for body in exported_items:
            if body.GetName() == name:
                bodies.append(body)
                break
    for body in bodies:
        f.write(
            #',Pos_x_{name},Pos_y_{name},Pos_z_{name},Fa_x_{name},Fa_y_{name},Fa_z_{name},Fc_x_{name},Fc_y_{name},Fc_z_{name},Ta_x_{name},Ta_y_{name},Ta_z_{name},Tc_x_{name},Tc_y_{name},Tc_z_{name}'.format(
            #',Pos_x_{name},Pos_y_{name},Pos_z_{name}'.format(name=body.GetName()
            ',Pos_z_{name}'.format(name=body.GetName()
            )
        )
    f.write('\n')

def dump_info_to_file(my_system, exported_items, t, f=None, body_names=[], timestep_for_file=0.01, m_timestep=0.001):
    if (t - m_timestep)//timestep_for_file == t//timestep_for_file:
        return
    bodies = []
    f.write('%f' % t)
    for name in body_names:
        for body in exported_items:
            if body.GetName() == name:
                bodies.append(body)
                break
    for body in bodies:
        _write_data_for_body_to_file(f, body)
    f.write('\n')

def add_spring_between_bodies(my_system, exported_items, body_name_1='node_2250-1', body_name_2='node_2250-2',
                              rest_length=0.049, R=0.1, K=50):
    body_A = None
    for item in exported_items:
        if item.GetName() == body_name_1:
            body_A = item
            break
    body_B = None
    for item in exported_items:
        if item.GetName() == body_name_2:
            body_B = item
            break
    spring = chrono.ChLinkSpring()
    # spring.Initialize(body_A, body_B, True, chrono.ChVectorD(-0.12,0,0.12), chrono.ChVectorD(0.12,0,-0.12)  , True)
    spring.Initialize(body_A, body_B, True, chrono.ChVectorD(0.0, 0, 0.0), chrono.ChVectorD(0.0, 0, 0.0), True)
    spring.Set_SpringRestLength(rest_length)
    #spring.Set_SpringR(R)
    spring.Set_SpringK(K)
    spring.SetName("Spring_%s_%s" % (body_name_1, body_name_2))
    my_system.AddLink(spring)
    exported_items.append(spring)

def run_two_node_study(
        m_filename="two_node_2250_study.py",
        m_timestep=0.0001,
        m_length=10.0,
        m_visualization="irrlicht",
        m_datapath="./data/",
        preprocess_funcs=[add_spring_between_bodies],
        preprocess_params=[{'body_name_1': 'node_2250-1', 'body_name_2': 'node_2250-2', 'R': 0.1}],
        iteration_postprocess_funcs=[set_G],
        iteration_postprocess_params=[{}],
        video_save_interval = 100,
        snapshots_dir='./snapshots/'
    ):
    """
    :param  m_filename: string
    :param  m_timestep: float
    :param  m_length: float
    :param  m_visualization: string
    :param  m_datapath: string
    :param  csv_filename: string
    :param  preprocess_funcs: list of functions
        Functions should have call syntax f(exported_items, **kwargs)
    :param  preprocess_params: list of dicts
        Should be of same length as preprocess_funcs.
        Each dict will be passed as kwargs to preprocess_funcs
    """
    if not os.path.isfile(m_filename):
        print("Error. Filename " + m_filename + " does not exist.")
        sys.exit(2)
    chrono.SetChronoDataPath(m_datapath)

    logger.info("  file to load is %s " % m_filename)
    logger.info("  timestep is %f" % m_timestep)
    logger.info("  length is %f" % m_length)
    logger.info("  data path for fonts etc.: %s" % m_datapath)

    # Remove the trailing .py and add / in case of file without ./
    m_absfilename = os.path.abspath(m_filename)
    m_modulename = os.path.splitext(m_absfilename)[0]
    logger.info ("Loading C::E scene...");
    # ---------------------------------------------------------------------
    #
    #  load the file generated by the SolidWorks CAD plugin
    #

    exported_items = chrono.ImportSolidWorksSystem(m_modulename)
    logger.info ("...loading done!");
    # Print exported items
    for my_item in exported_items:
        logger.info(my_item.GetName())

    # Add items to the physical system
    my_system = chrono.ChSystemNSC()
    for my_item in exported_items:
        my_system.Add(my_item)

    my_system.SetSolverTolerance(1e-9)
    solver = chrono.ChSolverBB();
    my_system.SetSolver(solver)
    solver.SetMaxIterations(300);
    # solver.EnableWarmStart(True);
    # my_system.SetTimestepperType(1) # Euler Implicit Projected
    chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.001)

    my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))
    """
    body_A = None
    for item in exported_items:
        if item.GetName() == 'node_2250-1':
            body_A = item
            break
    body_B = None
    for item in exported_items:
        if item.GetName() == 'node_2250-2':
            body_B = item
            break

    spring = chrono.ChLinkSpring()
    # spring.Initialize(body_A, body_B, True, chrono.ChVectorD(-0.12,0,0.12), chrono.ChVectorD(0.12,0,-0.12)  , True)
    spring.Initialize(body_A, body_B, True, chrono.ChVectorD(0.0, 0, 0.0), chrono.ChVectorD(0.0, 0, 0.0), True)
    spring.Set_SpringRestLength(0.05)
    spring.Set_SpringR(0.1)
    spring.Set_SpringK(50)
    spring.SetName("Spring2")
    my_system.AddLink(spring)
    """


    for func, kwargs in zip(preprocess_funcs, preprocess_params):
        func(my_system, exported_items, **kwargs)

    if m_visualization == "irrlicht":

        #  Create an Irrlicht application to visualize the system

        myapplication = chronoirr.ChIrrApp(my_system, 'Two Node Study', chronoirr.dimension2du(1280, 720))

        myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
        myapplication.AddTypicalCamera(chronoirr.vector3df(0, .25, 0), chronoirr.vector3df(0.0, 0.0, 0.0))
        myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
        myapplication.SetSymbolscale(0.002)
        myapplication.SetShowInfos(False)
        myapplication.AssetBindAll();
        myapplication.AssetUpdateAll();

        myapplication.SetTimestep(m_timestep);
        t = 0

        import time
        if video_save_interval is not None and isinstance(video_save_interval, int):
            if not os.path.exists(snapshots_dir):
                os.makedirs(snapshots_dir)
            os.chdir(snapshots_dir)
            myapplication.SetVideoframeSave(True)
            myapplication.SetVideoframeSaveInterval(video_save_interval)

        myapplication.BeginScene()
        myapplication.DrawAll()
        myapplication.DoStep()
        myapplication.EndScene()
        #time.sleep(1)
        while True:#myapplication.GetDevice().run():
            myapplication.BeginScene()
            myapplication.DrawAll()
            myapplication.DoStep()
            myapplication.EndScene()
            t += m_timestep
            #print(t)
            logger.debug(t)
            for func, kwargs in zip(iteration_postprocess_funcs, iteration_postprocess_params):
                func(my_system, exported_items, t, **kwargs)
            if (t >= m_length):
                myapplication.GetDevice().closeDevice()
                return
    else:
        raise ValueError("Unsupported Visualization engine")


if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:],"f:d:T:v:p:",["filename=","timestep=","Tlength=","visualization=","datapath="])
    except getopt.GetoptError:
        print ("%s -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]" % sys.argv[0])
        sys.exit(2)
    kwargs = {}
    for opt, arg in opts:
        print ("opt:", opt, "  arg", arg)
        if   opt in ("-d", "--timestep"):
            kwargs['m_timestep'] = float(arg)
        elif opt in ("-T", "--Tlength"):
            kwargs['m_length'] = float(arg)
        elif opt in ("-f", "--filename"):
            kwargs['m_filename ']= arg
        elif opt in ("-v", "--visualization"):
            kwargs['m_visualization'] = arg
        elif opt in ("-p", "--datapath"):
            kwargs['m_datapath'] = arg

    run_two_node_study(**kwargs)












