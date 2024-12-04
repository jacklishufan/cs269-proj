"""Gripper interaction demo.

This script illustrates the process of importing grippers into a scene and making it interact
with the objects with actuators. It also shows how to procedurally generate a scene with the
APIs of the MJCF utility functions.

Example:
    $ python run_gripper_test.py
"""

import xml.etree.ElementTree as ET

from robosuite.models import MujocoWorldBase
from robosuite.models.arenas.table_arena import TableArena
from robosuite.models.grippers import PandaGripper, RethinkGripper,Robotiq85Gripper
from robosuite.models.robots import UR5e,Panda
from robosuite.models.objects import BoxObject
from robosuite.utils import OpenCVRenderer
from robosuite.utils.binding_utils import MjRenderContextOffscreen, MjSim
from robosuite.utils.mjcf_utils import new_actuator, new_joint
from robosuite.models.grippers import gripper_factory
from robosuite.renderers.mjviewer.mjviewer_renderer import MjviewerRenderer



if __name__ == "__main__":

    # start with an empty world
    world = MujocoWorldBase()

    # add a table
    arena = TableArena(table_full_size=(1,1, 0.05), table_offset=(0, 0, 1.1), has_legs=False,xml='arenas/table_arena_large.xml')
    world.merge(arena)

    # add a gripper
    mujoco_robot = Panda()
    # gripper = Robotiq85Gripper()
    
    front_view_camera =  world.worldbody.findall(".//camera[@name='frontview']")[0]

    front_view_camera.set("pos", "2.0 0 1.8")
    
    gripper =gripper_factory('PandaGripper')
    mujoco_robot.add_gripper(gripper)
    mujoco_robot.set_base_xpos([-1, 0, 1])
    world.merge(mujoco_robot)
    breakpoint()
    #world.actuator.append(new_actuator(joint="gripper_z_joint", act_type="position", name="gripper_z", kp="500"))
    # Create another body with a slider joint to which we'll add this gripper
    # gripper_body = ET.Element("body", name="gripper_base")
    # gripper_body.set("pos", "0 0 1.3")
    # gripper_body.set("quat", "0 0 1 0")  # flip z
    # gripper_body.append(new_joint(name="gripper_z_joint", type="slide", axis="0 0 1", damping="50"))
    # Add the dummy body with the joint to the global worldbody
    #world.worldbody.append(gripper_body)
    # world.worldbody.append(robot)
    # breakpoint()
    # Merge the actual gripper as a child of the dummy body
    #world.merge(gripper, merge_body="gripper_base")
    # world.merge(robot, merge_body="Robotiq85Gripper")
    # Create a new actuator to control our slider joint
    # world.actuator.append(new_actuator(joint="gripper_z_joint", act_type="position", name="gripper_z", kp="500"))

    # add an object for grasping
    mujoco_object = BoxObject(
        name="box", size=[0.02, 0.02, 0.02], rgba=[1, 0, 0, 1], friction=[1, 0.005, 0.0001]
    ).get_obj()
    # Set the position of this object
    mujoco_object.set("pos", "0 0 1.11")
    # Add our object to the world body
    world.worldbody.append(mujoco_object)
    # breakpoint()
    

    book1 = BoxObject(
        name="book1", size=[0.03, 0.05, 0.01], rgba=[1, 0, 0, 1], friction=[1, 0.005, 0.0001]
    ).get_obj()
    book1.set("pos", "0 0.1 1.11")
    book1.set("quat", "0.707 0 0.707 0")  # Rotation example
    world.worldbody.append(book1)

    book2 = BoxObject(
        name="book2", size=[0.04, 0.06, 0.015], rgba=[0, 1, 0, 1], friction=[1, 0.005, 0.0001]
    ).get_obj()
    book2.set("pos", "-0.1 0.2 1.12")
    book2.set("quat", "0.5 0.5 0.5 0.5")  # Another rotation example
    world.worldbody.append(book2)

    book3 = BoxObject(
        name="book3", size=[0.05, 0.04, 0.02], rgba=[0, 0, 1, 1], friction=[1, 0.005, 0.0001]
    ).get_obj()
    book3.set("pos", "0.1 -0.1 1.13")
    book3.set("quat", "0 0 0.707 0.707")  # Yet another rotation example
    world.worldbody.append(book3)



    # Add our object to the world body
    # world.worldbody.append(mujoco_object2)

    # add reference objects for x and y axes
    x_ref = BoxObject(
        name="x_ref", size=[0.01, 0.01, 0.01], rgba=[0, 1, 0, 1], obj_type="visual", joints=None
    ).get_obj()
    x_ref.set("pos", "0.2 0 1.105")
    world.worldbody.append(x_ref)
    y_ref = BoxObject(
        name="y_ref", size=[0.01, 0.01, 0.01], rgba=[0, 0, 1, 1], obj_type="visual", joints=None
    ).get_obj()
    y_ref.set("pos", "0 0.2 1.105")
    world.worldbody.append(y_ref)

    # start simulation
    model = world.get_model(mode="mujoco")

    sim = MjSim(model)
   
    viewer = OpenCVRenderer(sim,h=512,w=512)
    #viewer = MjviewerRenderer(sim,camera_id=0)
    # viewer =MjviewerRenderer(sim)
    # viewer.set_camera(1)
    # breakpoint()
    render_context = MjRenderContextOffscreen(sim, device_id=0)
    sim.add_render_context(render_context)

    sim_state = sim.get_state()

    # # for gravity correction
    # gravity_corrected = ["gripper_z_joint"]
    # _ref_joint_vel_indexes = [sim.model.get_joint_qvel_addr(x) for x in gravity_corrected]

    # # Set gripper parameters
    # gripper_z_id = sim.model.actuator_name2id("gripper_z")
    # gripper_z_low = 0.07
    # gripper_z_high = -0.02
    # gripper_z_is_low = False

    # gripper_jaw_ids = [sim.model.actuator_name2id(x) for x in gripper.actuators]
    # gripper_open = [-0.0115, 0.0115]
    # gripper_closed = [0.020833, -0.020833]
    # gripper_is_closed = True

    # hardcode sequence for gripper looping trajectory
    seq = [(False, False), (True, False), (True, True), (False, True)]

    sim.set_state(sim_state)
    step = 0
    T = 500
    while True:
        if step % 100 == 0:
            print("step: {}".format(step))

            # Get contact information
            for contact in sim.data.contact[0 : sim.data.ncon]:

                geom_name1 = sim.model.geom_id2name(contact.geom1)
                geom_name2 = sim.model.geom_id2name(contact.geom2)
                if geom_name1 == "floor" and geom_name2 == "floor":
                    continue

                print("geom1: {}, geom2: {}".format(geom_name1, geom_name2))
                print("contact id {}".format(id(contact)))
                print("friction: {}".format(contact.friction))
                print("normal: {}".format(contact.frame[0:3]))

        # Iterate through gripping trajectory
        # if step % T == 0:
        #     plan = seq[int(step / T) % len(seq)]
        #     gripper_z_is_low, gripper_is_closed = plan
        #    print("changing plan: gripper low: {}, gripper closed {}".format(gripper_z_is_low, gripper_is_closed))

        # Control gripper
        # if gripper_z_is_low:
        #     sim.data.ctrl[gripper_z_id] = gripper_z_low
        # else:
        #     sim.data.ctrl[gripper_z_id] = gripper_z_high
        # if gripper_is_closed:
        #     sim.data.ctrl[gripper_jaw_ids] = gripper_closed
        # else:
        #     sim.data.ctrl[gripper_jaw_ids] = gripper_open

        # Step through sim
        sim.step()
        #sim.data.qfrc_applied[_ref_joint_vel_indexes] = sim.data.qfrc_bias[_ref_joint_vel_indexes]
        viewer.render()
        step += 1
