#!/usr/bin/env python
#
# Moves a robotic arm with a motion plan as defined by a given scene with the PySDK
#
# This script assumes the following: 1) that the given scene path file is accessible in the directory above, 2) that the
# number of joints on the connected robot arm matches the number of joints in the plan file, and 3) that the kinematic
# model of the connected arm matches the kinematic model used when solving the initial plan.

import argparse
import asyncio
import csv
import numpy
import os
import sys

from viam.components.arm import Arm, JointPositions
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

ARM_COMPONENT_NAME = "MANIPULATOR"

async def connect():
    creds = Credentials(type='robot-location-secret', payload='<YOUR PAYLOAD HERE>')
    opts = RobotClient.Options(refresh_interval=0, dial_options=DialOptions(credentials=creds))
    return await RobotClient.at_address('<YOUR ADDRESS HERE>', opts)

async def main():
    # Get the scene whose plan we want to use
    parser = argparse.ArgumentParser(description='Executes a motion plan on the connected arm using the PySDK')
    parser.add_argument('scene_name', type=str, help='Name of the scene plan file to parse and execute')
    args = parser.parse_args()

    # Parse the plan for the given scene name
    plan_csv = args.scene_name + ".csv"
    plan_filepath = os.path.join(os.path.abspath(os.path.curdir), "..", plan_csv)
    print(">>> Searching for plan path file [ {0} ]".format(plan_filepath))
    if os.path.isfile(plan_filepath):
        print(">>> Executing plan generated for [ {0} ] with [ {1} ] : Loading [ {2} ]...".format(
              args.scene_name, ARM_COMPONENT_NAME, plan_filepath))
    else:
        print("!!! Requested scene plan file not found... Exiting.")
        sys.exit(1)

    plan = []
    with open(plan_filepath, newline='') as csvfile:
        plan_reader = csv.reader(csvfile, delimiter=',')
        for row in plan_reader:
            joints = JointPositions(values=[numpy.rad2deg(float(value)) for value in row])
            plan.append(joints)

    # Connect to the robot arm 
    robot = await connect()
    arm = Arm.from_robot(robot, ARM_COMPONENT_NAME)

    # Execute the plan
    print("??? Moving robot arm to each joint position in motion plan...")
    for target in plan:
        await arm.move_to_joint_positions(target)
    print(">>> Arm motion has completed, returning to zero position...")

    # Set a JointPositions struct full of zeroes with the same length as a state in the plan
    await arm.move_to_joint_positions(JointPositions(values=[0 for idx in range(len(plan[0].values))]))

    await robot.close()


if __name__ == '__main__':
    asyncio.run(main())
