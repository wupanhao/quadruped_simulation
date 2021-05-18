import pybullet as p
import time
import numpy as np

maxForceId = None
jointIds = []
quadruped = None
jointStateIds = []


def load_robot():
    global maxForceId
    global quadruped
    p.connect(p.GUI)
    plane = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1./100)
    # p.setDefaultContactERP(0)
    # urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
    urdfFlags = p.URDF_USE_SELF_COLLISION
    quadruped = p.loadURDF(
        "a1/urdf/a1.urdf", [0, 0, 0.48], [0, 0, 0, 1], flags=urdfFlags, useFixedBase=False)

    # enable collision between lower legs
    for j in range(p.getNumJoints(quadruped)):
        print(p.getJointInfo(quadruped, j))

    lower_legs = [2, 5, 8, 11]
    for l0 in lower_legs:
        for l1 in lower_legs:
            if (l1 > l0):
                enableCollision = 1
                print("collision for pair", l0, l1, p.getJointInfo(quadruped, l0)[
                    12], p.getJointInfo(quadruped, l1)[12], "enabled=", enableCollision)
                p.setCollisionFilterPair(
                    quadruped, quadruped, 2, 5, enableCollision)

    maxForceId = p.addUserDebugParameter("maxForce", 0, 100, 20)

    for j in range(p.getNumJoints(quadruped)):
        p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(quadruped, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
            jointIds.append(j)
            if len(jointIds) % 3 == 0:
                jointStateIds.append(p.addUserDebugParameter(
                    jointName.decode(), -np.pi + np.pi/4, -np.pi/4, -np.pi/2))
            elif len(jointIds) % 3 == 2:
                jointStateIds.append(p.addUserDebugParameter(
                    jointName.decode(), -np.pi/2, np.pi/2, np.pi/4))
            else:
                jointStateIds.append(p.addUserDebugParameter(
                    jointName.decode(), -np.pi/2, np.pi/2, 0))
    p.getCameraImage(480, 320)
    p.setRealTimeSimulation(0)


def set_actuator_postions(joint_states):
    maxForce = p.readUserDebugParameter(maxForceId)
    for leg_index in range(4):
        for axis_index in range(3):
            if axis_index == 2:
                targetPos = joint_states[axis_index][leg_index] - \
                    joint_states[axis_index-1][leg_index]
            else:
                targetPos = joint_states[axis_index][leg_index]
            i = leg_index*3+axis_index
            p.setJointMotorControl2(
                quadruped, jointIds[i], p.POSITION_CONTROL, targetPos, force=maxForce)
    p.stepSimulation()


def read_parameter():
    maxForce = p.readUserDebugParameter(maxForceId)
    for j in range(12):
        targetPos = p.readUserDebugParameter(jointStateIds[j])
        p.setJointMotorControl2(
            quadruped, jointIds[j], p.POSITION_CONTROL, targetPos, force=maxForce)
    p.stepSimulation()
    time.sleep(1./500)


if __name__ == '__main__':
    load_robot()
    while True:
        read_parameter()
