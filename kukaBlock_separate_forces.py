
import pybullet_utils.bullet_client as bc
import pybullet_data
import pybullet
import math
import pickle
import numpy as np
import matplotlib.pyplot as plt
import time


def main():
  start = time.time()
  p = bc.BulletClient(connection_mode=pybullet.DIRECT)
  p.setTimeStep(1/240.)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf", [0, 0, -0.05], useFixedBase=True)
  kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
  p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
  kukaEndEffectorIndex = 6
  numJoints = p.getNumJoints(kukaId)
  blockId = p.loadURDF("urdfs/block_big.urdf", [-0.4, 0, .1], useFixedBase=0)

  #restposes for null space
  rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
  #joint damping coefficents
  damping = 1e-1
  jd = [damping] * 7

  for i in range(numJoints):
    p.resetJointState(kukaId, i, rp[i])

  p.setPhysicsEngineParameter(constraintSolverType=p.CONSTRAINT_SOLVER_LCP_SI)

  p.setGravity(0, 0, -10)
  t = 0.
  speedup = 1
  prevPose = [0, 0, 0]
  prevPose1 = [0, 0, 0]
  hasPrevPose = 0
  useNullSpace = 0

  count = 0
  useOrientation = 1
  useSimulation = 1

  #trailDuration is duration (in seconds) after debug lines will be removed automatically
  #use 0 for no-removal
  trailDuration = 15

  simLength = 30000
  x = -0.4
  y = -0.2
  z = 0.01
  t = 0
  orn = p.getQuaternionFromEuler([0, -math.pi, 0])

  traj = np.zeros((simLength, 3))
  contactForceRobotBlock = np.zeros(simLength)
  jointInds = [i for i in range(numJoints)]
  penetrations = np.zeros(simLength)

  p.changeDynamics(blockId, linkIndex=-1, mass=0.7)

  for simTime in range(simLength):
    p.stepSimulation()

    # set end effector pose
    dx = 0.00
    dy = 5e-6 # 5e-5
    dz = 0.00
    x += dx
    y += dy
    z += dz
    eePos = [x, y, z]

    # compute the inverse kinematics
    jointPoses = p.calculateInverseKinematics(kukaId,
                                              kukaEndEffectorIndex,
                                              eePos,
                                              orn,
                                              jointDamping=jd)



    p.setJointMotorControlArray(bodyIndex=kukaId,
                              jointIndices=range(numJoints),
                              controlMode=p.POSITION_CONTROL,
                              targetPositions=jointPoses[:numJoints],
                              forces=[100]*numJoints,
                              positionGains=[0.3]*numJoints,
                              velocityGains=[1]*numJoints)

    # get information on contact between the block and the robot
    contactInfo = p.getContactPoints(kukaId, blockId)

    if len(contactInfo) > 0:
      contactForceRobotBlock[simTime] = sum([c[9] for c in contactInfo])
      penetrations[simTime] = max([c[8] for c in contactInfo])

  end = time.time()
  print("time elapsed: ", end - start)
  plt.plot(contactForceRobotBlock)
  plt.title("total contact force between robot and block")
  plt.savefig("sum_forces.png")
  plt.show()

  results = {
    "contact force": contactForceRobotBlock,
    "parameters": p.getDynamicsInfo(blockId, -1)}

  # pickle.dump(results, open("results.pickle", "wb"))

if __name__ == '__main__':
  main()
