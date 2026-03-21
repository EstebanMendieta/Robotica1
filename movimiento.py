import numpy as np
import roboticstoolbox as rtb
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

L1 = 0.30
L2 = 0.25
d1 = 0.40

eslabon1 = rtb.RevoluteDH(d=d1, a=L1, alpha=0, qlim=[-np.pi, np.pi])
eslabon2 = rtb.RevoluteDH(d=0, a=L2, alpha=np.pi, qlim=[-np.pi, np.pi])
eslabon3 = rtb.PrismaticDH(theta=0, a=0, alpha=0, qlim=[0, 0.20])

scara = rtb.DHRobot([eslabon1, eslabon2, eslabon3], name="SCARA_Agricola")

q_home = np.array([0, 0, 0]) 
q_agarre = np.array([np.pi/4, -np.pi/6, 0.05]) 

pasos = 50
trayectoria = rtb.jtraj(q_home, q_agarre, pasos)

client = RemoteAPIClient()
sim = client.require('sim')

joint1 = sim.getObject('/Joint1')
joint2 = sim.getObject('/Joint2')
joint3 = sim.getObject('/Joint3')

print("Simulacion iniciada")
sim.startSimulation()
time.sleep(0.5)
sim.setInt32Signal('close_gripper', 1)

for q_actual in trayectoria.q:
    sim.setJointTargetPosition(joint1, q_actual[0])
    sim.setJointTargetPosition(joint2, q_actual[1])
    sim.setJointTargetPosition(joint3, q_actual[2])
    time.sleep(0.05)

time.sleep(0.5) 
sim.setInt32Signal('close_gripper', 0)
time.sleep(1.5) 
sim.stopSimulation()