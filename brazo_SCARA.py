import time
import numpy as np
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

L1 = 0.4      
L2 = 0.3      
D1 = 0.2      
L_PINZA = 0.15

def generar_perfil_cubico(q_ini, q_fin, t_total, dt):
    pasos = int(t_total / dt)
    time_vector = np.linspace(0, t_total, pasos)
    traj_data = []
    
    for i in range(3): 
        q0 = q_ini[i]
        qf = q_fin[i]
        a0 = q0
        a2 = 3 * (qf - q0) / (t_total**2)
        a3 = -2 * (qf - q0) / (t_total**3)
        q_t = a0 + a2 * time_vector**2 + a3 * time_vector**3
        traj_data.append(q_t)
        
    return np.array(traj_data).T

def main():
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    
    try:
        h_j1 = sim.getObject('/Joint1')
        h_j2 = sim.getObject('/Joint2')
        h_j3 = sim.getObject('/Joint3')
    except Exception as e:
        print(f"Error: No encuentro los joints. {e}")
        return

    q_home       = [0, 0, 0]
    q_over_pick  = [math.radians(45), math.radians(30), 0.0] 
    q_pick       = [math.radians(45), math.radians(30), 0.18] 
    q_over_place = [math.radians(-45), math.radians(-60), 0.0]
    q_place      = [math.radians(-45), math.radians(-60), 0.18]

    secuencia = [
        (q_over_pick, 2.0, 0), 
        (q_pick,      1.5, 0), 
        (q_pick,      0.5, 1), 
        (q_over_pick, 1.0, 1), 
        (q_over_place,3.0, 1), 
        (q_place,     1.5, 1), 
        (q_place,     0.5, 0), 
        (q_over_place,1.0, 0),
        (q_home,      2.0, 0)  
    ]

    sim.startSimulation()
    print("Ciclo iniciado")
    
    dt = 0.05 
    pos_actual = q_home 

    for i, (q_objetivo, duracion, estado_pinza) in enumerate(secuencia):
        trayectoria = generar_perfil_cubico(pos_actual, q_objetivo, duracion, dt)
        
        sim.setInt32Signal('close_gripper', estado_pinza)

        for punto in trayectoria:
            sim.setJointTargetPosition(h_j1, punto[0])
            sim.setJointTargetPosition(h_j2, punto[1])
            sim.setJointTargetPosition(h_j3, punto[2])
            time.sleep(dt) 
            
        sim.setInt32Signal('close_gripper', estado_pinza)
        
        if np.allclose(pos_actual, q_objetivo):
            time.sleep(duracion) 

        pos_actual = q_objetivo 

    print("Ciclo terminado.")
    time.sleep(1)
    sim.stopSimulation()

if __name__ == "__main__":
    main()