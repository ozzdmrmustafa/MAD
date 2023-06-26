import numpy as np
from TRIM_SIM.SIMULATION import Simulation

class Sim_startup:

    def Simulation_StartUp(dat,geo,Results,Final_time,Frequency):
        
        Result   = Results[0]      # Trim results at t=0
        Jacobian = Results[1]      # Trim results at t=0
        
        [*Rt] =[*Results[0].values()]
        k=[]
        for i in Results[0].keys(): 
            k.append(i)
        for i in range(0,len(Results[0])):
            exec("dat.trim_%s=%f" % (k[i],Rt[i]))
            exec("%s=%f" % (k[i],Rt[i]))
    
        Delta_time=1/Frequency #seconds
        
        State=[dat.Aoa,dat.Beta,dat.V_inf,dat.U,dat.V,dat.W,dat.P,dat.Q,\
                dat.R,dat.Phi,dat.Theta,dat.Psi]
        Input=[dat.HTL,dat.FlaperonL,dat.Rudder,dat.PLA]
        
        Simulation.simulation(dat,State,Input,Final_time,Delta_time,geo)
        Simulation.sim_plot(dat)