import numpy as np


# Atm.atm

class Atm:	 
    def atm(dat):
        
         #######ATMOSPHERÄ°C MODEL###############
         #British unit temperature=Kelvin and density=slug/ft^3#
    
         #up to 36,089 feet altitude
         
         rho_std=1.225*0.00194
         B=0
         A=288.15
         Altitude_ft=dat.Altitude*3.28
    
         if Altitude_ft < 36089.2:
            A=288.15
            B=-1.9812*(10)**(-3)
            I=0.24179285
            J=-1.6624675*(10)**(-6)
            L=4.2558797
            rho_std=(I+J*Altitude_ft)**L
        
         #between from 36,0892 to 65,6168 feet altitude
         elif Altitude_ft < 65616.8:
            A=216.65
            B=0
            M=4.0012122*(10)**(-3)
            N=-48.063462*(10)**(-6)
            rho_std=(M)*np.exp(N*Altitude_ft)
                           
         T_std=A+B*Altitude_ft
         T_resultant=T_std+dat.DeltaISA   #Kelvin
         rho_resultant=rho_std/(1+(dat.DeltaISA)/T_std)   #slug/ft^3
        
         V_sound=(1.4*287.0529*T_resultant)**.5
         
         dat.Mach= dat.V_inf/V_sound #m/s
         dat.rho=rho_resultant*515.379 #kg/m^3
         dat.temperature=T_resultant #Kelvin
         # return [rho,Mach]