import numpy as np

from ATMOSPHERE.ATM import Atm
from scipy.optimize import curve_fit


# Engine.engine

class Engine:

    def engine(dat,geo):
        
        Atm.atm(dat)
        
        dat.PLA=np.clip(dat.PLA,0,geo.Max_engine_hp) # maximum PLA level clip
    
        
        rho_0=1.2250004 #kg/m^3
    
        #Power_sealevel = 97 # sea level and standart conditions horsepower (continues)
        #PLA = between(0-100)
        Power_sealevel = dat.PLA # normally this parameter defines sea level available power but we use as if PLA

        sigma = dat.rho/rho_0;  #density ratio
        

    
        dat.Power_resultant=Power_sealevel*(1.132*sigma-0.132) #Gagg and Ferrar model #Resultant horse power
        
        Correlation_coeff=0.60 # Real dataya bakarak sonucu calibre et between 0.5 - 0.9

        Total_Disk_Area=np.pi*(geo.propeller_diameter/2)**2  # m^2
        Spinner_Disk_Area=np.pi*(geo.hub_diameter/2)**2  # m^2
        Static_Thrust=0.85*Correlation_coeff*(dat.Power_resultant*745.69987158)**(2/3)*(2*dat.rho*Total_Disk_Area)**(1/3)*(1-(Spinner_Disk_Area/Total_Disk_Area)) #N
        
        # Total_Disk_Area=np.pi*(geo.propeller_diameter/2)**2 *10.7639104  # ft^2
        # Spinner_Disk_Area=np.pi*(geo.hub_diameter/2)**2 * 10.7639104 # ft^2
        # Static_Thrust=0.85*Correlation_coeff*(dat.Power_resultant*550)**(2/3)*(2*dat.rho*0.00194*Total_Disk_Area)**(1/3)*(1-(Spinner_Disk_Area/Total_Disk_Area)) #Lbf
        # Static_Thrust=Static_Thrust*4.44822162 #N
        
    
        min_Thrust=(0.5*dat.Power_resultant**2*(geo.Propeller_efficiency*745.69987158)**2*dat.rho*geo.S_ref_w*geo.CD0)**(1/3)
        
        V_max=(min_Thrust*2/(geo.CD0*dat.rho*geo.S_ref_w))**0.5
        
        V_cruise=V_max*0.8
        
        cruise_Thrust=745.7*geo.Propeller_efficiency*dat.Power_resultant/V_cruise
        
        
        
        def func(V, A, B, C):
            return  A * V ** 2  + B * V  +  C
        
        V_arr=np.asarray([0,V_cruise,V_max])
        T_arr=np.asarray([Static_Thrust,cruise_Thrust,min_Thrust])

        (T_popt, _) = curve_fit(func, V_arr, T_arr)# bounds=(0, [3., 1., 0.5]))
        
        
        Actual_Thrust=func(dat.V_inf, *T_popt)
        # Actual_Thrust_arr=func(V_arr, *T_popt)
        
        
        # Actual_Thrust= dat.Power_resultant*geo.Propeller_efficiency*745.69987158/dat.V_inf             #Newton   V_inf sıfıra yaklaşırken ne olur ????????????
        
        # Actual_Thrust= Static_Thrust - (dat.V_inf/V_max)*(Static_Thrust-min_Thrust)      #Newton   V_inf sıfıra yaklaşırken ne olur ????????????

        
        dat.Thrust=np.min([Static_Thrust,Actual_Thrust])
    
        # print(dat.Thrust)
        # print(dat.PLA)
        
        # return [Power_resultant,Thrust]
