import numpy as np
import os
import shutil

import pandas as pd
from TRIM_SIM.DATA import data
from TRIM_SIM.T import TClass
from TRIM_SIM.SIMULATION import Simulation
from FCS_CONTROL.CONTROL import Control
from WEIGHT_BALANCE.WEIGHTMODULE import WeightModule
from GEO_DEF.GEO import geom
from SIM_STARTUP import Sim_startup
from matplotlib import pyplot as plt
import time

class Performance:
    
    current_path = os.getcwd()

    
    #---------------------------------------#
    #             Trim Analyses
    #---------------------------------------#
      

    def Stall_Speed(dat,geo):
        start=time.time()  
        
        dat.ksi=1  # relax parameter 
        
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        for dat.ksi in [1,.9,.8]:
            try:
                dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":10,"Track":0,"Ny":0}
                Analysis_type="Trim"   # "Trim" or "Sim"
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
            if dat.xcode==0:
                break
        
                
        dat.stall_speed=data.V_inf
        dat.Stall_speed_time=time.time()-start
        
    def Max_Cruise_Speed(dat,geo):
        start=time.time()  
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        try:
            dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
            Trim_expression={"trim_conf":"Max_Cruise_Speed_Prediction","Unknown_type":"Wind","Altitude":50,"Gamma":0,"PLA":geo.Max_engine_hp,"DeltaISA":0,"TEF":0}
            Analysis_type="Trim"   # "Trim" or "Sim"
            Results=TClass.main_trim(dat,Trim_expression,geo)
        except:
            dat.V_inf =100
            
        
        dat.max_cruise_speed=dat.V_inf 
        dat.max_cruise_speed_time=time.time()-start

    def TakeOffDistance(dat,geo):
        
        start=time.time()  
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        for dat.ksi in [1,.9,.8]:
            try:
                dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0}
                Analysis_type="Trim"   # "Trim" or "Sim"
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
            if dat.xcode==0:
                break
            
        V_stall=dat.V_inf
            
        for dat.ksi in [1,.9,.8]:
            try:
                dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":0,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
                Analysis_type="Trim"   # "Trim" or "Sim"
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
            if dat.xcode==0:
                break
            
        CL_gr=dat.CLtot
        CD_gr=dat.CDtot
        CDmin=geo.CD0
        CLmax=geo.CL_max
        
        Trim_expression={"trim_conf":"Straight_flight_ROC","Unknown_type":"Wind","V_inf":V_stall*1.3,"Altitude":50,"PLA":geo.Max_engine_hp,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0}
        Results=TClass.main_trim(dat,Trim_expression,geo)
        
        Thrust=dat.Thrust
        ###############################################
        # HP_to = Maximum available HP at take off Altitude
        Vstall = V_stall * 3.28  ; #ft/s (CALIBRATED)
        S=geo.S_ref_w * 3.28**2;  # ft**2
        W=dat.Mass_kg*2.2; #  lb MAximum take off Weight
        AR= geo.Aspect_ratio_w; # Aspect Ratio
        e = 1.78*(1-0.045*AR**.68)-0.64 ; # oswald efficiency
        k=1/(np.pi*AR*e) ; #Lift ýnduced drag constant
        
        CLmax=CLmax ;
        CDmin_to=CDmin +0.02 ; # Parasite drag for semi opened flap condition
        mu = 0.005; # coefficient of rolling friction
        eta_gr = 0.7; #propeller efficiency at ground run
        eta_c = 0.8; #propeller efficiency at climb 
        CDgr=CD_gr ;# ground run drag coefficient
        CLgr=CL_gr  ;# ground run lift coefficient
        rho_to = dat.rho *0.00194 #  0.002378; #density for take off condition (sea level) because of VELOCITYs are Calibrated
        Vlof = Vstall*1.15; # lift off speed
        Vroll=Vlof; #velocity at aircraft start to rotate in ground
        Vgr= Vlof/(2)**.5 ;# Average take off speed for ground run
        
        L_gr=.5*rho_to*S*CLgr*Vgr**2;# Lift Force at Ground run condition
        D_gr=.5*rho_to*S*CDgr*Vgr**2;# Drag Force at Ground run condition
        
        
        T_gr = Thrust*0.2248089431 ; # Thrust available at ground run condiditon for related HP_to
        
        
        a= (32.174/W)*(T_gr-D_gr-mu*(W-L_gr)); # Ground run acceleration
        Sg = Vlof**2/(2*a); # Ground Run
        Srot = 2*Vlof; # Distance taken at rotation phase
        Vc = 1.2*Vstall;
        Vtr = 1.15*Vstall;
        T_c = eta_c*550*geo.Max_engine_hp / Vc ; # Thrust available at climb condiditon for related HP_to
        CLc=CLmax*(1/1.2)**2;  # CL at climb point
        CDc=CDmin_to+k*CLc**2; # CD at climb point
        
        D_c=.5*rho_to*S*CDc*Vc**2;# Drag Force at climb point
        
        Theta_climb = np.arcsin((T_c/W)-(CDc/CLc)) ; # Climb angle in radian
        Theta_climb_degree = Theta_climb * 180/np.pi;
        
        n= (.5*rho_to*S*0.9*CLmax*Vtr**2)/W ; #Load Factor for transition phase (L/W)
        R= Vtr**2/(32.174*(n-1)); # Turn Radius for transition phase
        
        #R= 0.2156* Vstall**2# Also this formula can be used for radius of rotation
        
        Str = R*np.sin(Theta_climb); # Horizontal distance taken in Translation phase
        htr = R*(1-np.cos(Theta_climb)); #Vertical distance taken in Translation phase
        h_obs = 50 ; #ft
        Sc = (h_obs-htr)/np.tan(Theta_climb); #Horizontal distance taken in climb phase
        
        Take_off_distance = Sg+Srot+Str+Sc ; 
        Take_off_distance_SI=Take_off_distance*.3048
        
        dat.takeoffdistance=Take_off_distance_SI
        dat.takeoffdistance_time=time.time()-start
        
    def Sustained_Turn(dat,geo):
        
        start=time.time()  
        
        dat.ksi=1  # relax parameter 
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
                    
        # For Complete Analysis #
        # Vinf_arr=np.arange(np.floor(cls.stall_speed)+1,np.floor(cls.max_cruise_speed)+7,5)
        
        Vinf_arr= dat.V_max_range
        
        Nz_max=0
        for Nz in np.arange(1,3.75,0.25):
            for V_inf in Vinf_arr:
                for dat.ksi in [1,0.9,0.8]:
                
                    Phi=np.arccos(1/Nz)*180/np.pi
                    Trim_expression={"trim_conf":"Coordinated_Turn","Unknown_type":"Wind","V_inf":V_inf,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":0,"Phi":Phi}
                    Analysis_type="Trim"   # "Trim" or "Sim"
                    
                    try:
                        Results=TClass.main_trim(dat,Trim_expression,geo)
                        if dat.xcode==0:
                            break   
                    except:
                        dat.xcode=1
                    
                if dat.xcode==0 :
                    plt.scatter(V_inf,Nz,color='b')
                    if Nz>Nz_max:
                        Nz_max=Nz
                        V_max_nz=dat.V_inf
                elif dat.xcode!=0 :
                    plt.scatter(V_inf,Nz,color='r')
                
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("V_EAS (m/s)")
        plt.ylabel("NZ")
        plt.title('Load Factor Map')
        plt.show()
        
        dat.Sustained_turn_Nz_max=Nz_max
        dat.Sustained_turn_V_max_nz=V_max_nz
        dat.Sustained_turn_time=time.time()-start

    def Range(dat,geo):
        start=time.time()  
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Body","V_inf":50,"Altitude":2500,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}
        Analysis_type="Trim"   # "Trim" or "Sim"
        Results=TClass.main_trim(dat,Trim_expression,geo)
                
        e = 1.78*(1-0.045*geo.Aspect_ratio_w**.68)-0.64  #  oswald efficiency
        k=1/(np.pi*geo.Aspect_ratio_w*e)  # Lift ınduced drag constant
        eta= 0.90# propeller efficiency at cruise
        
        Gph=7 #Gallons per hour
        Lph=Gph*6 # pound per hour
        Average_BHP=geo.Max_engine_hp/2
        Cbhp=Lph/Average_BHP
        
        # Cbhp = 0.6 # British
        rho=dat.rho*0.00194 # British
        Usable_Fuel_Perc=0.92
        Reserved_Fuel=0.1
        
        S=geo.S_ref_w*10.7639  #lb
        CDmin=geo.CD0
        
        Wto=dat.Mass_kg*2.2 # lb
        
        Cruise_consumption_ratio=0.9  #Taxi, Take off and climb Fuel consumption
        
        Wc=Wto - (dat.Weight_full_fuel*dat.Fuel_percentage*Usable_Fuel_Perc*(1-Cruise_consumption_ratio)/100)*2.2 # lb
        
        Wl=Wto - (dat.Weight_full_fuel*dat.Fuel_percentage*Usable_Fuel_Perc*(1-Reserved_Fuel)/100)*2.2 # lb
        
        Vinf=np.arange(np.floor(dat.stall_speed)+1,np.floor(dat.max_cruise_speed)-1)
            

        Vinf=Vinf*3.28
        
        CL = Wc/(.5*rho*Vinf**2*S)
        CD = CDmin+k*CL**2
        Ct = Vinf*Cbhp/(550*eta*3600)
        
        #Range for Profile 2 (changing velocity , constant AOA, constant altitude)
        
        Range2 = (1/(CD*Ct))*(8*CL/(rho*S))**.5*(Wc**.5-Wl**.5)
        Range2=Range2/3280 # km
        
        #Range for Profile 3 (Constant velocity , constant AOA, changing altitude)
        
        Range3= Vinf*(CL/CD)/Ct*np.log(Wc/Wl)
        Range3=Range3/3280 # km
        
        Vinf=Vinf/3.28
       
        
        plt.plot(Vinf,Range2,color='g',label="Range")
        # plt.plot(Vinf,Range3,color='r',label="Range3")
        plt.legend(loc="upper right")
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("$V_{TAS} (m/s)$")
        plt.ylabel("$Range (km)$")
        
                
        plt.title('Range')
        plt.show()
          
        Range2=np.round(Range2,4) 
        dat.Max_Range=np.max(Range2)
        dat.V_max_range=Vinf[np.where(Range2==dat.Max_Range)]
        dat.Range_time=time.time()-start
        
    def Endurance(dat,geo):
        start=time.time()  
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Body","V_inf":50,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}
        Analysis_type="Trim"   # "Trim" or "Sim"
        Results=TClass.main_trim(dat,Trim_expression,geo)
                
        e = 1.78*(1-0.045*geo.Aspect_ratio_w**.68)-0.64  #  oswald efficiency
        k=1/(np.pi*geo.Aspect_ratio_w*e)  # Lift ınduced drag constant
        eta= 0.80 # propeller efficiency at cruise
        
        Gph=7 #Gallons per hour
        Lph=Gph*6 # pound per hour
        Average_BHP=geo.Max_engine_hp/2.5
        Cbhp=Lph/Average_BHP
        
        # Cbhp = 0.45 # British
        rho=dat.rho*0.00194 # British
        Usable_Fuel_Perc=0.92
        Reserved_Fuel=0.1
        
        
        S=geo.S_ref_w*10.7639  #lb
        CDmin=geo.CD0
        Wto=dat.Mass_kg*2.2 # lb
        
        Loiter_consumption_ratio=0.9  #Taxi, Take off and climb Fuel consumption
        
        Wc=Wto - (dat.Weight_full_fuel*dat.Fuel_percentage*Usable_Fuel_Perc*(1-Loiter_consumption_ratio)/100)*2.2 # lb
        
        Wl=Wto - (dat.Weight_full_fuel*dat.Fuel_percentage*Usable_Fuel_Perc*(1-Reserved_Fuel)/100)*2.2 # lb
        
        Vinf=np.arange(np.floor(dat.stall_speed)+1,np.floor(dat.max_cruise_speed)-1)
            

        Vinf=Vinf*3.28
        
        CL = Wc/(.5*rho*Vinf**2*S)
        CD = CDmin+k*CL**2
        Ct = Vinf*Cbhp/(550*eta*3600)
        
        
        Endurance= (CL/CD)/Ct*np.log(Wc/Wl)
        Endurance=Endurance/3600 #Minutes
        
        Vinf=Vinf/3.28
       
        
        plt.plot(Vinf,Endurance,color='r',label="Endurance")
        plt.legend(loc="upper right")
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("$V_{TAS} (m/s)$")
        plt.ylabel("$Endurance (hr)$")
        plt.title('Endurance Calculation')
        plt.show()
        
        Endurance=np.round(Endurance,4)
        dat.Max_Endurance=np.max(Endurance)
        dat.V_max_Endurance=Vinf[np.where(Endurance==dat.Max_Endurance)]
        dat.Endurance_time=time.time()-start
        
    def ROC(dat,geo):
        start=time.time()  
        
        
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        Vinf_arr=np.arange(np.floor(dat.stall_speed)+5,np.floor(dat.max_cruise_speed)-5,2.5)

        ROC_max=0
        i=-1
        k=0
        Altitude_arr=[0,2500,5000]
        for iAltitude in [0,3000,6000]: # 0 3000 6000 90000
            
            roc=[]
            # T_available=[]
            # T_req=[]
            Gamma=[]
            V_t=[]

            for V_inf in Vinf_arr:
            
                dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                Trim_expression={"trim_conf":"Straight_flight_ROC","Unknown_type":"Wind","V_inf":V_inf,"Altitude":iAltitude+50,"PLA":geo.Max_engine_hp*0.95,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0}
                Analysis_type="Trim"   # "Trim" or "Sim"
                try:
                    Results=TClass.main_trim(dat,Trim_expression,geo)
                except:
                    continue
                
                legends=['r','b','k','g','y']
                if dat.xcode==0:
                    roc.append(dat.h_dot)
                    Gamma.append(dat.Gamma)
                    V_t.append(dat.V_inf)
                    
                    if dat.h_dot>ROC_max:
                        ROC_max=dat.h_dot
                        V_max_ROC=dat.V_inf
                    
                    # T_available.append(dat.Thrust)
                    # T_req.append(dat.CDtot*0.5*dat.rho*V_inf**2*geo.S_ref_w)
            if len(V_t) != 0  :  
                i=i+1
                plt.plot(V_t,roc,color=('%s'% legends[i]),label=("ROC%0.fm" % Altitude_arr[k]))
                k=k+1
        
        # plt.plot(V_t,Gamma,color='b',label="Gamma")
        plt.legend(loc="upper right")
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("$V_{TAS} (m/s)$")
        plt.ylabel("$ROC (m/s)$")
        plt.title('Rate of Climb Calculation')
        plt.show()
        

        dat.ROC_max=ROC_max
        dat.V_max_ROC=V_max_ROC
        dat.ROC_time=time.time()-start
        
    def Ceiling(dat,geo):
        start=time.time()  
        
        dat.ksi=1  # relax parameter 
        geo.Propeller_efficiency=0.7
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        Vinf_arr=np.arange(np.floor(dat.stall_speed)+10,np.floor(dat.max_cruise_speed)-10,2.5)

        i=-1
        for iH_dot in [0,100]:  # fpm 0 100 300 500  ['Absolute','Service','Cruise', 'Combat']
            
            Alt=[]
            Gamma=[]
            V_t=[]

            for V_inf in Vinf_arr:
                try:
                    dat.Altitude=50 #m  init condition
                    dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                    Trim_expression={"trim_conf":"Ceiling","Unknown_type":"Wind","V_inf":V_inf,"H_dot":iH_dot/(60*3.28),"PLA":geom.Max_engine_hp*0.78,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0}
                    Analysis_type="Trim"   # "Trim" or "Sim"
                    Results=TClass.main_trim(dat,Trim_expression,geo)       
                except:
                    continue
                
                legends=['r','b','g']
                labels=['Absolute','Service','Cruise']
                if dat.xcode==0 and dat.Altitude>100:
                    Alt.append(dat.Altitude)
                    V_t.append(V_inf)
            if len(V_t) != 0  :  
                i=i+1
                plt.plot(V_t,Alt,color=('%s'% legends[i]),label=("%s Ceiling %0.f fpm" % (labels[i],iH_dot)))
                
                Vt=np.asarray(V_t)
                Alt=np.round(Alt,4)
                exec("cls.Altitude_%s=np.max(Alt)" % labels[i])
                exec("cls.V_%s=Vt[np.where(Alt==np.max(Alt))]" % labels[i])
                
        plt.legend(loc="lower left")
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("$V_{TAS} (m/s)$")
        plt.ylabel("Altitude (m)")
        plt.title('Ceiling Calculation')
        plt.show()
             
        dat.Ceiling_time=time.time()-start

        
    # =============================================================================
    # =============================================================================
    # =============================================================================
    # # #                       FLYING QUALITY MANEUVERS                      # # #
    # =============================================================================
    # =============================================================================
    # =============================================================================
    
    def Flight_Map(dat,geo):
        start=time.time()  
        
        dat.ksi=1  # relax parameter 
        geo.Max_engine_hp=160*0.78
        
        for Weight_Coeff in [0,1]:

            # Weight_Coeff=0.75 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
            dat.Fuel_percentage=100*Weight_Coeff
            dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
            dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
            dat.Crew_weight_left=dat.Weight_max_crew_left
            
            Vinf_arr=np.arange(np.floor(dat.stall_speed)-5,np.floor(dat.max_cruise_speed)+10,5)
            i=0;j=0;k=0;m=0
    
            for Altitude in np.arange(0,10000,1000):
                for V_inf in Vinf_arr:
                    dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                    # Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":16,"Altitude":50,"Gamma":0,"DeltaISA":0,"Mass_kg":1088,"TEF":40,"Track":0,"Ny":0}
                    Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Wind","V_inf":V_inf,"Altitude":Altitude+50,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0}
                    Analysis_type="Trim"   # "Trim" or "Sim"
                    
                    try:
                        Results=TClass.main_trim(dat,Trim_expression,geo)
                    except:
                        dat.xcode=1
                        
                    if dat.xcode!=0 :
                        if m==0:
                            plt.scatter(V_inf,Altitude,color='r',label='No trim')
                            m=m+1
                        else:
                            plt.scatter(V_inf,Altitude,color='r')
                            
                        
                    elif dat.xcode==0 :
                        Control.control_inputs(dat,geo)
                        
                        A_long=dat.eigenvalues_long=data.Flight_character_body_long
                        A_lat=dat.eigenvalues_lat=data.Flight_character_body_lat
                        
                        Short_Dmp=np.max(A_long.Dmp_ratio)
                        Phugoid_Dmp=np.min(A_long.Dmp_ratio)
                        Dutch_roll_Dmp=np.max(A_lat.Dmp_ratio)
                        
                        Short_Nat_Frq=np.max(A_long.Nat_Frq)
                        Phugoid_Nat_Frq=np.min(A_long.Nat_Frq)
                        Dutch_roll_Nat_Frq=np.max(A_lat.Nat_Frq)
                        
                        Spiral_time_to_double=np.max(A_lat.Time_to_double)
                        Roll_time_constant=np.min(A_lat.Time_constant)
                        
                        if Short_Dmp>0.3 :
                            Short_Level=1
                        elif Short_Dmp>0.2 :
                            Short_Level=2
                        else : Short_Level=3
                            
                            
                        if Phugoid_Dmp>0.04 :
                            Phugoid_Level=1
                        elif Phugoid_Dmp>0:
                            Phugoid_Level=2
                        else: Phugoid_Level=3
                             
                        if Roll_time_constant<1.4 :
                            Roll_Level=1
                        elif Roll_time_constant<3.0:
                            Roll_Level=2
                        else: Roll_Level=3 
                            
                            
                        if Spiral_time_to_double>20 or Spiral_time_to_double==0 :
                            Spiral_Level=1
                        elif Spiral_time_to_double>8:
                            Spiral_Level=2
                        else : Spiral_Level=3
                            
                        
                        if Dutch_roll_Dmp>0.08 :
                            Dutch_roll_Level=1
                        elif Dutch_roll_Dmp>0.02:
                            Dutch_roll_Level=2
                        else : Dutch_roll_Level=3
                            
                            
                        Levels=[Short_Level,Phugoid_Level,Roll_Level,Spiral_Level,Dutch_roll_Level]
                        Level=np.max(Levels)
                                            
                        if Level==1:
                            if i==0:
                                plt.scatter(V_inf,Altitude,color='g',label='Level 1')
                                i=i+1
                            else:
                                plt.scatter(V_inf,Altitude,color='g')
                                
                        if Level==2:
                            if j==0:
                                plt.scatter(V_inf,Altitude,color='b',label='Level 2')
                                j=j+1
                            else:
                                plt.scatter(V_inf,Altitude,color='b')
                                
                                
                        elif Level==3:
                            if k==0:
                                plt.scatter(V_inf,Altitude,color='c',label='Level 3')
                                k=k+1
                            else:
                                plt.scatter(V_inf,Altitude,color='c')
                                
                            
                        
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("V_TAS (m/s)")
            plt.ylabel("Alttude (m)")
            plt.title('Flying Qualities Map')
            plt.show()         
                
        dat.FlightMap_time=time.time()-start
    
    def EigenValues(dat,geo):
        
        start=time.time()  
        
        i=-1
        legends=['r','b']
        labels=['Most Forward', 'Most Aft']
        Average_Unstable_Poles=[]
        X_arr=[]
        Y_arr=[]
        
        for Weight_Coeff in [0,1]:
            i=i+1
        
            dat.Fuel_percentage=100*Weight_Coeff
            dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
            dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
            dat.Crew_weight_left=dat.Weight_max_crew_left
            
            Vinf_arr=np.arange(np.floor(dat.stall_speed),np.floor(dat.max_cruise_speed),10)
            dat.xcode=1
            Most_Unstable_Poles=[]
            V_t=[]
            
            for V_inf in Vinf_arr:
                try:
                    dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                    Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Body","V_inf":V_inf,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}
                    Analysis_type="Trim"   # "Trim" or "Sim"
                    Results=TClass.main_trim(dat,Trim_expression,geo)
                except:
                    continue
                
                if dat.xcode==0:
                    Control.control_inputs(dat,geo)
            
                    X = [x.real for x in dat.Eig_body]
                    Y = [x.imag for x in dat.Eig_body]
                    Most_Unstable_Poles.append(max(X))
                    V_t.append(dat.V_inf)
                    X_arr.append(X)
                    Y_arr.append(Y)
            
            plt.plot(V_t,Most_Unstable_Poles, color=legends[i],label=labels[i])
            Average_Unstable_Poles.append(np.average(Most_Unstable_Poles))
            
        plt.legend(loc="upper right")
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.title('Most Unstable Poles')
        plt.xlabel("V_eas")
        plt.ylabel("Poles")
        plt.show()
        
        X_arr=np.asarray(X_arr)
        Y_arr=np.asarray(Y_arr)
        plt.scatter(X_arr,Y_arr, color='r')
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.title('EIGENVALUES')
        plt.show()
        
        
        dat.Eig_time=time.time() -start   
        dat.Average_Unstable_Pole=np.average(Average_Unstable_Poles)
        
    def Crosswind(dat,geo):
        start=time.time()  
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        Analysis_type="Trim"   # "Trim" or "Sim"
        
        dat.xcode=1
        
        for dat.ksi in [1,0.9,0.8]:
            try:
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
        
        V_inf=1.3*dat.V_inf
           
        for Crosswind in np.arange(12,1,-0.5):
            
            try:
                Trim_expression={"trim_conf":"Sideslip_Landing","Unknown_type":"Wind","V_inf":V_inf,"Altitude":50,"Gamma":-2,"DeltaISA":0,"TEF":40,"Track":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":Crosswind}
                Results=TClass.main_trim(dat,Trim_expression,geo)
            except:
                continue
        
            if dat.xcode==0:
                break
            if dat.xcode==1:
                Crosswind=0
                
        dat.Max_Tolerable_Crosswind=Crosswind
        dat.Crosswind_time=time.time()-start
        
    def Crosswind_TBD(dat,geo):
        start=time.time()  
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        Analysis_type="Trim"   # "Trim" or "Sim"
        
        dat.xcode=1
        
        for dat.ksi in [1,0.9,0.8]: 
            try:
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
        
        V_inf=1.3*dat.V_inf
        
        dat.Crosswind_mag_ms=3
           
        # for dat.ksi in [1,0.9,0.8]:
            # try:
        Trim_expression={"trim_conf":"Crosswind_Trim","Unknown_type":"Wind","V_inf":V_inf,"Altitude":50,"Gamma":-2,"DeltaISA":0,"TEF":40,"Track":0,"Crosswind_dir_earth_deg":90,"Rudder":10}
        Results=TClass.main_trim(dat,Trim_expression,geo)
            #     break
            # except:
            #     continue
    
        if dat.xcode==0:
            Crosswind=dat.Crosswind_mag_ms
        if dat.xcode==1:
            Crosswind=0
                
        dat.Max_Tolerable_Crosswind=Crosswind
        dat.Crosswind_time=time.time()-start
        
    
         
    def Controllability(dat,geo):
        start=time.time()
        
        dat.ksi=1  # relax parameter 
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        
        for Aoa in np.arange(-3,23,2):
            for Aos in np.arange(0,24,3):
                dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
                # Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":16,"Altitude":50,"Gamma":0,"DeltaISA":0,"Mass_kg":1088,"TEF":40,"Track":0,"Ny":0}
                Trim_expression={"trim_conf":"Quasi_Trim","Unknown_type":"Wind","V_inf":40,"Altitude":400,"Gamma":0,"PLA":120,"DeltaISA":0,"TEF":0,"Aoa":Aoa,"Beta":Aos,"P":0.00,"Q":0.00,"R":0.00,"Phi":0}
                Analysis_type="Trim"   # "Trim" or "Sim"
                
                try:
                    Results=TClass.main_trim(dat,Trim_expression,geo)
                except:
                    dat.xcode=1
                    
                if dat.xcode==0 :
                    plt.scatter(Aoa,Aos,color='b')
                elif dat.xcode!=0 :
                    plt.scatter(Aoa,Aos,color='r')
                    
        plt.legend(loc="lower right")
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("AOA")
        plt.ylabel("AOS")
        plt.title('Controllability Map Calculation')
        plt.show()   
        
        dat.Controllability_time=time.time()-start
        
    def FPS(dat,geo):
        
        start=time.time()
        
        Weight_Coeff=0.25
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        Analysis_type="Trim"   # "Trim" or "Sim"
        
        dat.xcode=1
        
        for dat.ksi in [1,0.9,0.8]:
            try:
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
        
        V_approach=dat.V_inf*1.2
        
        try:
            
            Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Body","V_eas":V_approach,"Altitude":100,"Gamma":-3,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
            Results=TClass.main_trim(dat,Trim_expression,geo)
            
            Throttle=dat.PLA
            
            Trim_expression={"trim_conf":"Straight_flight_ROC","Unknown_type":"Wind","V_eas":V_approach+1,"Altitude":100,"PLA":Throttle,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
            Results=TClass.main_trim(dat,Trim_expression,geo)
            
            Gamma_forward = dat.Gamma
            
            Trim_expression={"trim_conf":"Straight_flight_ROC","Unknown_type":"Wind","V_eas":V_approach-1,"Altitude":100,"PLA":Throttle,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
            Results=TClass.main_trim(dat,Trim_expression,geo)
            
            Gamma_backward = dat.Gamma
            
            FPS_deriv=(Gamma_forward-Gamma_backward)/2 # degrees/ (m/s)
            FPS_deriv_kts=FPS_deriv/1.9438
            
        except:
            pass
        if dat.xcode==1:
            FPS_deriv_kts=0
            
        
        dat.FPS_deriv=FPS_deriv_kts
        dat.FPS_time=time.time()-start
            
    def CG_Envelope(dat,geo):
        
        for i in np.arange(0,1.1,0.1):
            dat.Fuel_percentage=i*100
            for j in np.arange(0,1.1,0.1):
                dat.Payload_weight=j*dat.Weight_max_payload #rear passangers+ payload
                for k in np.arange(0,1.1,0.1):
                    dat.Crew_weight_right=k*dat.Weight_max_crew_right
                    dat.Crew_weight_left=dat.Weight_max_crew_left
                    
                    WeightModule.weight_module(data,geom)  
                    plt.scatter(dat.x_cg,dat.Mass_kg,color='y')
            
        Weight_Coeff=0 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        WeightModule.weight_module(data,geom)  
        plt.scatter(dat.x_cg,dat.Mass_kg,color='b',label='Most Forward')
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        WeightModule.weight_module(data,geom)  
        plt.scatter(dat.x_cg,dat.Mass_kg,color='c',label='Mid CG')
        
        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        WeightModule.weight_module(data,geom)  
        plt.scatter(dat.x_cg,dat.Mass_kg,color='g',label='Most Aft')
            
            
        plt.minorticks_on()
        plt.legend(loc="upper left")
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.xlabel("$X_{CG} (m)$")
        plt.ylabel("$Mass (kg)$")
        plt.title('CG & Mass Envelope Calculation')
        plt.show() 
        

    def Cost_Analysis(dat,geo):
        start=time.time()  
        
        dat.Flight_Mode = 'Normal'
        Analysis_type="Trim"   # "Trim" or "Sim"

        dat.ksi=1  # relax parameter 

        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        

        Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Body","V_inf":50,"Altitude":2500,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}
        Results=TClass.main_trim(dat,Trim_expression,geo)
        
        Empty_weight_lb=dat.Empty_weight
        
        Cost=192.38*Empty_weight_lb**0.4854*geo.Max_engine_hp**0.5843 # in Dollars

        dat.Cost=Cost
        
        
        #---------------------------------------#
        #         Simulation Analyses
        #---------------------------------------#
        
    @classmethod
    def NoseDownRecovery(dat,geo):
        start=time.time()  
        
        dat.Flight_Mode = 'NDR'
        Analysis_type="Trim"   # "Trim" or "Sim"

        # Alpha arttıkça Cna çok büyüyor.Bu yüzden rudder ı çok büküyor. ?????
        # Flaperon veya P arttğı zaman yaw momenti dengelemek için rudder da artıyor. Rudder ın tek başına artması betayı çok artırıyor. (CYrudder*Rudder=CYbeta*Beta)
        # Flaperon veya P nin artması da Rudder ı çok artırıyor. Yani roll momentin yaw momente etkisi çok fazla. (Cn flaperon and Cn p)
  
        dat.ksi=1  # relax parameter 

        Weight_Coeff=1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        for dat.ksi in [1,.9,.8]:
            try:
                Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":500,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
                Results=TClass.main_trim(dat,Trim_expression,geo)
                break
            except:
                continue
            if dat.xcode==0:
                break
                
            
        Final_time=3.1 #seconds
        Frequency=10 #1/seconds   #   # it should be 20
        
        Sim_startup.Simulation_StartUp(dat,geo,Results,Final_time,Frequency)
        
        shutil.rmtree(Performance.current_path+'\\RESULTS\\NoseDownRecovery\\TestDataStored')
        src_path = Performance.current_path + '\\TRIM_SIM\\Visual3D\\TestDataStored'
        dst_path = Performance.current_path+'\\RESULTS\\NoseDownRecovery\\TestDataStored'
        shutil.copytree(src_path, dst_path)
        
        # np.min(dat.Q_dot_arr[:20])
        Available_Q=np.min(dat.Q_arr[:30])*180/np.pi # it should be less than -24 deg/sec
        
        dat.Available_Q=Available_Q
        dat.NDR_time=time.time()-start  
        
        
    def Max_Roll_Rate(dat,geo):
        start=time.time()  

        
        # Alpha arttıkça Cna çok büyüyor.Bu yüzden rudder ı çok büküyor. ?????
        # Flaperon veya P arttğı zaman yaw momenti dengelemek için rudder da artıyor. Rudder ın tek başına artması betayı çok artırıyor. (CYrudder*Rudder=CYbeta*Beta)
        # Flaperon veya P nin artması da Rudder ı çok artırıyor. Yani roll momentin yaw momente etkisi çok fazla. (Cn flaperon and Cn p)
        
        Weight_Coeff=0.5 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.xcode=1  # xcode=1 means no trim condition / xcode=0 is trim condition founded
        
        P=0.8
        while dat.xcode==1:
            
            P=P-0.02
            dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
            Analysis_type="Sim"   # "Trim" or "Sim"
            
            try:
                Trim_expression={"trim_conf":"Steady_roll","Unknown_type":"Wind","V_inf":50,"Altitude":1000,"Gamma":0,"DeltaISA":0,"TEF":0,"P":P,"Phi":0}
                Results=TClass.main_trim(dat,Trim_expression,geo)
            except:
                continue
                
            
            if dat.xcode==0:
                dat.max_roll_rate=dat.P*180/np.pi
                break
            
        Final_time=1.8 #seconds
        Frequency=10 #1/seconds   #   # it should be 20
        
        Sim_startup.Simulation_StartUp(dat,geo,Results,Final_time,Frequency)
        
        shutil.rmtree(Performance.current_path+'\\RESULTS\\Max_Roll_Rate\\TestDataStored')
        src_path = Performance.current_path + '\\TRIM_SIM\\Visual3D\\TestDataStored'
        dst_path = Performance.current_path+'\\RESULTS\\Max_Roll_Rate\\TestDataStored'
        shutil.copytree(src_path, dst_path)
        
        dat.Max_Bank=dat.Phi
        dat.Max_Roll_time=time.time()-start    
        
        
    @classmethod
    def LandingDistance(dat,geo):
        start=time.time()  
        
        Weight_Coeff=0.1 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        try:
            dat.Flight_Mode="Landing" #"Landing" or "TakeOff" or "Normal"
            Trim_expression={"trim_conf":"Fixed_theta","Unknown_type":"Wind","Theta":0,"Altitude":50,"Gamma":-3,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}
            Analysis_type="Sim"   # "Trim" or "Sim"
            Results=TClass.main_trim(dat,Trim_expression,geo)
            
            Final_time=60 #seconds
            Frequency=20 #1/seconds   #   # it should be 20
            Sim_startup.Simulation_StartUp(dat,geo,Results,Final_time,Frequency)
            
            shutil.rmtree(Performance.current_path+'\\RESULTS\\Landing_Distance\\TestDataStored')
            src_path = Performance.current_path + '\\TRIM_SIM\\Visual3D\\TestDataStored'
            dst_path = Performance.current_path+'\\RESULTS\\Landing_Distance\\TestDataStored'
            shutil.copytree(src_path, dst_path)
        except Exception as Error:
            print(Error)
            dat.landingdistance=0

        dat.landingdistance=np.max(dat.x_arr)
        dat.landingdistance_time=time.time()-start
        
    @classmethod
    def TakeOffDistance_Old(dat,geo):
        start=time.time()  
        
        Weight_Coeff=0.75 # 0 or 0.5 or 1 # Most forward/ Mid /Most Aft
        dat.Fuel_percentage=100*Weight_Coeff
        dat.Payload_weight=dat.Weight_max_payload*Weight_Coeff #rear passangers+ payload
        dat.Crew_weight_right=dat.Weight_max_crew_right*Weight_Coeff
        dat.Crew_weight_left=dat.Weight_max_crew_left
        
        dat.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
        Trim_expression={"trim_conf":"Stall_Speed_Prediction","Unknown_type":"Wind","Aoa":geom.AOA_max,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0}
        Analysis_type="Trim"   # "Trim" or "Sim"
        Results=TClass.main_trim(dat,Trim_expression,geo)
        
        
        
        geom.lift_off_speed=dat.V_inf*1.1 #m/s    SPEC   #Stall speed cinsinden tanımlayıp bunu sil
        
        dat.Flight_Mode="TakeOff" #"Landing" or "TakeOff" or "Normal"
        Trim_expression={"trim_conf":"Fixed_theta","Unknown_type":"Wind","Theta":0,"Altitude":0,"Gamma":0,"DeltaISA":0,"TEF":20,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}
        Analysis_type="Sim"   # "Trim" or "Sim"
        Results=TClass.main_trim(dat,Trim_expression,geo)
        
        Final_time=50 #seconds
        Frequency=30 #1/seconds   #   # it should be 30
        Sim_startup.Simulation_StartUp(dat,geo,Results,Final_time,Frequency)
        
        shutil.rmtree(Performance.current_path+'\\RESULTS\\Take_off_Distance\\TestDataStored')
        src_path = Performance.current_path + '\\TRIM_SIM\\Visual3D\\TestDataStored'
        dst_path = Performance.current_path+'\\RESULTS\\Take_off_Distance\\TestDataStored'
        shutil.copytree(src_path, dst_path)

        dat.takeoffdistance=np.max(data.x_arr)
        dat.takeoffdistance_time=time.time()-start

        
        
        