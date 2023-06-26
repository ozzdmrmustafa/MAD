#Author :  Mustafa Özdemir /September.2022
#Mail   :  ozzdmr.mustafa@gmail.com

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
from PERFORMANCE.PERFANALYSIS import Performance
from matplotlib import pyplot as plt
from SIM_STARTUP import Sim_startup
import time

class Main:
    
    current_path = os.getcwd()
    
    # =============================================================================
    # Settings
    # =============================================================================
    geom.Fuselage_effect="off"    
    data.Weight_calculation = "on"
    geom.Method='Cessna'  # Cessna Torenbeek
    geom.Wing_pos='high'
    geom.lg_situation= 'non_retractable'
    geom.Fuel_location= "fuselage"
    data.print_info = True
    data.sim_plot='on'
    
    ## Gross _weight ### if  Weight_calculation=on then data.Gross_weight will be used as initial astimation and it will change
    data.Gross_weight=1040  
    
    data.Weight_full_fuel=108 #kg
    data.Weight_max_payload= 94 #(154+25) #kg #rear passangers+ payload
    data.Weight_max_crew_right=90 #kg
    data.Weight_max_crew_left=90 #kg

    ## INIT values -THEY WILL CHANGE ##
    data.Fuel_percentage=100
    data.Payload_weight=data.Weight_max_payload
    data.Crew_weight_right=data.Weight_max_crew_right
    data.Crew_weight_left=data.Weight_max_crew_left
    ###################################
    
    
    # =============================================================================
    # =============================================================================
    # =============================================================================
    # # #                              MAIN PART                              # # #
    # =============================================================================
    # =============================================================================
    # =============================================================================
        

if __name__=="__main__":
    
    
    # =============================================================================
    # Change Design Variable
    # ============================================================================= 

    
    geom.S_ref_w=17.56
    geom.Aspect_ratio_w=6.63
    geom.Incidence_deg_w=0.46
    geom.W_LE_x=1.98
    geom.S_ref_ht=3
    geom.S_ref_vt=1.62*2
    geom.Max_engine_hp=180
    data.Weight_full_fuel=120
    

    # =============================================================================
    # Execute
    # =============================================================================
    
    geom.calculate_geo()
    WeightModule.weight_module(data,geom) 
    geom.calculate_Datcom(data)
    
    # =============================================================================
    # Limitations
    # =============================================================================
    
    geom.Flaperon_Limits=[-25,25]   #
    geom.Aoa_Limits=[-3,geom.AOA_max*1.05]   # 
    geom.Beta_Limits=[-20,20]   # 
    geom.TEF_Limits=[0,40]   # 
    geom.HTL_Limits=[-25,15]   # 
    geom.Rudder_Limits=[-25,25]   # 
    
    # =============================================================================
    # =============================================================================
    # =============================================================================
    # Sample Case
    # =============================================================================
    # =============================================================================
    # =============================================================================
    
    data.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
    
    #Trim_expression={"trim_conf":"Straight_flight","Unknown_type":"Body","V_inf":50,"Altitude":50,"Gamma":0,"DeltaISA":0,"TEF":0,"Track":0,"Ny":0,"Crosswind_dir_earth_deg":90,"Crosswind_mag_ms":0}

    Trim_expression={"trim_conf":"Coordinated_Turn","Unknown_type":"Wind","V_inf":50,"Altitude":1000,"Gamma":0,"DeltaISA":0,"TEF":0,"Phi":60}
    
    Analysis_type="Sim"   # "Trim" or "Sim"
    Results=TClass.main_trim(data,Trim_expression,geom)
    
    #Control.control_inputs(data,geom)
    
    Final_time=3 #seconds
    Frequency=10 #1/seconds   #   # it should be 20
    Sim_startup.Simulation_StartUp(data,geom,Results,Final_time,Frequency)
    
    # =============================================================================
    # =============================================================================
    # =============================================================================
    # Maneuvers
    # =============================================================================
    # =============================================================================
    # =============================================================================
    

    # Performance.Stall_Speed(data,geom)
    # Performance.Max_Cruise_Speed(data,geom)
    # Performance.Range(data,geom) # Cruise Speed
    # Performance.Endurance(data,geom) #Loiter Speed
    # Performance.Sustained_Turn(data,geom)
    # Performance.ROC(data,geom)
    # Performance.Ceiling(data,geom)
    # Performance.CG_Envelope(data,geom) 
    # Performance.FPS(data,geom) 
    # Performance.Crosswind(data,geom)  
    # Performance.EigenValues(data,geom)
    # Performance.Max_Roll_Rate(data,geom)  
    # Performance.TakeOffDistance(data,geom)
    # Performance.LandingDistance(data,geom)
    # Performance.NoseDownRecovery(data,geom)
    # Performance.Cost_Analysis(data,geom)
    # Performance.Flight_Map(data,geom) 
    # Performance.Controllability(data,geom)  
    # Performance.Instant_Turn(data,geom)
    # Performance.Push_down(data,geom)
    # Performance.Spin_Trim(data,geom)
        
        