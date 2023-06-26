Author :  Mustafa Özdemir /September.2022
Mail   :  ozzdmr.mustafa@gmail.com


Installation Instructions

1) Download avl.exe (AVL 3.40b executable for Windows) from "https://web.mit.edu/drela/Public/web/avl/"
2) Copy inside of the "..\MAD_V2\AERODYNAMICS" folder

Running Instructions

1) Define Input Section inside of the "..\MAD_V2\MAIN_ONE_GEOMETRY.py"
2) Run "MAIN_ONE_GEOMETRY.py" script



%% INPUT SECTION %%
%%%%%%%%%%%%%%%%%%%%%%%%
geom.Fuselage_effect="off"    
data.Weight_calculation = "on"
geom.Method='Cessna'  # Cessna or Torenbeek
geom.Wing_pos='high'
geom.lg_situation= 'non_retractable'
geom.Fuel_location= "fuselage"
data.print_info = True
data.sim_plot='on'

geom.S_ref_w=17.56
geom.Aspect_ratio_w=6.63
geom.Incidence_deg_w=0.46
geom.W_LE_x=1.98
geom.S_ref_ht=3
geom.S_ref_vt=1.62*2
geom.Max_engine_hp=180
data.Weight_full_fuel=120

geom.Flaperon_Limits=[-25,25]   
geom.Aoa_Limits=[-3,geom.AOA_max*1.05]   
geom.Beta_Limits=[-20,20]    
geom.TEF_Limits=[0,40]   
geom.HTL_Limits=[-25,15]   
geom.Rudder_Limits=[-25,25]   

data.Flight_Mode="Normal" #"Landing" or "TakeOff" or "Normal"
Final_time=3 #seconds
Frequency=10 #1/seconds   #   # it should be 20

%%%%%%%%%%%%%%%%%%%%%%%%%


%% TRIM & SIM SECTION %%
%%%%%%%%%%%%%%%%%%%%%%%%
Trim_expression={"trim_conf":"Coordinated_Turn","Unknown_type":"Wind","V_inf":50,"Altitude":1000,"Gamma":0,"DeltaISA":0,"TEF":0,"Phi":60}  # Trim Condition Definition
Analysis_type="Sim"   # "Trim" or "Sim"
Results=TClass.main_trim(data,Trim_expression,geom) # Obtaining Initial Condition from Trim
Sim_startup.Simulation_StartUp(data,geom,Results,Final_time,Frequency)  # Perform Simulation

%%%%%%%%%%%%%%%%%%%%%%%%
