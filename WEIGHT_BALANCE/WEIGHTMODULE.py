import numpy as np

# WeightModule.weight_module
# Roskam design Part 5 book

class WeightModule:
    
    conversion_m2ft=3.2808
    conversion_kg2lbs=2.2046    
    Tech_factor=1

    @classmethod
    def Definitions(cls,dat,geo):

        cls.Method=geo.Method  # Cessna Torenbeek
        cls.Wing_pos= geo.Wing_pos
        cls.lg_situation= geo.lg_situation
        cls.Fuel_location= geo.Fuel_location
        
        cls.S= geo.S_ref_w*cls.conversion_m2ft**2       # Wing area in ft2
        cls.Nult=  5.7       # design ultimate load factor
        cls.A=   geo.Aspect_ratio_w         # wing aspect ratio
        
        cls.Sv= geo.S_ref_vt*0.5*cls.conversion_m2ft**2          # vertical tail area in ft2
        cls.Av=  geo.Aspect_ratio_vt        # vertical tail aspect ratio
        cls.Sweep_quart_v=geo.LE_sweep_deg_vt    # vertical tail quarter chord aspect ratio
        vt_tickness_ratio=int(geo.datcom_vt_airfoil[-2:])
        cls.Trv=(vt_tickness_ratio/100)* np.max(geo.C_vt)*cls.conversion_m2ft      # vertical tail maximum root tickness in ft
        
        cls.Sh= geo.S_ref_ht*cls.conversion_m2ft**2         # horizontal tail area in ft2
        cls.Ah=  geo.Aspect_ratio_ht        # horizontal tail quarter chord aspect ratio
        
        ht_tickness_ratio=int(geo.datcom_ht_airfoil[-2:])
        cls.Trh=(ht_tickness_ratio/100)* np.max(geo.C_ht)*cls.conversion_m2ft      # horizontal tail maximum root tickness in ft
        
        cls.Pmax= (geo.Fuselage_max_width)*4*cls.conversion_m2ft     # maximum fuselage perimeter in ft width*4
        cls.Lfn= geo.Fuselage_length*cls.conversion_m2ft    # fuselage length ft
        cls. Npax=4    # number of passangers including pilots
        
        cls.Wl= cls.Wto*0.997  # design landing weight in lbs Roskan design part 1 table 3.3
        cls.Nult_land= 5.7     # ultimate load factor for landing
        distance_spinner2lowerfuselage= 0.40 #m
        cls.Lsm= (( geo.Location_lg_body_right[1]**2 + (geo.Location_lg_body_right[2]-distance_spinner2lowerfuselage)**2 )**.5)*cls.conversion_m2ft  # shock strut length for main gear in ft
        cls.Lsn=   (geo.Location_lg_body_nose[2]-distance_spinner2lowerfuselage)*cls.conversion_m2ft   # shock strut length for nose gear in ft
        
        cls.Pto= geo.Max_engine_hp   # Required take off power in hp
        
        cls.Weight_full_fuel=dat.Weight_full_fuel*cls.conversion_kg2lbs
        cls.Weight_max_payload=dat.Weight_max_payload*cls.conversion_kg2lbs #rear passangers+ payload
        cls.Weight_max_crew_right=dat.Weight_max_crew_right*cls.conversion_kg2lbs
        cls.Weight_max_crew_left=dat.Weight_max_crew_left*cls.conversion_kg2lbs
    
        cls.Weight_actual_fuel=dat.Fuel_percentage*dat.Weight_full_fuel*cls.conversion_kg2lbs/100
        cls.Weight_actual_payload=dat.Payload_weight*cls.conversion_kg2lbs #rear passangers+ payload
        cls.Weight_actual_crew_right=dat.Crew_weight_right*cls.conversion_kg2lbs
        cls.Weight_actual_crew_left=dat.Crew_weight_left*cls.conversion_kg2lbs
            
    @classmethod
    def wing_weight(cls,dat,geo):
        
        if cls.Method=='Cessna':
            
            # Cantilever
            # Weight=0.04674*cls.Wto**0.397*cls.S**0.360*cls.Nult**0.397*cls.A**1.712
            
            #Strut Braced
            Weight=0.002933*cls.S**1.018*cls.A**2.473*cls.Nult**0.611*cls.Tech_factor
            
            W_LE_x=2.0  # x distance between nose and  wing leading edge at centerline
            W_LE_y=0  # y distance between nose and  wing leading edge at centerline
            W_LE_z=0.65  # z distance between nose and  wing leading edge at centerline
            
                        
            Cgx = geo.W_LE_x + np.tan(geo.LE_sweep_deg_w * np.pi/180) * geo.Span_w * 0.5 * 0.4\
                + 0.38 * ( np.min(geo.C_w) + (1-0.4) * (np.max(geo.C_w)-np.min(geo.C_w)))
            Cgy = [-geo.Span_w * 0.5 * 0.4,geo.Span_w * 0.5 * 0.4]
            Cgz = geo.W_LE_z
            

            
        
        return [Weight,Cgx,Cgy,Cgz]
        
    @classmethod    
    def ht_weight(cls,dat,geo):
        
        if cls.Method=='Cessna':
            Weight=3.184*cls.Wto**0.887*cls.Sh**0.101*cls.Ah**0.138/(174.04*cls.Trh**0.223)
            
            Cgx = geo.HT_LE_x + np.tan(geo.LE_sweep_deg_ht * np.pi/180) * geo.Span_ht * 0.5 * 0.38\
                + 0.42 * ( np.min(geo.C_ht) + (1-0.38) * (np.max(geo.C_ht)-np.min(geo.C_ht)))
            Cgy = [-geo.Span_ht * 0.5 * 0.38,geo.Span_ht * 0.5 * 0.38]
            Cgz = geo.HT_LE_z
            

        
        return[Weight,Cgx,Cgy,Cgz]
        
    @classmethod
    def vt_weight(cls,dat,geo):
        if cls.Method=='Cessna':
            Weight=1.68*cls.Wto**0.567*cls.Sv**1.249*cls.Av**0.482/(639.95*cls.Trv**0.747*np.cos(cls.Sweep_quart_v*0.9*np.pi/180)**0.882)
            
            Cgx = geo.VT_LE_x + np.tan(geo.LE_sweep_deg_vt * np.pi/180) * geo.Span_vt * 0.5 * 0.38\
                + 0.42 * ( np.min(geo.C_vt) + (1-0.38) * (np.max(geo.C_vt)-np.min(geo.C_vt)))
            Cgy = 0
            Cgz = geo.VT_LE_z - geo.Span_vt * 0.5 * 0.38
            

        
        return[Weight,Cgx,Cgy,Cgz]
        
    @classmethod    
    def fuselage_weight(cls,dat,geo):
        
        if cls.Method=='Cessna':
            if cls.Wing_pos=='low':
                Weight=0.04682*cls.Wto**0.692*cls.Pmax**0.374*cls.Lfn**0.590*cls.Tech_factor
            if cls.Wing_pos=='high':
                Weight=14.86*cls.Wto**0.144*(cls.Lfn/cls.Pmax)**0.778*cls.Lfn**0.383*cls.Npax**0.455*1.1
                
            Cgx=geo.Fuselage_length*0.34
            Cgy=0
            Cgz=0
            


        return[Weight,Cgx,Cgy,Cgz]
        
    @classmethod    
    def lg_weight(cls,dat,geo):
        
        if cls.Method=='Cessna':
            # if cls.lg_situation=='non_retractable':
            Weight_lgm = 0.013*cls.Wto + 0.362*cls.Wl**0.417*cls.Nult_land**0.950
            Weight_lgn = 6.2 + 0.0013*cls.Wto+0.007157*cls.Wl**0.749*cls.Nult_land*cls.Lsn**0.788
            Weight = Weight_lgm + Weight_lgn
            if cls.lg_situation=='retractable':
                Weight=Weight+0.014*cls.Wto
                
        
                
        Cgx_lgm=geo.Location_lg_body_right[0]+2.4
        Cgy_lgm=[geo.Location_lg_body_left[1],geo.Location_lg_body_right[1]]
        Cgz_lgm=geo.Location_lg_body_right[2]+0.16
        
        Cgx_lgn=geo.Location_lg_body_nose[0]+2.4
        Cgy_lgn=0
        Cgz_lgn=geo.Location_lg_body_nose[2]+0.16
        


        return[Weight,Weight_lgm,Weight_lgn,Cgx_lgm,Cgy_lgm,Cgz_lgm,Cgx_lgn,Cgy_lgn,Cgz_lgn]
    
     
    @classmethod    
    def engine_weight(cls,dat,geo):
        
        if cls.Method=='Cessna':
            
            Kp=1.8 #from 1.1 to 1.8
            Weight=Kp*cls.Pto*cls.Tech_factor
            
            Cgx=geo.Fuselage_length*0.075
            Cgy=0
            Cgz=0
            

        
        return[Weight,Cgx,Cgy,Cgz]
        
    @classmethod    
    def fixed_equipment_weight(cls,dat,geo):
        
        
        # set the fixed_equipment CG location to get cessna 172 required total cg location acuuratelly
        
        Wfc=0.0168*cls.Wto
        Wels=0.0268*cls.Wto
        Wiae=20+0.008*cls.Wto
        Wapi=2.5*cls.Npax
        Wox=20+0.5*cls.Npax
        Wapu=0.0085*cls.Wto
        Wfur=0.412*cls.Npax**1.145*cls.Wto**0.489
        Wpt=0.005*cls.Wto
        
        Weight=Wfc+Wels+Wiae+Wapi+Wox+Wapu+Wfur+Wpt
        
        Cgx=geo.Fuselage_length*0.15
        Cgy=0
        Cgz=-geo.Fuselage_max_height*0.1
        

        
        return[Weight,Cgx,Cgy,Cgz]
        
    @classmethod    
    def fuel_weight(cls,dat,geo):
        
        Weight_max=cls.Weight_full_fuel
        Weight_actual=cls.Weight_actual_fuel

        if cls.Fuel_location== "wings":
            
            [_,cgx_wing,cgy_wing,cgz_wing]=cls.wing_weight(dat,geo)
            
            Cgx=cgx_wing
            Cgy=cgy_wing
            Cgz=cgz_wing
            
            
        elif cls.Fuel_location== "fuselage":
            
            Cgx=geo.Fuselage_length*0.4
            Cgy=0
            Cgz=0
            

        
        return[Weight_max,Weight_actual,Cgx,Cgy,Cgz]
     
    @classmethod    
    def payload_weight(cls,dat,geo):
        
        Weight_max=cls.Weight_max_payload
        Weight_actual=cls.Weight_actual_payload
        
        Cgx=geo.Fuselage_length*0.5
        Cgy=0
        Cgz=0
        

        
        return[Weight_max,Weight_actual,Cgx,Cgy,Cgz]
        
    @classmethod    
    def crew_weight(cls,dat,geo):
        
        Weight_max=cls.Weight_max_crew_right+cls.Weight_max_crew_left
        Weight_actual=cls.Weight_actual_crew_right+cls.Weight_actual_crew_left

        
        Cgx=geo.Fuselage_length*0.5
        Cgy=[-geo.Fuselage_max_width/4,geo.Fuselage_max_width/4]
        Cgz=0
        

        
        return[Weight_max,Weight_actual,cls.Weight_actual_crew_left,cls.Weight_actual_crew_right,Cgx,Cgy,Cgz]
        
    
    @classmethod    
    def weight_module(cls,dat,geo):
        
        # Calculate total weight / Inertia / and Cg in this section
        
        cls.Wto=dat.Gross_weight*cls.conversion_kg2lbs  # init con for weight estimation iteration

        while True:
        
            cls.Definitions(dat,geo)

            [weight_wing,cgx_wing,cgy_wing,cgz_wing]=cls.wing_weight(dat,geo)
            [weight_ht,cgx_ht,cgy_ht,cgz_ht]=cls.ht_weight(dat,geo)
            [weight_vt,cgx_vt,cgy_vt,cgz_vt]=cls.vt_weight(dat,geo)
            [weight_fuselage,cgx_fuselage,cgy_fuselage,cgz_fuselage]=cls.fuselage_weight(dat,geo)
            
            [weight_lg,weight_lgm,weight_lgn,cgx_lgm,cgy_lgm,cgz_lgm,cgx_lgn,cgy_lgn,cgz_lgn]=cls.lg_weight(dat,geo)
            
            [weight_engine,cgx_engine,cgy_engine,cgz_engine]=cls.engine_weight(dat,geo)
            [weight_equipment,cgx_equipment,cgy_equipment,cgz_equipment]=cls.fixed_equipment_weight(dat,geo)
            [weight_max_fuel,weight_actual_fuel,cgx_fuel,cgy_fuel,cgz_fuel]=cls.fuel_weight(dat,geo)
            [weight_max_payload,weight_actual_payload,cgx_payload,cgy_payload,cgz_payload]=cls.payload_weight(dat,geo)
            [weight_max_crew,weight_actual_crew,weight_actual_crew_left,weight_actual_crew_right,cgx_crew,cgy_crew,cgz_crew]=cls.crew_weight(dat,geo)
    
            Empty_weight = weight_wing + weight_ht + weight_vt + weight_fuselage\
                + weight_lg + weight_engine + weight_equipment
            dat.Empty_weight =    Empty_weight
                    
            Gross_weight = Empty_weight + weight_max_fuel + weight_max_payload + weight_max_crew
            
                    
            if np.abs((cls.Wto-Gross_weight)/Gross_weight)<0.01:
                break
            else:
                cls.Wto=Gross_weight
        
        if dat.Weight_calculation == "on" : 
            Total_weight = Empty_weight + (weight_actual_fuel + weight_actual_payload + weight_actual_crew)  
            dat.Gross_weight=Gross_weight/cls.conversion_kg2lbs
        elif dat.Weight_calculation == "off" : 
            Total_weight = dat.Gross_weight*cls.conversion_kg2lbs - (weight_max_fuel + weight_max_payload + weight_max_crew) + (weight_actual_fuel + weight_actual_payload + weight_actual_crew)  
                
                
        Total_cgx = (weight_wing*cgx_wing + weight_ht*cgx_ht + weight_vt*cgx_vt + weight_fuselage*cgx_fuselage \
            + weight_lgm*cgx_lgm + weight_lgn*cgx_lgn + weight_engine*cgx_engine + weight_equipment*cgx_equipment + weight_actual_fuel*cgx_fuel\
                + weight_actual_payload*cgx_payload + weight_actual_crew*cgx_crew)/Total_weight
            
        Total_cgy = (weight_wing*0 + weight_ht*0 + weight_vt*0 + weight_fuselage*0 \
           +  weight_lgm*0 + weight_lgn*0  + weight_engine*0 + weight_equipment*0 + weight_actual_fuel*0\
                + weight_actual_payload*0 + weight_actual_crew_left*cgy_crew[0]+ weight_actual_crew_right*cgy_crew[1])/Total_weight
            
        Total_cgz = (weight_wing*cgz_wing + weight_ht*cgz_ht + weight_vt*cgz_vt + weight_fuselage*cgz_fuselage \
             + weight_lgm*cgz_lgm + weight_lgn*cgz_lgn + weight_engine*cgz_engine + weight_equipment*cgz_equipment + weight_actual_fuel*cgz_fuel\
                + weight_actual_payload*cgz_payload + weight_actual_crew*cgz_crew)/Total_weight
            
            
        #  Calculate distance btw parts and cg   # Distance are meter # Weights are pound
        
        Wing_to_cg_x=cgx_wing-Total_cgx ; HT_to_cg_x=cgx_ht-Total_cgx ; VT_to_cg_x=cgx_vt-Total_cgx
        Wing_to_cg_y=cgy_wing-Total_cgy ; HT_to_cg_y=cgy_ht-Total_cgy ; VT_to_cg_y=cgy_vt-Total_cgy
        Wing_to_cg_z=cgz_wing-Total_cgz ; HT_to_cg_z=cgz_ht-Total_cgz ; VT_to_cg_z=cgz_vt-Total_cgz
        
        Fuselage_to_cg_x=cgx_fuselage-Total_cgx ; LGM_to_cg_x=cgx_lgm-Total_cgx ; LGN_to_cg_x=cgx_lgn-Total_cgx
        Fuselage_to_cg_y=cgy_fuselage-Total_cgy ; LGM_to_cg_y=cgy_lgm-Total_cgy ; LGN_to_cg_y=cgy_lgn-Total_cgy
        Fuselage_to_cg_z=cgz_fuselage-Total_cgz ; LGM_to_cg_z=cgz_lgm-Total_cgz ; LGN_to_cg_z=cgz_lgn-Total_cgz
        
        Engine_to_cg_x=cgx_engine-Total_cgx ; Fuel_to_cg_x=cgx_fuel-Total_cgx ; Eqp_to_cg_x=cgx_equipment-Total_cgx
        Engine_to_cg_y=cgy_engine-Total_cgy ; Fuel_to_cg_y=cgy_fuel-Total_cgy ; Eqp_to_cg_y=cgy_equipment-Total_cgy
        Engine_to_cg_z=cgz_engine-Total_cgz ; Fuel_to_cg_z=cgz_fuel-Total_cgz ; Eqp_to_cg_z=cgz_equipment-Total_cgz
        
        Payload_to_cg_x=cgx_payload-Total_cgx ; Crew_to_cg_x=cgx_crew-Total_cgx 
        Payload_to_cg_y=cgy_payload-Total_cgy ; Crew_to_cg_y=cgy_crew-Total_cgy
        Payload_to_cg_z=cgz_payload-Total_cgz ; Crew_to_cg_z=cgz_crew-Total_cgz
        
        
            
        Total_Ixx = weight_wing*((Wing_to_cg_y[0])**2+(Wing_to_cg_z)**2) + weight_ht*(HT_to_cg_y[0]**2+HT_to_cg_z**2)\
                      + weight_vt*(VT_to_cg_y**2+VT_to_cg_z**2)  + weight_fuselage*(Fuselage_to_cg_y**2+Fuselage_to_cg_z**2)\
                          + weight_lgm*(LGM_to_cg_y[0]**2+LGM_to_cg_z**2) + weight_lgn*(LGN_to_cg_y**2+LGN_to_cg_z**2)  + weight_engine*(Engine_to_cg_y**2+Engine_to_cg_z**2)\
                              + weight_equipment*(Eqp_to_cg_y**2+Eqp_to_cg_z**2)  + weight_actual_fuel*(np.max(Fuel_to_cg_y)**2+Fuel_to_cg_z**2)\
                                  + weight_actual_payload*(Payload_to_cg_y**2+Payload_to_cg_z**2) + weight_actual_crew*(Crew_to_cg_y[0]**2+Crew_to_cg_z**2)
                                  
        Total_Iyy = weight_wing*((Wing_to_cg_x)**2+(Wing_to_cg_z)**2) + weight_ht*(HT_to_cg_x**2+HT_to_cg_z**2)\
                      + weight_vt*(VT_to_cg_x**2+VT_to_cg_z**2)  + weight_fuselage*(Fuselage_to_cg_x**2+Fuselage_to_cg_z**2)\
                          + weight_lgm*(LGM_to_cg_x**2+LGM_to_cg_z**2) + weight_lgn*(LGN_to_cg_x**2+LGN_to_cg_z**2)  + weight_engine*(Engine_to_cg_x**2+Engine_to_cg_z**2)\
                              + weight_equipment*(Eqp_to_cg_x**2+Eqp_to_cg_z**2)  + weight_actual_fuel*(Fuel_to_cg_x**2+Fuel_to_cg_z**2)\
                                  + weight_actual_payload*(Payload_to_cg_x**2+Payload_to_cg_z**2) + weight_actual_crew*(Crew_to_cg_x**2+Crew_to_cg_z**2)
                                  
        Total_Izz = weight_wing*((Wing_to_cg_y[0])**2+(Wing_to_cg_x)**2) + weight_ht*(HT_to_cg_y[0]**2+HT_to_cg_x**2)\
                      + weight_vt*(VT_to_cg_y**2+VT_to_cg_x**2)  + weight_fuselage*(Fuselage_to_cg_y**2+Fuselage_to_cg_x**2)\
                          + weight_lgm*(LGM_to_cg_y[0]**2+LGM_to_cg_x**2) + weight_lgn*(LGN_to_cg_y**2+LGN_to_cg_x**2)  + weight_engine*(Engine_to_cg_y**2+Engine_to_cg_x**2)\
                              + weight_equipment*(Eqp_to_cg_y**2+Eqp_to_cg_x**2)  + weight_actual_fuel*(np.max(Fuel_to_cg_y)**2+Fuel_to_cg_x**2)\
                                  + weight_actual_payload*(Payload_to_cg_y**2+Payload_to_cg_x**2) + weight_actual_crew*(Crew_to_cg_y[0]**2+Crew_to_cg_x**2)
                                 
                          
        Total_Ixz = weight_wing*((Wing_to_cg_x)*(Wing_to_cg_z)) + weight_ht*(HT_to_cg_x*HT_to_cg_z)\
                      + weight_vt*(VT_to_cg_x*VT_to_cg_z)  + weight_fuselage*(Fuselage_to_cg_x*Fuselage_to_cg_z)\
                          + weight_lgm*(LGM_to_cg_x*LGM_to_cg_z) + weight_lgn*(LGN_to_cg_x*LGN_to_cg_z)  + weight_engine*(Engine_to_cg_x*Engine_to_cg_z)\
                              + weight_equipment*(Eqp_to_cg_x*Eqp_to_cg_z)  + weight_actual_fuel*(Fuel_to_cg_x*Fuel_to_cg_z)\
                                  + weight_actual_payload*(Payload_to_cg_x*Payload_to_cg_z) + weight_actual_crew*(Crew_to_cg_x*Crew_to_cg_z)
                                  
                                  
                                                  
        Total_Ixx= 1.3 * Total_Ixx / cls.conversion_kg2lbs
        Total_Iyy= 1.3 * Total_Iyy / cls.conversion_kg2lbs
        Total_Izz= 1.3 * Total_Izz / cls.conversion_kg2lbs
        Total_Ixz= 1.3 * Total_Ixz / cls.conversion_kg2lbs
        
        dat.Mass_kg=Total_weight/cls.conversion_kg2lbs
        
        dat.x_cg=  Total_cgx      # from nose to tail   +      
        dat.y_cg=  0 #  Total_cgy      # right hand side  +         
        dat.z_cg=  Total_cgz      # UP +
        
        # dat.x_cg=  2.45     # from nose to tail   +      
        # dat.y_cg=  0      # right hand side  +         
        # dat.z_cg=  0.16      # UP +
        
        # dat.Mass_kg= 1100
        
        
        
        [dat.Ixx,dat.Iyy,dat.Izz,dat.Ixy,dat.Ixz,dat.Iyz]=np.array([Total_Ixx,Total_Iyy,Total_Izz,0,Total_Ixz,0])

        # return [x_cg,y_cg,z_cg,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]

    
	 
