import os
import numpy as np
from GEO_DEF.PARAMETRIC import Parametric
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


class geom:
    
    #########################Geometry Definition##############################
    ##############################################################################
    #All_SI_unit
    
    # =============================================================================
    #     Wing
    # =============================================================================
    
    S_ref_w=16.11 #  wing area projected over XY plane
    
    Taper_ratio_w=0.0
    Aspect_ratio_w=7.5
    LE_sweep_deg_w=0.0
    Incidence_deg_w=1.5
    Dihedral_deg_w=1.73
    Twist_deg_w=-3.0   #Twist angle at section tip
    n_w=4  # Number_of_section
    Span_percent_cumulative_w=np.array([0.1,0.55,0.99,1])

    W_LE_x=2.0  # x distance between nose and  wing leading edge at centerline
    W_LE_y=0  # y distance between nose and  wing leading edge at centerline
    W_LE_z=0.65  # z distance between nose and  wing leading edge at centerline
    
    Flap_Xhinge_root=0.67
    Flap_Xhinge_tip=0.67
    
    Aileron_Xhinge_root=0.70
    Aileron_Xhinge_tip=0.70
    
    
        
    # if Taper manuel="on"  then "Taper_ratio_arr_w" will be used instead of "Taper_ratio_w"
    # İf you want to define taper ratio array manually you can activate following line
    #################################################################################
    Taper_manuel_w="on"
    Taper_ratio_arr_w=np.array([1,1,0.675,0.6])
    #################################################################################
    # However,to check, you should be sure that "multiplication of all partial taper ratio's should give total taper ratio"
    
    # =============================================================================
    #     HT
    # =============================================================================
    
    S_ref_ht=3.59 #  wing area projected over XY plane
    Taper_ratio_ht=0.62
    Aspect_ratio_ht=3.13
    LE_sweep_deg_ht=10
    Incidence_deg_ht=0
    Dihedral_deg_ht=0
    Twist_deg_ht=0   #Twist angle at section tip
    n_ht=2  # Number_of_section
    Span_percent_cumulative_ht=np.array([0.90,1]) #elevator is up to %80 of span
    

    HT_LE_x=6.41  # x distance between nose and  wing leading edge at centerline
    HT_LE_y=0  # y distance between nose and  wing leading edge at centerline
    HT_LE_z=0.21  # z distance between nose and  wing leading edge at centerline
    
    Elevator_Xhinge_root=0.60
    Elevator_Xhinge_tip=0.60
    
    # if Taper manuel="on"  then "Taper_ratio_arr_w" will be used instead of "Taper_ratio_w"
    # İf you want to define taper ratio array manually you can activate following line
    #################################################################################
    Taper_manuel_ht="off"
    Taper_ratio_arr_ht=np.array([0.0,0.0,0.0,0.0])
    #################################################################################
    # However,to check, you should be sure that "multiplication of all partial taper ratio's should give total taper ratio"
    
    # =============================================================================
    #     VT
    # =============================================================================
    
    S_ref_vt=1.71 
    S_ref_vt=S_ref_vt*2 #vertical tail span defination correction
    Taper_ratio_vt=0.52
    Aspect_ratio_vt=1.32
    Aspect_ratio_vt=Aspect_ratio_vt*2 #vertical tail span defination correction
    LE_sweep_deg_vt=42
    Incidence_deg_vt=0
    Dihedral_deg_vt=0
    Twist_deg_vt=0   #Twist angle at section tip
    n_vt=3  # Number_of_section
    Span_percent_cumulative_vt=np.array([0.1,0.9,1])
    

    VT_LE_x=6.15  # x distance between nose and  wing leading edge at centerline
    VT_LE_y=0  # y distance between nose and  wing leading edge at centerline
    VT_LE_z=0.085  # z distance between nose and  wing leading edge at centerline
    
    Rudder_Xhinge_root=0.60
    Rudder_Xhinge_tip=0.60
    
    # if Taper manuel="on"  then "Taper_ratio_arr_w" will be used instead of "Taper_ratio_w"
    # İf you want to define taper ratio array manually you can activate following line
    #################################################################################
    Taper_manuel_vt="off"
    Taper_ratio_arr_vt=np.array([0.0,0.0,0.0,0.0])
    #################################################################################
    # However,to check, you should be sure that "multiplication of all partial taper ratio's should give total taper ratio"
    

    
    # =============================================================================
    #     Airfoils
    # =============================================================================
    
    datcom_wing_airfoil="2412"
    datcom_ht_airfoil="0012"
    datcom_vt_airfoil="0009"
    
    wing_section1_airfoil="NACA2412.dat"
    wing_section2_airfoil="NACA2412.dat"
    wing_section3_airfoil="NACA2412.dat"
    wing_section4_airfoil="NACA0012.dat"
    wing_section5_airfoil="NACA0012.dat"
    
    ht_section1_airfoil="NACA0012.dat"
    ht_section2_airfoil="NACA0012.dat"
    
    vt_section1_airfoil="NACA0009.dat"
    vt_section2_airfoil="NACA0009.dat"
    
    # =============================================================================
    #     Fuselage
    # =============================================================================

    Fuselage_length=7.3 #meters from nose to most aft fuselage point
    Fuselage_max_width=1.22
    Fuselage_max_height=1.22 # meters from bottom surface to top surface of fuselage
    
    # =============================================================================
    #     Landing Gear
    # =============================================================================
    
    # Location_lg_body_right=np.asarray([-0.4,1.3,1.36])+np.asarray([2.40,0,0.16])# Location of tip of the landing gear with respect to datum point in body axis
    # Location_lg_body_left=np.asarray([-0.4,-1.3,1.36])+np.asarray([2.40,0,0.16])
    # Location_lg_body_nose=np.asarray([1.2,0,1.425])+np.asarray([2.40,0,0.16])
    
    Location_lg_body_right=np.asarray([-0.4,1.3,1.36])# Location of tip of the landing gear with respect to cg point in (body axis?)
    Location_lg_body_left=np.asarray([-0.4,-1.3,1.36])
    Location_lg_body_nose=np.asarray([1.2,0,1.425])
    
    # =============================================================================
    #     Engine
    # =============================================================================
    
    Propeller_efficiency=0.7  #at max'mum speed cond't'on
    Number_of_blade=2
    Max_engine_hp=160 #hp   SPEC

    
    @classmethod
    def file_reader(cls,Input_file):  

        file=open(Input_file,"r")
        nth_line=[]
        local_list=[]
        global_list_line=[""]
        global_list_all=[""]
        file.seek(0)
        for k in range(1,6):
         	nth_line=(file.readline())
         	local_list=nth_line.split(" ")
         	length=len(local_list)
         	space_number=local_list.count("")
         	i=0
         	while i<(length-space_number):
                 if local_list[i]==(""):
                     local_list.pop(i)
                 else:
                     i += 1
         	local_list_end=local_list[length-space_number-1]
         	local_list_end=local_list_end.strip("\n")
         	local_list.pop(length-space_number-1)
         	local_list.append(local_list_end)
            
         	global_list_line.append(local_list) #for list in list as lines
         	global_list_all.extend(local_list) # for just one one list
         	try:
                 global_list_line.remove("")
                 global_list_all.remove("")
         	except ValueError:
                 pass
        file.close()
        return global_list_all
    
    @classmethod
    def calculate_geo(cls):
        
        cls.LE_sweep_deg_arr_w=np.array([cls.LE_sweep_deg_w,cls.LE_sweep_deg_w,3,3])
        cls.Dihedral_deg_arr_w=np.array([cls.Dihedral_deg_w,cls.Dihedral_deg_w,cls.Dihedral_deg_w,cls.Dihedral_deg_w])
        cls.Twist_deg_arr_w=np.array([0,0,cls.Twist_deg_w/2,cls.Twist_deg_w,cls.Twist_deg_w])

        
        cls.LE_sweep_deg_arr_ht=np.array([cls.LE_sweep_deg_ht,cls.LE_sweep_deg_ht])
        cls.Dihedral_deg_arr_ht=np.array([cls.Dihedral_deg_ht,cls.Dihedral_deg_ht])
        cls.Twist_deg_arr_ht=np.array([cls.Twist_deg_ht,cls.Twist_deg_ht,cls.Twist_deg_ht])

        
        cls.LE_sweep_deg_arr_vt=np.array([cls.LE_sweep_deg_vt,cls.LE_sweep_deg_vt,cls.LE_sweep_deg_vt])
        cls.Dihedral_deg_arr_vt=np.array([cls.Dihedral_deg_vt,cls.Dihedral_deg_vt,cls.Dihedral_deg_vt])
        cls.Twist_deg_arr_vt=np.array([cls.Twist_deg_vt,cls.Twist_deg_vt,cls.Twist_deg_vt,cls.Twist_deg_vt])

        
        [cls.Xe_w,cls.Ye_w,cls.Ze_w,cls.C_w,cls.Twist_deg_arr_w,cls.MAC_tot_w,cls.Span_arr_w,cls.Span_w,cls.Taper_ratio_arr_w]\
            =Parametric.parametric(cls.S_ref_w,cls.Taper_ratio_w,cls.Aspect_ratio_w,cls.LE_sweep_deg_w,cls.Incidence_deg_w,\
                                  cls.Dihedral_deg_w,cls.Twist_deg_w,cls.n_w,cls.Span_percent_cumulative_w,cls.LE_sweep_deg_arr_w,\
                                      cls.Dihedral_deg_arr_w,cls.Twist_deg_arr_w,cls.Taper_ratio_arr_w,cls.Taper_manuel_w)
        
        [cls.Xe_ht,cls.Ye_ht,cls.Ze_ht,cls.C_ht,cls.Twist_deg_arr_ht,cls.MAC_tot_ht,cls.Span_arr_ht,cls.Span_ht,cls.Taper_ratio_arr_ht]\
            =Parametric.parametric(cls.S_ref_ht,cls.Taper_ratio_ht,cls.Aspect_ratio_ht,cls.LE_sweep_deg_ht,cls.Incidence_deg_ht,\
                                  cls.Dihedral_deg_ht,cls.Twist_deg_ht,cls.n_ht,cls.Span_percent_cumulative_ht,cls.LE_sweep_deg_arr_ht,\
                                      cls.Dihedral_deg_arr_ht,cls.Twist_deg_arr_ht,cls.Taper_ratio_arr_ht,cls.Taper_manuel_ht)
        
        [cls.Xe_vt,cls.Ze_vt,cls.Ye_vt,cls.C_vt,cls.Twist_deg_arr_vt,cls.MAC_tot_vt,cls.Span_arr_vt,cls.Span_vt,cls.Taper_ratio_arr_vt]\
            =Parametric.parametric(cls.S_ref_vt,cls.Taper_ratio_vt,cls.Aspect_ratio_vt,cls.LE_sweep_deg_vt,cls.Incidence_deg_vt,\
                                  cls.Dihedral_deg_vt,cls.Twist_deg_vt,cls.n_vt,cls.Span_percent_cumulative_vt,cls.LE_sweep_deg_arr_vt,\
                                      cls.Dihedral_deg_arr_vt,cls.Twist_deg_arr_vt,cls.Taper_ratio_arr_vt,cls.Taper_manuel_vt)
        if cls.Number_of_blade==2:
            Kp=20.4  #m
        elif cls.Number_of_blade==3:
            Kp=19.2  #m
        else:
            Kp=18.0  #m
        cls.propeller_diameter=(Kp*cls.Max_engine_hp**0.25)*0.0254
        cls.hub_diameter=cls.propeller_diameter*0.15   #m
    
    @classmethod            
    def calculate_Datcom(cls,dat):
        
        print ("\nDatcom is working..")
        """
        cwd = os.getcwd()
        os.chdir(cwd+'\AERODYNAMICS\Datcom_CD0')
        
        file=open('Aircraft_Datcom.dat','w')


        file.write("CASEID Aircraft\nDERIV RAD\nDIM M\nTRIM\nDAMP\n")

        file.write(" $FLTCON NMACH=1.0,MACH=%.2f,NALPHA=20.0,WT=%.2f,\n" % (0.2,750))   
        file.write("  ALPHA=-5.0,-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 5.0,\n  9.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0,\n" )    
        file.write("  NALT=1.0,ALT=%.1f$\n" % (1000))    
        file.write(" $OPTINS SREF=%.2f,BLREF=%.2f$\n" % (cls.S_ref_w,cls.Span_w))    
        file.write(" $SYNTHS XCG=%.2f,ZCG=%.2f,XW=%.2f,ZW=%.2f,\n" % (dat.x_cg,dat.z_cg,cls.W_LE_x,cls.W_LE_z))    
        file.write("  XH=%.2f,ZH=%.2f,\n" % (cls.HT_LE_x,cls.HT_LE_z))    
        file.write("  XV=%.2f,ZV=%.2f,\n" % (cls.VT_LE_x,cls.VT_LE_z))    
        file.write("  ALIW=%.1f,ALIH=%.1f,SCALE=1.0$\n" % (cls.Incidence_deg_w,cls.Incidence_deg_ht))    
        
        file.write(" $BODY NX=9.0, ITYPE=1.0, METHOD=1.0,\n")    
        file.write("  X=0.41,0.42,0.56,1.58,2.09,3.82,4.41,%.2f,%.2f,\n" % (cls.Fuselage_length,cls.Fuselage_length*1.015))    
        file.write("  S=0.00,0.08,0.57,1.18,%.2f,1.35,0.85,0.04,0.00,\n" % ((cls.Fuselage_max_width*cls.Fuselage_max_height)*1.5))    
        file.write("  ZU=0.00,0.14,0.25,0.38,%.2f,0.62,0.39,0.28,0.28,\n" % ((cls.Fuselage_max_height/1.22)*.8)) 
        file.write("  ZL=0.00,-0.14,-0.45,%.2f,-0.77,-0.58,-0.49,-0.08,-0.08$\n" % ((cls.Fuselage_max_height/1.22)*(-0.72)))  
          
        file.write("NACA-W-4-%s\n" % (cls.datcom_wing_airfoil))    
        file.write(" $WGPLNF CHRDTP=%.2f,CHRDR=%.2f,SSPN=%.2f,SSPNE=%.2f,\n" % (cls.C_w[-1],cls.C_w[0],cls.Span_w/2,cls.Span_w/2-cls.Fuselage_max_width/2))    
        file.write("  SAVSI=%.1f,CHSTAT=0.25,TWISTA=%.1f,DHDADI=%.1f,TYPE=1.0$\n" % (cls.LE_sweep_deg_w,cls.Twist_deg_w,cls.Dihedral_deg_w))   
        
        file.write("NACA-H-4-%s\n" % (cls.datcom_ht_airfoil))    
        file.write(" $HTPLNF CHRDTP=%.2f,CHRDR=%.2f,SSPN=%.2f,SSPNE=%.2f,\n" % (cls.C_ht[-1],cls.C_ht[0],cls.Span_ht/2,cls.Span_ht/2-cls.Fuselage_max_width/20))  
        file.write("  SAVSI=%.1f,CHSTAT=0.0,TWISTA=%.1f,DHDADI=%.1f,TYPE=1.0$\n" % (cls.LE_sweep_deg_ht,cls.Twist_deg_ht,cls.Dihedral_deg_ht))    

        file.write("NACA-V-4-%s\n" % (cls.datcom_vt_airfoil))    
        file.write(" $VTPLNF CHRDTP=%.2f,CHRDR=%.2f,SSPN=%.2f,SSPNE=%.2f,\n" % (cls.C_vt[-1],cls.C_vt[0],cls.Span_vt/2,cls.Span_vt/2-cls.Fuselage_max_width/20))    
        file.write("  SAVSI=%.1f,CHSTAT=0.0,TWISTA=%.1f,DHDADI=%.1f,TYPE=1.0$\n" % (cls.LE_sweep_deg_vt,cls.Twist_deg_vt,cls.Dihedral_deg_vt))   
        file.write("NEXT CASE")
  
        file.close()
        
        os.system("echo | executableDatcomTool.exe")
        AllData=cls.file_reader("DatcomOut.txt")
        CD=AllData[AllData.index("CD")+2]
        CD=CD.split(",")
        CD_arr=[]
        for iCD in CD:
            try:
                CD_arr.append(float(iCD))
            except:
                pass     
        CL=AllData[AllData.index("CL")+2]
        CL=CL.split(",")
        CL_arr=[]
        for iCL in CL:
            try:
                CL_arr.append(float(iCL))
            except:
                pass   
        AOA=AllData[AllData.index("AOA")+2]
        AOA=AOA.split(",")
        AOA_arr=[]
        for iAOA in AOA:
            try:
                AOA_arr.append(float(iAOA))
            except:
                pass   
            
        def func(x, a, b, c, d, e, f, g, h, i):
            return a * x**8 + b * x ** 7 + c * x ** 6  + d * x ** 5 + e * x** 4 + f * x ** 3 + g * x ** 2  + h * x + i
        
        AOA_arr=np.asarray(AOA_arr)
        CL_arr=np.asarray(CL_arr)
        CD_arr=np.asarray(CD_arr)
    
        (CL_popt, _) = curve_fit(func, AOA_arr, CL_arr)# bounds=(0, [3., 1., 0.5]))
        (CD_popt, _) = curve_fit(func, AOA_arr, CD_arr)# bounds=(0, [3., 1., 0.5]))
        
        AOA_arr_fitted=np.arange(-5,25,0.01)
        
        cls.CD0=np.min(func(AOA_arr_fitted, *CD_popt))*1.5
        cls.CL_max=np.max(func(AOA_arr_fitted, *CL_popt))
        cls.AOA_max=AOA_arr_fitted[np.where(cls.CL_max==func(np.arange(-5,25,0.01), *CL_popt))[0][0]]

        # plt.plot(AOA_arr, CL_arr)
        # plt.plot(AOA_arr, func(AOA_arr, *CL_popt), 'g--')
        # plt.minorticks_on()
        # plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        # plt.show()
            
        os.chdir(cwd)
        """
        cls.CD0=0.025
        cls.CL_max=1.6
        cls.AOA_max=17



    
    
    
    
        

