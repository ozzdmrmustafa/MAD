
import numpy as np

from WEIGHT_BALANCE.WEIGHTMODULE import WeightModule
from TRIM_SIM.TRANS_MATRIX import Transformation_Matrices
from ATMOSPHERE.ATM import Atm
import os

# Avl.avl

class Avl:
    
    def avl(dat,geo):
    
        WeightModule.weight_module(dat,geo)    
        Atm.atm(dat)
        
        ##############################################################################
        
        ##############################################################################
        #Flight Conditions#
        
        
        # V_inf    =66.75  #used for calculation of nondimentional P Q R calculation
        # Mach     =0.2   #used in aerodynamic coefficient calculation for compresibility effect
        # Aoa    =0
        # Beta     =0
        # P        =0.00   # P Q R radyan
        # Q        =0.00
        # R        =0.00
        # TEF     =0
        # FlaperonL  =0
        # HTL =0
        # Rudder   =0
                     
        ################################################################################
        ################################################################################
        ################################################################################
        
        
        dat.V_inf    =np.clip(dat.V_inf,1,300)
        dat.Mach     =np.clip(dat.Mach,0,0.9)
        dat.Aoa    =np.clip(dat.Aoa,min(geo.Aoa_Limits),max(geo.Aoa_Limits))
        dat.Beta     =np.clip(dat.Beta,min(geo.Beta_Limits),max(geo.Beta_Limits))
        dat.TEF     =np.clip(dat.TEF ,min(geo.TEF_Limits),max(geo.TEF_Limits))  
        dat.FlaperonL  =np.clip(dat.FlaperonL ,min(geo.Flaperon_Limits),max(geo.Flaperon_Limits))
        dat.HTL =np.clip(dat.HTL,min(geo.HTL_Limits),max(geo.HTL_Limits))
        dat.Rudder   =np.clip(dat.Rudder ,min(geo.Rudder_Limits),max(geo.Rudder_Limits))
    
        
        Ang_Rates_Stab=np.dot((Transformation_Matrices.stability_to_body(dat)).T,[dat.P,dat.Q,dat.R])
        dat.P_stab=Ang_Rates_Stab[0]
        dat.Q_stab=Ang_Rates_Stab[1]
        dat.R_stab=Ang_Rates_Stab[2]
        
        dat.nondim_p= dat.P_stab*geo.Span_w/(2*dat.V_inf)
        dat.nondim_q= dat.Q_stab*geo.MAC_tot_w/(2*dat.V_inf)
        dat.nondim_r= dat.R_stab*geo.Span_w/(2*dat.V_inf)
        
        # dat.nondim_p= dat.P*Span_w/(2*dat.V_inf)
        # dat.nondim_q= dat.Q*MAC_tot_w/(2*dat.V_inf)
        # dat.nondim_r= dat.R*Span_w/(2*dat.V_inf)
        
        
        #AVL dynamic coefficient calculation reliability limits
        """
        nondim_p=np.clip(nondim_p,-0.10,0.10)
        nondim_q=np.clip(nondim_q,-0.03,0.03) 
        nondim_r=np.clip(nondim_r,-0.25,0.25) 
        """
        
        ###################################################################################
        ###################################################################################
        
        
        
        #Write DAta toAVL
        cwd = os.getcwd()
        os.chdir(cwd+'\AERODYNAMICS')
        
        file=open("vla.avl","w")
        
        #input are in type of str or float
        
        CASE_ID="VLA"
        
        
        
        
        file.write("{}\n".format(CASE_ID,str))
        file.write("{}                      !Mach\n".format(dat.Mach,str))
        file.write("0   0   0            !iYsym  iZsym  Zsym\n") # For the activate ground effect iZsym should be 1. 
        #Also Zsym defines location of ground #(-9=defines ground is located 9 meters below the aircraft 
        #ground does not effect the aerodynamic coefficients if the aircraft is located to more than one span length above the ground)
        file.write("{}  {}  {}              !Sref   Cref   Bref   reference area, chord, span\n".format(geo.S_ref_w,geo.MAC_tot_w,geo.Span_w,str))
        file.write("{}  {}  {}              !Xref   Yref   Zref   moment reference location (arb.)\n".format(dat.x_cg,dat.y_cg,dat.z_cg,str))
        file.write("{}                      !CDp\n".format(geo.CD0,str))
        
        
        
        file.write("#=========================================================================\n")
        file.write("#===============================WING======================================\n")
        file.write("#=========================================================================\n")
        
        
        file.write("SURFACE\n")
        file.write("WING\n")
        file.write("{}  {}  {}  {}  !  Nchord   Cspace   Nspan  Sspace\n".format(10  , 1  , 15 , -2,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("YDUPLICATE\n")
        file.write("# reflect image wing about y=0 plane\n")
        file.write(" 0 \n")
        file.write("#--------------------------------------------------------------\n")
        
        file.write("ANGLE\n")
        file.write("# incidence angle bias for whole surface\n")
        file.write("   {}    \n".format(geo.Incidence_deg_w,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("TRANSLATE\n")
        file.write("# x,y,z bias for whole surface\n")
        file.write("{}     {}      {}     \n".format(geo.W_LE_x,geo.W_LE_y,geo.W_LE_z,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_w[0],geo.Ye_w[0],geo.Ze_w[0],geo.C_w[0],geo.Twist_deg_arr_w[0],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.wing_section1_airfoil,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_w[1],geo.Ye_w[1],geo.Ze_w[1],geo.C_w[1],geo.Twist_deg_arr_w[1],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.wing_section2_airfoil,str))
        file.write("#--------------------------------------------------------------\n")
        
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("flap           {}    {}       {}            {}  \n".format(1,geo.Flap_Xhinge_root,"0 0 0",1,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_w[2],geo.Ye_w[2],geo.Ze_w[2],geo.C_w[2],geo.Twist_deg_arr_w[2],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.wing_section3_airfoil,str))
        file.write("#--------------------------------------------------------------\n")
        
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("flap           {}    {}       {}            {}  \n".format(1,geo.Flap_Xhinge_tip,"0 0 0",1,str))
        file.write("#--------------------------------------------------------------\n")
        
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("aileron          {}    {}       {}            {}  \n".format(1,geo.Aileron_Xhinge_root,"0 0 0",-1,str))
        file.write("#--------------------------------------------------------------\n")
        
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_w[3],geo.Ye_w[3],geo.Ze_w[3],geo.C_w[3],geo.Twist_deg_arr_w[3],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.wing_section4_airfoil,str))
        file.write("#\n")
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("aileron          {}    {}       {}            {}  \n".format(1,geo.Aileron_Xhinge_tip,"0 0 0",-1,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_w[4],geo.Ye_w[4],geo.Ze_w[4],geo.C_w[4],geo.Twist_deg_arr_w[4],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.wing_section5_airfoil,str))
        file.write("#\n")
        
        file.write("#=========================================================================\n")
        file.write("#===============================HORIZONTAL TAIL===========================\n")
        file.write("#=========================================================================\n")
        file.write("#\n")
        
        
        file.write("SURFACE\n")
        file.write("Horizontal Tail\n")
        file.write("{}  {}  {}  {}  !  Nchord   Cspace   Nspan  Sspace\n".format(5  , 1  , 10 , -2 ,str))
        file.write("#---------------------------------------------------------------\n")
        
        file.write("#\n")
        file.write("YDUPLICATE\n")
        file.write("0\n")
        
        file.write("ANGLE\n")
        file.write("# incidence angle bias for whole surface\n")
        file.write("   {}    \n".format(geo.Incidence_deg_ht,str))
        file.write("#---------------------------------------------------------------\n")
        
        file.write("TRANSLATE\n")
        file.write("# x,y,z bias for whole surface\n")
        file.write("{}     {}      {}     \n".format(geo.HT_LE_x,geo.HT_LE_y,geo.HT_LE_z,str))
        file.write("#--------------------------------------------------------------\n")
        
        
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_ht[0],geo.Ye_ht[0],geo.Ze_ht[0],geo.C_ht[0],geo.Twist_deg_arr_ht[0],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.ht_section1_airfoil,str))
        file.write("#---------------------------------------------------------------\n")
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("elevator          {}    {}       {}            {}  \n".format(1,geo.Elevator_Xhinge_root,"0 0 0",1,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_ht[1],geo.Ye_ht[1],geo.Ze_ht[1],geo.C_ht[1],geo.Twist_deg_arr_ht[1],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.ht_section2_airfoil,str))
        file.write("#\n")
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("elevator          {}    {}       {}            {}  \n".format(1,geo.Elevator_Xhinge_tip,"0 0 0",1,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_ht[2],geo.Ye_ht[2],geo.Ze_ht[2],geo.C_ht[2],geo.Twist_deg_arr_ht[2],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.ht_section2_airfoil,str))
        file.write("#\n")
        
        file.write("#\n")
        
        file.write("#=========================================================================\n")
        file.write("#===============================VERTICAL TAIL=============================\n")
        file.write("#=========================================================================\n")
        file.write("#\n")
        
        file.write("SURFACE\n")
        file.write("Vertical Tail\n")
        file.write("{}  {}  {}  {}  !  Nchord   Cspace   Nspan  Sspace\n".format(5  , 1  , 10 , 1,str))
        file.write("#---------------------------------------------------------------\n")
                 
        
        file.write("TRANSLATE\n")
        file.write("# x,y,z bias for whole surface\n")
        file.write("{}     {}      {}     \n".format(geo.VT_LE_x,geo.VT_LE_y,geo.VT_LE_z,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_vt[3],geo.Ye_vt[3],geo.Ze_vt[3],geo.C_vt[3],geo.Twist_deg_arr_vt[3],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.vt_section2_airfoil,str))
        file.write("#---------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_vt[2],geo.Ye_vt[2],geo.Ze_vt[2],geo.C_vt[2],geo.Twist_deg_arr_vt[2],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.vt_section2_airfoil,str))
        file.write("#---------------------------------------------------------------\n")
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("rudder          {}    {}       {}            {}  \n".format(1,geo.Rudder_Xhinge_tip,"0 0 0",1,str))
        file.write("#--------------------------------------------------------------\n")
        
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_vt[1],geo.Ye_vt[1],geo.Ze_vt[1],geo.C_vt[1],geo.Twist_deg_arr_vt[1],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.vt_section1_airfoil,str))
        file.write("#---------------------------------------------------------------\n")
        
        file.write("CONTROL\n")
        file.write("#Cname      Cgain  Xhinge  HingeVec        SgnDup\n")
        file.write("rudder          {}    {}       {}            {}  \n".format(1,geo.Rudder_Xhinge_root,"0 0 0",1,str))
        file.write("#--------------------------------------------------------------\n")
        
        file.write("SECTION\n")
        file.write("#    Xle       Yle         Zle         chord     angle   \n")
        file.write("     {}          {}           {}           {}         {}         \n".format(geo.Xe_vt[0],geo.Ye_vt[0],geo.Ze_vt[0],geo.C_vt[0],geo.Twist_deg_arr_vt[0],str))
        
        file.write("AFIL\n")
        file.write("{}\n".format(geo.vt_section1_airfoil,str))
        file.write("#---------------------------------------------------------------\n")
        
        
        file.write("#==============================================================\n")
        
        if geo.Fuselage_effect=="on" :
            #-------------------------------------------------------------
            #--------------------------Fuselage---------------------------
            #-------------------------------------------------------------
            file.write("SURFACE \n")
            file.write("Fuselage H \n")
            file.write("#Nchordwise  Cspace   Nspanwise  Sspace \n")
            file.write("24           1.0  \n")
            file.write("COMPONENT \n")
            file.write("1 \n")
            file.write("YDUPLICATE  \n")
            file.write("0.0 \n")
            file.write("SCALE \n")
            file.write("1.0   1.0  1.0 \n")
            file.write("TRANSLATE \n")
            file.write("0.0   0.0   0.0 \n")
            file.write("SECTION \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.0   0.0    0.0       {}   0.    1          0. \n".format(geo.Fuselage_length,str))
            file.write("SECTION \n")
            file.write("#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.11   0.26   0.0      {}   0.    1          0. \n".format(geo.Fuselage_length,str))
            file.write("SECTION \n")
            file.write("#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.83   0.45   0.0      2.99   0.    1          0. \n")
            file.write("SECTION \n")
            file.write("#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 1.42   {}   0.0      1.61   0.    1          0. \n".format(geo.Fuselage_max_width/2,str))
            file.write("#-------------------------------------------------- \n")
            file.write("SURFACE \n")
            file.write("Fuselage V Bottom \n")
            file.write("#Nchordwise  Cspace   Nspanwise  Sspace \n")
            file.write("24           1.0  \n")
            file.write("NOWAKE \n")
            file.write("COMPONENT \n")
            file.write("1 \n")
            file.write("SCALE \n")
            file.write("1.0   1.0  1.0 \n")
            file.write("TRANSLATE \n")
            file.write("0.0   0.0   0.0 \n")
            file.write("SECTION \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 1.75  0.0    {}        0.42   0.    1          0. \n".format(-geo.Fuselage_max_height/2,str))
            file.write("SECTION		   \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.59  0.0   -0.52      2.71   0.    1          0.  \n")
            file.write("SECTION		 \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.13  0.0   -0.30      4.84   0.    1          0. \n")
            file.write("SECTION \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.0   0.0    0.0        {}   0.    1          0. \n".format(geo.Fuselage_length,str))
            file.write("#-------------------------------------------------- \n")
            file.write("SURFACE \n")
            file.write("Fuselage V Top \n")
            file.write("#Nchordwise  Cspace   Nspanwise  Sspace \n")
            file.write("24           1.0  \n")
            file.write("COMPONENT \n")
            file.write("1 \n")
            file.write("SCALE \n")
            file.write("1.0   1.0  1.0 \n")
            file.write("TRANSLATE \n")
            file.write("0.0   0.0   0.0 \n")
            file.write("SECTION	 \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.00  0.0    0.0       {}   0.    1          0. \n".format(geo.Fuselage_length,str))
            file.write("SECTION		 \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 0.09  0.0    0.11      {}   0.    1          0. \n".format(geo.Fuselage_length,str))
            file.write("SECTION		 \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 1.24  0.0    0.27      4.15   0.    1          0. \n")
            file.write("SECTION		      \n")
            file.write("#Xle   Yle    Zle      Chord   Ainc  Nspanwise  Sspace \n")
            file.write(" 2.18  0.0    {}      0.60   0.    1          0. \n".format(geo.Fuselage_max_height/2,str))
            file.write("============================================================== \n")
            
            
        file.close()
        
        ###############################################################################
        
        #RUN FILE CREATION
        
        file=open("vla.run","w")
        
        
        file.write("---------------------------------------------\n")
        file.write("Run case  1: - Cruise Condition -\n")                               
        
        file.write("alpha        ->  alpha       =   {}  \n".format(dat.Aoa,str))    
        file.write("beta         ->  beta        =   {}  \n".format(dat.Beta,str))     
        file.write("pb/2V        ->  pb/2V       =   {}  \n".format(dat.nondim_p,str))    
        file.write("qc/2V        ->  qc/2V       =   {}  \n".format(dat.nondim_q,str))     
        file.write("rb/2V        ->  rb/2V       =   {}  \n".format(dat.nondim_r,str))     
        file.write("flap         ->  flap        =   {}  \n".format(dat.TEF,str))     
        file.write("aileron      ->  aileron     =   {}  \n".format(dat.FlaperonL,str))     
        file.write("elevator     ->  elevator    =   {}  \n".format(dat.HTL,str))     
        file.write("rudder       ->  rudder      =   {}  \n".format(dat.Rudder,str))     
                                                                                  
        file.write("CDo       =   {}  \n".format(geo.CD0,str))                                      
                                                         
        file.write("X_cg      =   {}    Lunit    \n".format(dat.x_cg,str))                        
        file.write("Y_cg      =   {}    Lunit    \n".format(dat.y_cg,str))                         
        file.write("Z_cg      =   {}    Lunit    \n".format(dat.z_cg,str))   
        file.write("velocity  =   {}    Lunit    \n".format(dat.V_inf,str))  
        file.write("Mach      =   {}    Lunit    \n".format(dat.Mach,str))  # Sadece comprasibility effect için kullanılıyor
        #file.write("density   =   {}    Lunit    \n".format(dat.rho,str)) #  Coefficientları değiştirmior
        
        file.close()
                
                
        
        ###############################################################################
        
        ################################################################################
        
        ################################################################################
        #import os
        import subprocess
        
        ################################################################################
        #you can use  one the 4 methods to run the avl
        ################################################################################
        
        #os.system("(echo oper && echo x && echo st aerodynamic_derivatives.txt && echo o )| avl.exe vla.avl >logfile.txt")
        
        #sb=subprocess.Popen('(echo oper && echo x && echo st aerodynamic_derivatives.txt && echo o )| avl.exe vla.avl >logfile.txt', shell=True)
        #sb.wait()
        
        subprocess.run('(echo oper && echo x && echo st aerodynamic_derivatives.txt && echo o )| avl.exe vla.avl >logfile.txt', shell=True)
        
        #subprocess.call('(echo oper && echo x && echo st aerodynamic_derivatives.txt && echo o )| avl.exe vla.avl >logfile.txt', shell=True)
        
        
        ##############################################################################
        
        ##############################################################################
        
        #getDataFromAVL
        
        
        file=open("aerodynamic_derivatives.txt","r")
        
        
        nth_line=[]
        local_list=[]
        global_list_line=[""]
        global_list_all=[""]
        file.seek(0)
        
        for k in range(1,69):
        
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
        
        os.chdir(cwd)
        
        Alpha=float(global_list_all[global_list_all.index("Alpha")+2])
        Beta=float(global_list_all[global_list_all.index("Beta")+2])
        dat.Mach=float(global_list_all[global_list_all.index("Mach")+2])
        ##########Body Axis Coefficients################
        dat.CXtot=float(global_list_all[global_list_all.index("CXtot")+2])
        dat.CYtot=float(global_list_all[global_list_all.index("CYtot")+2])
        dat.CZtot=float(global_list_all[global_list_all.index("CZtot")+2])
        dat.Cltot=float(global_list_all[global_list_all.index("Cltot")+2])
        dat.Cmtot=float(global_list_all[global_list_all.index("Cmtot")+2])
        dat.Cntot=float(global_list_all[global_list_all.index("Cntot")+2])
        ################################################
        dat.CLtot=float(global_list_all[global_list_all.index("CLtot")+2])
        dat.CDtot=float(global_list_all[global_list_all.index("CDtot")+2])
        dat.CDvis=float(global_list_all[global_list_all.index("CDvis")+2])
        
        dat.e=float(global_list_line[27][5])
        dat.Xnp=float(global_list_all[global_list_all.index("Xnp")+2])
        dat.Static_margin=(dat.Xnp-dat.x_cg)/geo.MAC_tot_w
        
        dat.CLa=float(global_list_all[global_list_all.index("CLa")+2])
        dat.CYa=float(global_list_all[global_list_all.index("CYa")+2])
        dat.Cla=float(global_list_all[global_list_all.index("Cla")+2])
        dat.Cma=float(global_list_all[global_list_all.index("Cma")+2])
        
        dat.Cna=float(global_list_all[global_list_all.index("Cna")+2])
        dat.CLb=float(global_list_all[global_list_all.index("CLb")+2])
        dat.CYb=float(global_list_all[global_list_all.index("CYb")+2])
        dat.Clb=float(global_list_all[global_list_all.index("Clb")+2])
        dat.Cmb=float(global_list_all[global_list_all.index("Cmb")+2])
        dat.Cnb=float(global_list_all[global_list_all.index("Cnb")+2])
        
        dat.CLp=global_list_all[global_list_all.index("CLp")+2]
        dat.CYp=global_list_all[global_list_all.index("CYp")+2]
        dat.Clp=global_list_all[global_list_all.index("Clp")+2]
        dat.Cmp=global_list_all[global_list_all.index("Cmp")+2]
        dat.Cnp=global_list_all[global_list_all.index("Cnp")+2]
        
        dat.CLq=global_list_all[global_list_all.index("CLq")+2]
        dat.CYq=global_list_all[global_list_all.index("CYq")+2]
        dat.Clq=global_list_all[global_list_all.index("Clq")+2]
        dat.Cmq=global_list_all[global_list_all.index("Cmq")+2]
        dat.Cnq=global_list_all[global_list_all.index("Cnq")+2]
        
        dat.CLr=global_list_all[global_list_all.index("CLr")+2]
        dat.CYr=global_list_all[global_list_all.index("CYr")+2]
        dat.Clr=global_list_all[global_list_all.index("Clr")+2]
        dat.Cmr=global_list_all[global_list_all.index("Cmr")+2]
        dat.Cnr=global_list_all[global_list_all.index("Cnr")+2]
        
        #Flap
        dat.CLd1=global_list_all[global_list_all.index("CLd1")+2]
        dat.CYd1=global_list_all[global_list_all.index("CYd1")+2]
        dat.Cld1=global_list_all[global_list_all.index("Cld1")+2]
        dat.Cmd1=global_list_all[global_list_all.index("Cmd1")+2]
        dat.Cnd1=global_list_all[global_list_all.index("Cnd1")+2]
        dat.CDffd1=global_list_all[global_list_all.index("CDffd1")+2]
    
    	#Aileron
        dat.CLd2=global_list_all[global_list_all.index("CLd2")+2]
        dat.CYd2=global_list_all[global_list_all.index("CYd2")+2]
        dat.Cld2=global_list_all[global_list_all.index("Cld2")+2]
        dat.Cmd2=global_list_all[global_list_all.index("Cmd2")+2]
        dat.Cnd2=global_list_all[global_list_all.index("Cnd2")+2]
        dat.CDffd2=global_list_all[global_list_all.index("CDffd2")+2]
    
    	#Elevator
        dat.CLd3=global_list_all[global_list_all.index("CLd3")+2]
        dat.CYd3=global_list_all[global_list_all.index("CYd3")+2]
        dat.Cld3=global_list_all[global_list_all.index("Cld3")+2]
        dat.Cmd3=global_list_all[global_list_all.index("Cmd3")+2]
        dat.Cnd3=global_list_all[global_list_all.index("Cnd3")+2]
        dat.CDffd3=global_list_all[global_list_all.index("CDffd3")+2]
    
    	#Rudder
        dat.CLd4=global_list_all[global_list_all.index("CLd4")+2]
        dat.CYd4=global_list_all[global_list_all.index("CYd4")+2]
        dat.Cld4=global_list_all[global_list_all.index("Cld4")+2]
        dat.Cmd4=global_list_all[global_list_all.index("Cmd4")+2]
        dat.Cnd4=global_list_all[global_list_all.index("Cnd4")+2]
        dat.CDffd4=global_list_all[global_list_all.index("CDffd4")+2]
    
    
        
        dat.Xaero=.5*dat.rho*dat.V_inf**2*geo.S_ref_w*dat.CXtot
        dat.Yaero=.5*dat.rho*dat.V_inf**2*geo.S_ref_w*dat.CYtot
        dat.Zaero=.5*dat.rho*dat.V_inf**2*geo.S_ref_w*dat.CZtot
        dat.Laero_cg=.5*dat.rho*dat.V_inf**2*geo.S_ref_w*dat.Cltot*geo.Span_w
        dat.Maero_cg=.5*dat.rho*dat.V_inf**2*geo.S_ref_w*dat.Cmtot*geo.MAC_tot_w
        dat.Naero_cg=.5*dat.rho*dat.V_inf**2*geo.S_ref_w*dat.Cntot*geo.Span_w
        
    
    
    ###################################################################################
    ###################################################################################
    
    
    # return [Mach,rho,Xa,Ya,Za,La_cg,Ma_cg,Na_cg,CLtot,CDtot,CXtot,CYtot,CZtot,Cltot,Cmtot,Cntot,e,Static_margin,CLa,Cma,Cnb,Clb,CDvis]
