import numpy as np
from ATMOSPHERE.ATM import Atm


class Trim_definition:
    
    @classmethod
    def trim_expression_to_data(cls,dat,geo):
        
        
        dat.Crosswind_dir_earth_deg=0
        dat.Crosswind_mag_ms=0
        dat.Track=0
        dat.V_inf=50# initial condition
        
        [*Tr_ex]=[*dat.Trim_expression.values()]
        
        k=[]
        for i in dat.Trim_expression.keys():
            k.append(i)  
        exec("dat.%s='%s'" % (k[0],Tr_ex[0]))  
        exec("dat.%s='%s'" % (k[1],Tr_ex[1])) 
        # exec("dat.%s='%s'" % (k[2],Tr_ex[2])) 
        for i in range(2,len(Tr_ex)):
            exec("dat.%s=%f" % (k[i],Tr_ex[i]))
        
        c_deg=dat.Crosswind_dir_earth_deg  # where the wind blowing from
        c_mag=dat.Crosswind_mag_ms
        dat.CW_earth_axis=np.asarray([-np.cos(c_deg*np.pi/180),-np.sin(c_deg*np.pi/180),0])*np.asarray([c_mag]) #North,East,Down
        dat.CW_dot_body_axis=[0,0,0]

        
        # =============================================================================
        #      This section is used only for Psi_Prediction   
        # =============================================================================
        V_groud_deg=dat.Track
        V_ground_mag=(dat.V_inf**2-dat.Crosswind_mag_ms**2)**.5 # Assumption of wind is blowing from 90 degree
        dat.V_earth_axis=np.asarray([-np.cos(V_groud_deg*np.pi/180),-np.sin(V_groud_deg*np.pi/180),0])*np.asarray([V_ground_mag]) #North,East,Down
        Psi_predicted=(np.arctan((dat.CW_earth_axis[1]+dat.V_earth_axis[1])/(dat.CW_earth_axis[0]+dat.V_earth_axis[0]))*180/np.pi)*.9  # It requires for crosswind cases
        # =============================================================================
        #         
        # =============================================================================
        
        
        ##Inıtıal condıtıon States #### if one of these states not defined in xt then that states will be freezed##All is in degree except for P Q R##
        [dat.U,dat.V,dat.W,dat.V_inf,dat.Aoa,dat.Beta,dat.TEF,dat.HTL,dat.FlaperonL,dat.Rudder,dat.P,dat.Q,dat.R,dat.Theta,dat.Phi,dat.Psi,dat.PLA]=np.array([dat.V_inf,0,0,dat.V_inf,0,0,0,0.0,0.0,0.0,0,0,0,0,0,Psi_predicted,geo.Max_engine_hp])
        dat.H_dot=0
        ##Constrainted derivative terms## (to activate this constraint you should activate related equation in EOM function)
        [dat.U_dot,dat.V_dot,dat.W_dot,dat.V_inf_dot,dat.Alpha_dot,dat.Beta_dot,dat.P_dot,dat.Q_dot,dat.R_dot,dat.Theta_dot,dat.Phi_dot,dat.Psi_dot,dat.Rw,dat.Qw,\
        dat.Gamma,dat.Ny,dat.Nz,dat.Track,dat.Vt,dat.Sideslip_deg]=np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0])
            
        
        k=[]
        for i in dat.Trim_expression.keys():
            k.append(i)  
        exec("dat.%s='%s'" % (k[0],Tr_ex[0]))  
        exec("dat.%s='%s'" % (k[1],Tr_ex[1])) 
        # exec("dat.%s='%s'" % (k[2],Tr_ex[2])) 
        for i in range(2,len(Tr_ex)):
            exec("dat.%s=%f" % (k[i],Tr_ex[i]))
            
            
# =============================================================================
####    Trim tanımlarken V_inf yerine  V_eas unknown olarak girilebilir   #####
# =============================================================================
        Atm.atm(dat)
        if 'V_eas' in k:
            dat.V_inf=dat.V_eas*(1.225/dat.rho)**.5 # both of them are m/s
# =============================================================================
###############################################################################                 
# =============================================================================
            
            
        if dat.Flight_Mode=='Landing':
            dat.Altitude=dat.Altitude+((geo.Location_lg_body_right[2])**2+(geo.Location_lg_body_right[0])**2)**.5+0.05
        elif dat.Flight_Mode=='Normal' or dat.Flight_Mode=='TakeOff':
            dat.Altitude=dat.Altitude+((geo.Location_lg_body_nose[0])**2+(geo.Location_lg_body_nose[1])**2+(geo.Location_lg_body_nose[2])**2)**.5
            
            
    @classmethod
    def trim_def(cls,dat,geo):
        
        cls.trim_expression_to_data(dat,geo)
        
# =============================================================================
#         
# =============================================================================
        if dat.trim_conf=="Straight_flight":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
                
                # #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # # Unknowns   # Jacobian Unknowns
                # dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                # #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # # Equations   # Jacobian Equations
                # dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny"]
                # # Constants   #Not equation and not unknown # Not zero Constraints
                # [dat.P,dat.Q,dat.R]=[0,0,0]
                
            if dat.Unknown_type == "Body":
            
                #All_Unknowns={"U","V","W","Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["U","V","W","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny","Vt"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Vt]=[0,0,0,dat.V_inf]
                
                # #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # # Unknowns   # Jacobian Unknowns
                # dat.Unknowns = ["U","V","W","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                # #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # # Equations   # Jacobian Equations
                # dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny","Vt"]
                # # Constants   #Not equation and not unknown # Not zero Constraints
                # [dat.P,dat.Q,dat.R,dat.Vt]=[0,0,0,dat.V_inf]
                
        
        if dat.trim_conf=="Fixed_theta":

            if dat.Unknown_type == "Wind":
            
                dat.V_inf=30;
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","V_inf","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
                
    
        if dat.trim_conf=="Straight_flight_ROC":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Track","Ny"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
                
                # #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # # Unknowns   # Jacobian Unknowns
                # dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                # #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # # Equations   # Jacobian Equations
                # dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny"]
                # # Constants   #Not equation and not unknown # Not zero Constraints
                # [dat.P,dat.Q,dat.R]=[0,0,0]
                
        if dat.trim_conf=="Ceiling":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","Altitude"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Track","Ny","H_dot"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
                
                # #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # # Unknowns   # Jacobian Unknowns
                # dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                # #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # # Equations   # Jacobian Equations
                # dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny"]
                # # Constants   #Not equation and not unknown # Not zero Constraints
                # [dat.P,dat.Q,dat.R]=[0,0,0]
            
            if dat.Unknown_type == "Body":
            
                #All_Unknowns={"U","V","W","Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["U","V","W","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny","Vt"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Vt]=[0,0,0,dat.V_inf]
                
                # #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # # Unknowns   # Jacobian Unknowns
                # dat.Unknowns = ["U","V","W","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                # #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # # Equations   # Jacobian Equations
                # dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny","Vt"]
                # # Constants   #Not equation and not unknown # Not zero Constraints
                # [dat.P,dat.Q,dat.R,dat.Vt]=[0,0,0,dat.V_inf]
# =============================================================================
#                 
# =============================================================================
        if dat.trim_conf=="Take_of_Run":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
        
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","Gamma","P_dot","Q_dot","R_dot","Track","Ny"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
# =============================================================================
#                 
# =============================================================================
        if dat.trim_conf=="Quasi_Trim":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
        
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["HTL","FlaperonL","Rudder","Theta"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["P_dot","Q_dot","R_dot","Gamma"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                
# =============================================================================
#                 
# =============================================================================
        if dat.trim_conf=="Spin_Trim":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.
            dat.Aoa=50;
            if dat.Unknown_type == "Wind":
        
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","V_inf","P","Q","R","Theta"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Rw","Qw"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.HTL,dat.FlaperonL,dat.Rudder]=[0,0,0]
                
                
# =============================================================================
#                 
# =============================================================================
        if dat.trim_conf=="Quasi_Trim_Nz":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
        
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["HTL","FlaperonL","Rudder","Theta","Aoa"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["P_dot","Q_dot","R_dot","Gamma","Nz"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                
# =============================================================================
#                 
# =============================================================================
                      
        if dat.trim_conf=="Stall_Speed_Prediction":

            if dat.Unknown_type == "Wind":
                dat.V_inf=25 # initial condition for stall prediction
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["V_inf","Beta","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track","Ny"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
# =============================================================================
#                 
# =============================================================================
                      
        if dat.trim_conf=="Max_Cruise_Speed_Prediction":

            if dat.Unknown_type == "Wind":
            
                dat.V_inf=70 # initial condition for max speed prediction
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["V_inf","Aoa","Theta","Phi","HTL","FlaperonL","Rudder"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
# =============================================================================
#                 
# =============================================================================
            

        if dat.trim_conf=="Instant_turn":
            
            if dat.Unknown_type == "Wind":

                #All_Unknowns={"U","V","W","Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","P","Q","R","Theta","HTL","FlaperonL","Rudder"]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","P_dot","Q_dot","R_dot","Theta_dot","Phi_dot","Gamma","Ny","Nz"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.Phi]=[(180/np.pi)*np.arccos(1/dat.Nz)]
                
            if dat.Unknown_type == "Body":

                #All_Unknowns={"U","V","W","Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["U","V","W","P","Q","R","Theta","HTL","FlaperonL","Rudder"]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","P_dot","Q_dot","R_dot","Theta_dot","Phi_dot","Gamma","Ny","Nz","Vt"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.Phi,dat.Vt]=[(180/np.pi)*np.arccos(1/dat.Nz),dat.V_inf]
                
# =============================================================================
#                 
# =============================================================================
            
        if dat.trim_conf=="Coordinated_Turn":

            #All_Unknowns={"U","V","W","Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
            # Unknowns   # Jacobian Unknowns
            dat.Unknowns = ["Aoa","Beta","Theta","P","Q","R","HTL","FlaperonL","Rudder","PLA"]
            # Equations   # Jacobian Equations
            dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Theta_dot","Phi_dot","Ny"]
            # Constants   #Not equation and not unknown # Not zero Constraints
# =============================================================================
#             
# =============================================================================
            
        if dat.trim_conf=="Pull_up":
            
            #All_Unknowns={"U","V","W","Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
            # Unknowns   # Jacobian Unknowns
            dat.Unknowns = ["Aoa","Beta","Theta","Q","HTL","FlaperonL","Rudder","PLA"]
            # Equations   # Jacobian Equations
            dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Nz"]
            # Constants   #Not equation and not unknown # Not zero Constraints
            [dat.Phi]=[0]
# =============================================================================
#             
# =============================================================================
            
        if dat.trim_conf=="Steady_roll":

            #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
            # Unknowns   # Jacobian Unknowns
            dat.Unknowns = ["Theta","Rudder","HTL","PLA","FlaperonL","Aoa","Beta"]
            #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
            # Equations   # Jacobian Equations
            dat.Equations = ["V_inf_dot","R_dot","Q_dot","Gamma","P_dot","Alpha_dot","Beta_dot"]
            # Constants   #Not equation and not unknown # Not zero Constraints
            
# =============================================================================
#             
# =============================================================================

        if dat.trim_conf=="Sideslip_Landing":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","Theta","Phi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Psi]=[0,0,0,dat.Track]
            
            if dat.Unknown_type == "Body":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["U","V","W","Theta","Phi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track","Vt"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Vt,dat.Psi]=[0,0,0,dat.V_inf,dat.Track]
                
# =============================================================================
#             
# =============================================================================

        if dat.trim_conf=="Crosswind_Trim":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                dat.Crosswind_mag_ms=3
                
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Beta","Theta","Phi","HTL","FlaperonL","Crosswind_mag_ms","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Psi]=[0,0,0,dat.Track]
            

# =============================================================================
#                 
# =============================================================================

        if dat.trim_conf=="Crabbing_Landing":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Psi","Theta","Phi","HTL","FlaperonL","Crosswind_mag_ms","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Beta]=[0,0,0,0]
            
            if dat.Unknown_type == "Body":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["U","V","W","Theta","Psi","Phi","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Vt","Sideslip_deg"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Vt,dat.Sideslip_deg]=[0,0,0,dat.V_inf,0]
# =============================================================================
#                 
# =============================================================================
        if dat.trim_conf=="Combination_Landing":
            #Crosswind caselerinde Wind kullanmak daha avantajlı,
            #Çünkü Body de initial conditiondan dolayı bulunamayan caseler Wind de çok rahat trim bulunuyor.

            if dat.Unknown_type == "Wind":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["Aoa","Theta","Beta","HTL","FlaperonL","Rudder","PLA","Phi"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["U_dot","V_dot","W_dot","P_dot","Q_dot","R_dot","Gamma","Track"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R]=[0,0,0]
            
            if dat.Unknown_type == "Body":
            
                #All_Unknowns={"Aoa","Beta","P","Q","R","Theta","Phi","Psi","HTL","FlaperonL","Rudder","PLA"} #order of parameters is not important
                # Unknowns   # Jacobian Unknowns
                dat.Unknowns = ["U","V","W","Theta","Phi","Beta","HTL","FlaperonL","Rudder","PLA"]
                #All Equations= [U_dot,V_dot,W_dot,V_inf_dot,Alpha_dot,Beta_dot,P_dot,Q_dot,R_dot,Theta_dot,Phi_dot,Psi_dot,Gamma,Ny,Nz,Track,Vt]
                # Equations   # Jacobian Equations
                dat.Equations = ["Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Gamma","Track","Vt","Sideslip_deg"]
                # Constants   #Not equation and not unknown # Not zero Constraints
                [dat.P,dat.Q,dat.R,dat.Vt,dat.Sideslip_deg]=[0,0,0,dat.V_inf,dat.Beta]
# =============================================================================
#                 
# =============================================================================

        xc=""
        for j in range(len(dat.Unknowns)):
            if j==len(dat.Unknowns)-1:
                xc=xc+ '"%s":' % dat.Unknowns[j] 
                xc=xc+ 'cls.%s' % dat.Unknowns[j]
            else:
                xc=xc+ '"%s":' % dat.Unknowns[j] 
                xc=xc+ 'cls.%s,' % dat.Unknowns[j] 
        dat.xt_final="{"+xc+"}"    

        
    




    
