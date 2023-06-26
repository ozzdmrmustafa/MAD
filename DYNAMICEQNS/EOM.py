import numpy as np
from TRIM_SIM.TRANS_MATRIX import Transformation_Matrices as tm
from AERODYNAMICS.GETCOEFFICIENT import GetCoefficient

# Eom.eom

class Eom:
    
    @classmethod
    def Is_exist_in_Equations(cls,data,Constraint=""):
        
        try:
            data.Equations.index(Constraint) 
            return True
        except:
            return False
            
    
    @classmethod
    def eom(cls,dat,geo):
        
        
        
        
# =============================================================================
# To use Body unknowns (U,V,W)
# =============================================================================

        if dat.Unknown_type == "Body":
            
            
            dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)

            U_prime=dat.U-dat.CW_body_axis[0]
            V_prime=dat.V-dat.CW_body_axis[1]
            W_prime=dat.W-dat.CW_body_axis[2]

            
            dat.Alpha_rad=np.arctan2(W_prime,U_prime)
            dat.Beta_rad=np.arctan2(V_prime,(W_prime**2+U_prime**2)**.5)
            dat.V_inf=(U_prime**2+V_prime**2+W_prime**2)**.5
            
            [dat.Aoa,dat.Beta]=[dat.Alpha_rad*180/np.pi,dat.Beta_rad*180/np.pi]
# =============================================================================
# =============================================================================

        GetCoefficient.get_coefficient(dat,geo)
    
        
        [Alpha_rad,Beta_rad,Theta_rad,Phi_rad,Psi_rad,Gamma_rad,Track_rad]=\
            [dat.Aoa*np.pi/180,dat.Beta*np.pi/180,dat.Theta*np.pi/180,dat.Phi*np.pi/180,dat.Psi*np.pi/180,dat.Gamma*np.pi/180,dat.Track*np.pi/180]
    
        V_inf=dat.V_inf
        Vt=dat.Vt
        Sideslip_deg=dat.Sideslip_deg
        Mass_kg=dat.Mass_kg
        CW_dot_body_axis=dat.CW_dot_body_axis
        
        [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]=[dat.Ixx,dat.Iyy,dat.Izz,dat.Ixy,dat.Ixz,dat.Iyz];[P,Q,R,P_dot,Q_dot,R_dot]=[dat.P,dat.Q,dat.R,dat.P_dot,dat.Q_dot,dat.R_dot]
        [Xa,Ya,Za,La_cg,Ma_cg,Na_cg,Xt,Yt,Zt,Lt_cg,Mt_cg,Nt_cg]=[dat.Xa,dat.Ya,dat.Za,dat.La_cg,dat.Ma_cg,dat.Na_cg,dat.Xt,dat.Yt,dat.Zt,dat.Lt_cg,dat.Mt_cg,dat.Nt_cg]
        [Alpha_dot,Beta_dot,V_inf_dot]=[dat.Alpha_dot,dat.Beta_dot,dat.V_inf_dot];[Ny,Nz]=[dat.Ny,dat.Nz];[U,V,W]=[dat.U,dat.V,dat.W]
        [Phi_dot,Psi_dot,Theta_dot]=[dat.Phi_dot,dat.Psi_dot,dat.Theta_dot];[U_dot,V_dot,W_dot]=[dat.U_dot,dat.V_dot,dat.W_dot]
        Rw=dat.Rw
        Qw=dat.Qw
        H_dot=dat.H_dot
        
        

        g=9.806
    
        f_arr=np.zeros(shape=(len(dat.Equations)))

        i=0
        
# =============================================================================
# To use Wind unknowns (Aoa,Beta)       
# =============================================================================
        if dat.Unknown_type == "Wind":
            U_prime=V_inf*np.cos(Beta_rad)*np.cos(Alpha_rad) # Crosswind etkisini dahil etmek için bu denklemden rüzgarın hızını çıkar öyle (U,V,W)(yere göre) yu bul.
            V_prime=V_inf*np.sin(Beta_rad)
            W_prime=V_inf*np.cos(Beta_rad)*np.sin(Alpha_rad)
            
            
            dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)
            
            U=U_prime+dat.CW_body_axis[0]
            V=V_prime+dat.CW_body_axis[1]
            W=W_prime+dat.CW_body_axis[2]

        
# =============================================================================
# =============================================================================
    
        
        U_dot_prime=(R*V-Q*W-g*np.sin(Theta_rad)+(Xa+Xt)/Mass_kg)-CW_dot_body_axis[0]
        V_dot_prime=(-R*U+P*W+g*np.sin(Phi_rad)*np.cos(Theta_rad)+(Ya+Yt)/Mass_kg)-CW_dot_body_axis[1]
        W_dot_prime=(Q*U-P*V+g*np.cos(Phi_rad)*np.cos(Theta_rad)+(Za+Zt)/Mass_kg)-CW_dot_body_axis[2]
        
# =============================================================================
#       Body Axis
# =============================================================================


        if cls.Is_exist_in_Equations(dat,"U_dot"):
            f_arr[i]=(R*V-Q*W-g*np.sin(Theta_rad)+(Xa+Xt)/Mass_kg)-(U_dot);i+=1
        if cls.Is_exist_in_Equations(dat,"V_dot"):
            f_arr[i]=(-R*U+P*W+g*np.sin(Phi_rad)*np.cos(Theta_rad)+(Ya+Yt)/Mass_kg)-(V_dot);i+=1
        if cls.Is_exist_in_Equations(dat,"W_dot"):
            f_arr[i]=(Q*U-P*V+g*np.cos(Phi_rad)*np.cos(Theta_rad)+(Za+Zt)/Mass_kg)-(W_dot);i+=1
        
        
# =============================================================================
#         Wind Axis
# =============================================================================
        
        if cls.Is_exist_in_Equations(dat,"Alpha_dot"):
            f_arr[i]=(U_prime*W_dot_prime-W_prime*U_dot_prime)/(U_prime**2+W_prime**2)-Alpha_dot;i+=1
        if cls.Is_exist_in_Equations(dat,"Beta_dot"):
            f_arr[i]=(V_dot_prime*V_inf-V_prime*V_inf_dot)/(V_inf*(U_prime**2+W_prime**2)**0.5)-Beta_dot;i+=1
        if cls.Is_exist_in_Equations(dat,"V_inf_dot"):
            f_arr[i]=((U_prime*U_dot_prime+V_prime*V_dot_prime+W_prime*W_dot_prime)/V_inf)-V_inf_dot;i+=1
# =============================================================================
#             
# =============================================================================
        
       
        """
        f_arr[i]= Ixx*P_dot - Iyz*(Q**2 - R**2) - Ixz*(R_dot+P*Q) - Ixy*(Q_dot - R*P) - (Iyy - Izz)* (Q*R)-(La_cg+Lt_cg );i+=1
        
        f_arr[i]= Iyy*Q_dot - Ixz*(R**2 - P**2) - Ixy*(P_dot+Q*R) - Iyz*(R_dot - P*Q) - (Izz - Ixx)* (R*P)-(Ma_cg+Mt_cg );i+=1
        
        f_arr[i]= Izz*R_dot - Ixy*(P**2 - Q**2) - Iyz*(Q_dot+R*P) - Ixz*(P_dot - Q*R) - (Ixx - Iyy)* (P*Q)-(Na_cg+Nt_cg );i+=1
        """  
        #Yukarıdaki 3 denklemde Ixy ve Iyz dahil edilmiştir.Aşağıdaki 3 denklemde ise yok sayılmıştır.
        
        if cls.Is_exist_in_Equations(dat,"P_dot"):
            f_arr[i]= (Ixz*(Ixx-Iyy+Izz)*P*Q-(Izz*(Izz-Iyy)+Ixz**2)*Q*R+Izz*(La_cg+Lt_cg )+Ixz*(Na_cg+Nt_cg ))/(Ixx*Izz-Ixz**2)-P_dot;i+=1
        
        if cls.Is_exist_in_Equations(dat,"Q_dot"):
            f_arr[i]= ((Izz-Ixx)*P*R-Ixz*(P**2-R**2)+(Ma_cg+Mt_cg ))/Iyy-Q_dot;i+=1
        
        if cls.Is_exist_in_Equations(dat,"R_dot"):
            f_arr[i]= (((Ixx-Iyy)*Ixx+(Ixz**2))*(P*Q)-Ixz*(Ixx-Iyy+Izz)*Q*R+Ixz*(La_cg+Lt_cg )+Ixx*(Na_cg+Nt_cg))/(Ixx*Izz-Ixz**2)-R_dot;i+=1  
        
     
        if cls.Is_exist_in_Equations(dat,"Phi_dot"):
            f_arr[i]=P+Q*(np.sin(Phi_rad)+R*np.cos(Phi_rad))*np.tan(Theta_rad)-Phi_dot;i+=1 
        
        if cls.Is_exist_in_Equations(dat,"Theta_dot"):
            f_arr[i]=Q*np.cos(Phi_rad)-R*np.sin(Phi_rad)-Theta_dot ;i+=1
        
        if cls.Is_exist_in_Equations(dat,"Psi_dot"):
            f_arr[i]=(Q*np.sin(Phi_rad)+R*np.cos(Phi_rad))*(1/np.cos(Theta_rad))-Psi_dot ;i+=1

        
        x_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[0]
        y_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[1]
        z_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[2]

        
        if cls.Is_exist_in_Equations(dat,"Gamma"):
            # f_arr[i]=np.arcsin(h_dot/((U**2+V**2+W**2)**.5))-Gamma_rad ;i+=1
            f_arr[i]=np.arctan(-z_dot/x_dot)-Gamma_rad ;i+=1
            
        if cls.Is_exist_in_Equations(dat,"H_dot"):
            f_arr[i]=-z_dot-H_dot ;i+=1
            

        if cls.Is_exist_in_Equations(dat,"Nz"):
            f_arr[i]=(-(Za+Zt) / (Mass_kg*g))-Nz ;i+=1
            
        if cls.Is_exist_in_Equations(dat,"Ny"):
            f_arr[i]=( (Ya+Yt) / (Mass_kg*g))-Ny ;i+=1
            
        if cls.Is_exist_in_Equations(dat,"Track"):
            f_arr[i]=np.arctan(y_dot/x_dot)-Track_rad;i+=1
        
        if cls.Is_exist_in_Equations(dat,"Vt"):
            f_arr[i]=(U_prime**2+V_prime**2+W_prime**2)**0.5-Vt ;i+=1
            
        if cls.Is_exist_in_Equations(dat,"Sideslip_deg"):
            f_arr[i]=np.arctan(V_prime/((W_prime**2+U_prime**2)**.5))*180/np.pi-Sideslip_deg ;i+=1
            
            
        if cls.Is_exist_in_Equations(dat,"Qw"):
            f_arr[i]=-P*np.cos(Alpha_rad)*np.sin(Beta_rad)+Q*np.cos(Beta_rad)-R*np.sin(Alpha_rad)*np.sin(Beta_rad)  - Qw ;i+=1
            
        if cls.Is_exist_in_Equations(dat,"Rw"):
            f_arr[i]=-P*np.sin(Alpha_rad)+R*np.cos(Alpha_rad) - Rw ;i+=1
            
            
            
        
        
        fx=np.asarray(f_arr)
    
        return fx
    
        
        
        
        
        
