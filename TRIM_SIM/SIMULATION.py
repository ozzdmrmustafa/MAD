import os
import shutil
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

from TRIM_SIM.T import TClass
from TRIM_SIM.TRIM_DEF import Trim_definition
from TRIM_SIM.TRANS_MATRIX import Transformation_Matrices as tm
from AERODYNAMICS.GETCOEFFICIENT import GetCoefficient
from FCS_CONTROL.PID_CONTROL import Pid_Control

class Simulation:
    
    @classmethod
    def perturbation(cls,dat,index,k_index,perturb_coeff,for_index):
        executable=('''dat.%s=dat.%s+%f*cls.k%.0f[%.0f]''' % (index,index,perturb_coeff,k_index,for_index) )
        exec(executable)
    
    @classmethod
    def perturbation2(cls,dat,k_index,perturb_coeff,geo):
        
        Variables=["U","V","W","P","Q","R","Phi_rad","Theta_rad","Psi_rad","x","y","z"]
        i=0
        for index in Variables:
            cls.perturbation(dat,index,k_index,perturb_coeff,i)
            i=i+1
            
        exec('deriv_k%.0f=cls.deriv(dat,geo)'% (k_index+1))
        
        i=0
        for index in Variables:
            cls.perturbation(dat,index,k_index,-perturb_coeff,i)
            i=i+1
        
        return eval('deriv_k%.0f'% (k_index+1))

    @classmethod
    def deriv(cls,dat,geo):
        
        g=9.806
        
        dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)
 
        U_prime=dat.U-dat.CW_body_axis[0]
        V_prime=dat.V-dat.CW_body_axis[1]
        W_prime=dat.W-dat.CW_body_axis[2]
        
        dat.Alpha_rad=np.arctan2(W_prime,U_prime)
        dat.Beta_rad=np.arctan2(V_prime,(W_prime**2+U_prime**2)**.5)
        dat.V_inf=(U_prime**2+V_prime**2+W_prime**2)**.5
        
        [dat.Aoa,dat.Beta]=[dat.Alpha_rad*180/np.pi,dat.Beta_rad*180/np.pi]
            
        dat.Altitude=-dat.z #This line include altitude and density effect to the simulation
        
        ########################################################################################################################
        ########################################################################################################################
        # dat.Time_def_0=0
        # dat.Time_def_end=3
        # if dat.t_total>dat.Time_def_0 and dat.t_total<=dat.Time_def_end: # Time_def_0 saniyeden başlayıp Time_def_end saniyeye kadar etki edecek
        #     dat.HTL=dat.HTL-5   #t_total gerekli değere ulaşıldığında flaperon birden 20 derece bükülmesi için bu if i kullanabilirsin
        ########################################################################################################################
        ########################################################################################################################
        
        
        
        # =============================================================================
        #       PID CONTROL BLOCK  
        # =============================================================================
        Pid_Control.pid_control(dat,geo)
        # =============================================================================
        #         
        # =============================================================================

        
        GetCoefficient.get_coefficient(dat,geo)
        
        Mass_kg=dat.Mass_kg
        CW_dot_body_axis=dat.CW_dot_body_axis
        
        [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]=[dat.Ixx,dat.Iyy,dat.Izz,dat.Ixy,dat.Ixz,dat.Iyz]
        [Xa,Ya,Za,La_cg,Ma_cg,Na_cg,Xt,Yt,Zt,Lt_cg,Mt_cg,Nt_cg]=[dat.Xa,dat.Ya,dat.Za,dat.La_cg,dat.Ma_cg,dat.Na_cg,dat.Xt,dat.Yt,dat.Zt,dat.Lt_cg,dat.Mt_cg,dat.Nt_cg]
        [U,V,W]=[dat.U,dat.V,dat.W];[x,y,z]=[dat.x,dat.y,dat.z];[P,Q,R]=[dat.P,dat.Q,dat.R]
    
        [Theta_rad,Phi_rad,Psi_rad]=[dat.Theta*np.pi/180,dat.Phi*np.pi/180,dat.Psi*np.pi/180]
        
    
        # =============================================================================
        #     
        # =============================================================================
    
    
        U_dot=R*V-Q*W-g*np.sin(Theta_rad)+(Xa+Xt)/Mass_kg
        V_dot=-R*U+P*W+g*np.sin(Phi_rad)*np.cos(Theta_rad)+(Ya+Yt)/Mass_kg
        W_dot=Q*U-P*V+g*np.cos(Phi_rad)*np.cos(Theta_rad)+(Za+Zt)/Mass_kg
    
        P_dot=(Ixz*(Ixx-Iyy+Izz)*P*Q-(Izz*(Izz-Iyy)+Ixz**2)*Q*R+Izz*(La_cg+Lt_cg )+Ixz*(Na_cg+Nt_cg ))/(Ixx*Izz-Ixz**2)
        Q_dot=((Izz-Ixx)*P*R-Ixz*(P**2-R**2)+(Ma_cg+Mt_cg ))/Iyy
        R_dot=(((Ixx-Iyy)*Ixx+(Ixz**2))*(P*Q)-Ixz*(Ixx-Iyy+Izz)*Q*R+Ixz*(La_cg+Lt_cg )+Ixx*(Na_cg+Nt_cg))/(Ixx*Izz-Ixz**2)
    
        Phi_dot=P+Q*(np.sin(Phi_rad)+R*np.cos(Phi_rad))*np.tan(Theta_rad)  
        Theta_dot=Q*np.cos(Phi_rad)-R*np.sin(Phi_rad) 
        Psi_dot =(Q*np.sin(Phi_rad)+R*np.cos(Phi_rad))*(1/np.cos(Theta_rad))
    
        x_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[0]
        y_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[1]
        z_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[2]
            
        
        return [U_dot,V_dot,W_dot,P_dot,Q_dot,R_dot,Phi_dot,Theta_dot,Psi_dot,x_dot,y_dot,z_dot]
    @classmethod
    def k(cls,dat,geo):
        
        cls.k1=np.zeros(shape=(12))
        cls.k2=np.zeros(shape=(12))
        cls.k3=np.zeros(shape=(12))
        cls.k4=np.zeros(shape=(12))
        
        # =============================================================================
        #         RK 2
        # =============================================================================
        deriv_k1=cls.deriv(dat,geo)
        
        for index in range(12):
        
            cls.k1[index]=dat.delta_t*(deriv_k1[index])
    
        deriv_k2=cls.perturbation2(dat,1,0.5,geo)
    
        for index in range(12):
        
            cls.k2[index]=dat.delta_t*(deriv_k2[index])
        # =============================================================================
        #             
        # =============================================================================
    
        # deriv_k3=cls.perturbation2(dat,2,0.5)
    
        # for index in range(12):
        
        #     cls.k3[index]=dat.delta_t*(deriv_k3[index])
    
        # deriv_k4=cls.perturbation2(dat,3,1)
    
        # for index in range(12):
        
        #     cls.k4[index]=dat.delta_t*(deriv_k4[index])
            
        
        return [cls.k1,cls.k2,cls.k3,cls.k4]
    @classmethod
    def one_step_forward(cls,dat,geo):
        
        
        [cls.k1,cls.k2,cls.k3,cls.k4]=cls.k(dat,geo)
        
        State=np.zeros(shape=(12))
        
        State=[dat.U,dat.V,dat.W,dat.P,dat.Q,dat.R,dat.Phi_rad,dat.Theta_rad,dat.Psi_rad,dat.x,dat.y,dat.z]
        
        for i in range(12):
        
            # State[i]=State[i]+(1/6)*(cls.k1[i]+2*cls.k2[i]+2*cls.k3[i]+cls.k4[i])  # Runge Kutta 4
            State[i]=State[i]+(1/2)*(cls.k1[i]+cls.k2[i])  # Runge Kutta 2
            # State[i]=State[i]+(1)*(cls.k1[i])  #Euler Method 

          
        [dat.U,dat.V,dat.W,dat.P,dat.Q,dat.R,dat.Phi_rad,dat.Theta_rad,dat.Psi_rad,dat.x,dat.y,dat.z]=State
        
        [dat.Phi,dat.Theta,dat.Psi]=[dat.Phi_rad*180/np.pi,dat.Theta_rad*180/np.pi,dat.Psi_rad*180/np.pi]
        
        return [dat.U,dat.V,dat.W,dat.P,dat.Q,dat.R,dat.Phi_rad,dat.Theta_rad,dat.Psi_rad,dat.x,dat.y,dat.z]
    
    @classmethod
    def simulation(cls,dat,State,Input,Time,Delta_t,geo):
        
        [dat.Aoa,dat.Beta,dat.V_inf,dat.U,dat.V,dat.W,dat.P,dat.Q,dat.R,dat.Phi,dat.Theta,dat.Psi]=State
        
        [dat.HTL,dat.FlaperonL,dat.Rudder,dat.PLA]=Input
        
        [dat.Theta_rad,dat.Phi_rad,dat.Psi_rad]=[dat.Theta*np.pi/180,dat.Phi*np.pi/180,dat.Psi*np.pi/180]


        
        if dat.Flight_Mode=='TakeOff':
            dat.Altitude=geo.Location_lg_body_nose[2]
            dat.V_inf=0.5
            [dat.Theta,dat.Phi,dat.Psi]=[0,0,0]
            [dat.Aoa,dat.Beta]=[0,0]
            [dat.U,dat.V,dat.W]=[0.5,0,0]
            dat.P=0
            dat.Q=0
            dat.R=0
            dat.HTL=0
            dat.FlaperonL=0
            dat.Rudder=0
            dat.PLA=geo.Max_engine_hp

        
        
        
        if dat.Unknown_type=="Wind":
            
            dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)

            [dat.Alpha_rad,dat.Beta_rad]=[dat.Aoa*np.pi/180,dat.Beta*np.pi/180]
            
            U_prime=dat.V_inf*np.cos(dat.Beta_rad)*np.cos(dat.Alpha_rad) # Crosswind etkisini dahil etmek için bu denklemden rüzgarın hızını çıkar öyle (U,V,W)(yere göre) yu bul.
            V_prime=dat.V_inf*np.sin(dat.Beta_rad)
            W_prime=dat.V_inf*np.cos(dat.Beta_rad)*np.sin(dat.Alpha_rad)
            
                       
            dat.U=U_prime+dat.CW_body_axis[0]
            dat.V=V_prime+dat.CW_body_axis[1]
            dat.W=W_prime+dat.CW_body_axis[2]

        
        
        dat.z=-dat.Altitude
        dat.x=0
        dat.y=0
        
        dat.t_0=0
        dat.delta_t=Delta_t
        dat.final_t=Time
        
        #Disturbances#
        
        dat.W=dat.W+0
        dat.V=dat.V+0
        dat.U=dat.U+0
        dat.P=dat.P+0
        dat.Q=dat.Q+0
        dat.R=dat.R+0
        dat.Theta=dat.Theta+0
        dat.Phi=dat.Phi+0
        dat.Psi=dat.Psi+0

        iter_number=round(dat.final_t/dat.delta_t)
        
        dat.U_arr=np.zeros(shape=(iter_number))
        dat.V_arr=np.zeros(shape=(iter_number))
        dat.W_arr=np.zeros(shape=(iter_number))
        dat.P_arr=np.zeros(shape=(iter_number))
        dat.Q_arr=np.zeros(shape=(iter_number))
        dat.R_arr=np.zeros(shape=(iter_number))
        dat.Theta_rad_arr=np.zeros(shape=(iter_number))
        dat.Phi_rad_arr=np.zeros(shape=(iter_number))
        dat.Psi_rad_arr=np.zeros(shape=(iter_number))
        dat.x_arr=np.zeros(shape=(iter_number))
        dat.y_arr=np.zeros(shape=(iter_number))
        dat.z_arr=np.zeros(shape=(iter_number))
        dat.x_dot_arr=np.zeros(shape=(iter_number))
        dat.y_dot_arr=np.zeros(shape=(iter_number))
        dat.z_dot_arr=np.zeros(shape=(iter_number))
        dat.HTL_arr=np.zeros(shape=(iter_number))
        dat.FlaperonL_arr=np.zeros(shape=(iter_number))
        dat.Rudder_arr=np.zeros(shape=(iter_number))
        dat.PLA_arr=np.zeros(shape=(iter_number))
        
        dat.U_dot_arr=np.zeros(shape=(iter_number))
        dat.V_dot_arr=np.zeros(shape=(iter_number))
        dat.W_dot_arr=np.zeros(shape=(iter_number))
        dat.P_dot_arr=np.zeros(shape=(iter_number))
        dat.Q_dot_arr=np.zeros(shape=(iter_number))
        dat.R_dot_arr=np.zeros(shape=(iter_number))
        dat.Phi_dot_arr=np.zeros(shape=(iter_number))
        dat.Theta_dot_arr=np.zeros(shape=(iter_number))
        dat.Psi_dot_arr=np.zeros(shape=(iter_number))
        dat.x_dot_arr=np.zeros(shape=(iter_number))
        dat.y_dot_arr=np.zeros(shape=(iter_number))
        dat.z_dot_arr=np.zeros(shape=(iter_number))
        dat.h_dot_arr=np.zeros(shape=(iter_number))
        dat.Gamma_arr=np.zeros(shape=(iter_number))
        dat.Nz_arr=np.zeros(shape=(iter_number))
        dat.Ny_arr=np.zeros(shape=(iter_number))
        dat.Track_arr=np.zeros(shape=(iter_number))
        dat.Vt_arr=np.zeros(shape=(iter_number))
        dat.Alpha_dot_arr=np.zeros(shape=(iter_number))
        dat.Beta_dot_arr=np.zeros(shape=(iter_number))
        dat.V_inf_dot_arr=np.zeros(shape=(iter_number))


        dat.t_total_arr=np.zeros(shape=(iter_number))
        dat.CW_body_axis_arr_0=np.zeros(shape=(iter_number))
        dat.CW_body_axis_arr_1=np.zeros(shape=(iter_number))
        dat.CW_body_axis_arr_2=np.zeros(shape=(iter_number))
        
        dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)    
        dat.CW_body_axis_arr_0[0]=dat.CW_body_axis[0]
        dat.CW_body_axis_arr_1[0]=dat.CW_body_axis[1]
        dat.CW_body_axis_arr_2[0]=dat.CW_body_axis[2]
        
        dat.x_dot_arr[0]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[0]
        dat.y_dot_arr[0]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[1]
        dat.z_dot_arr[0]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[2]
        
        
        [dat.U_arr[0],dat.V_arr[0],dat.W_arr[0],dat.P_arr[0],dat.Q_arr[0],dat.R_arr[0],dat.Phi_rad_arr[0],\
         dat.Theta_rad_arr[0],dat.Psi_rad_arr[0],dat.x_arr[0],dat.y_arr[0],dat.z_arr[0],dat.HTL_arr[0],\
             dat.FlaperonL_arr[0],dat.Rudder_arr[0],dat.PLA_arr[0],dat.t_total_arr[0]]\
            =np.array([dat.U,dat.V,dat.W,dat.P,dat.Q,dat.R,dat.Phi*np.pi/180,dat.Theta*np.pi/180,dat.Psi*np.pi/180,dat.x,\
                       dat.y,dat.z,dat.HTL,dat.FlaperonL,dat.Rudder,dat.PLA,0])
        
        
        for t in range(1,iter_number):
             
            dat.t_total=t*dat.delta_t
        
            [U,V,W,P,Q,R,Phi_rad,Theta_rad,Psi_rad,x,y,z]=cls.one_step_forward(dat,geo)  
            
            ########################################################################
            # Write down below section Whatever you want to see as an output in dat. structure #
            ########################################################################
            
            [dat.x,dat.y,dat.z]=[x,y,z]
            
            GetCoefficient.get_coefficient(dat,geo)
            
            Mass_kg=dat.Mass_kg;g=9.81;
            CW_dot_body_axis=dat.CW_dot_body_axis
            
            [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]=[dat.Ixx,dat.Iyy,dat.Izz,dat.Ixy,dat.Ixz,dat.Iyz]
            [Xa,Ya,Za,La_cg,Ma_cg,Na_cg,Xt,Yt,Zt,Lt_cg,Mt_cg,Nt_cg]=[dat.Xa,dat.Ya,dat.Za,dat.La_cg,dat.Ma_cg,dat.Na_cg,dat.Xt,dat.Yt,dat.Zt,dat.Lt_cg,dat.Mt_cg,dat.Nt_cg]
            [U,V,W]=[dat.U,dat.V,dat.W];[x,y,z]=[dat.x,dat.y,dat.z];[P,Q,R]=[dat.P,dat.Q,dat.R]
        
            [dat.Theta_rad,dat.Phi_rad,dat.Psi_rad]=[dat.Theta*np.pi/180,dat.Phi*np.pi/180,dat.Psi*np.pi/180]
            [Theta_rad,Phi_rad,Psi_rad]=[dat.Theta_rad,dat.Phi_rad,dat.Psi_rad]
        
        
            U_dot=(R*V-Q*W-g*np.sin(Theta_rad)+(Xa+Xt)/Mass_kg)
            V_dot=(-R*U+P*W+g*np.sin(Phi_rad)*np.cos(Theta_rad)+(Ya+Yt)/Mass_kg)
            W_dot=(Q*U-P*V+g*np.cos(Phi_rad)*np.cos(Theta_rad)+(Za+Zt)/Mass_kg)
            
            P_dot= (Ixz*(Ixx-Iyy+Izz)*P*Q-(Izz*(Izz-Iyy)+Ixz**2)*Q*R+Izz*(La_cg+Lt_cg )+Ixz*(Na_cg+Nt_cg ))/(Ixx*Izz-Ixz**2)
            Q_dot= ((Izz-Ixx)*P*R-Ixz*(P**2-R**2)+(Ma_cg+Mt_cg ))/Iyy
            R_dot= (((Ixx-Iyy)*Ixx+(Ixz**2))*(P*Q)-Ixz*(Ixx-Iyy+Izz)*Q*R+Ixz*(La_cg+Lt_cg )+Ixx*(Na_cg+Nt_cg))/(Ixx*Izz-Ixz**2)
     
            Phi_dot=P+Q*(np.sin(Phi_rad)+R*np.cos(Phi_rad))*np.tan(Theta_rad)
            Theta_dot=Q*np.cos(Phi_rad)-R*np.sin(Phi_rad)
            Psi_dot=(Q*np.sin(Phi_rad)+R*np.cos(Phi_rad))*(1/np.cos(Theta_rad))
            
            x_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[0]
            y_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[1]
            z_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[2]
        
            h_dot=-z_dot
        
            Gamma=np.arctan(h_dot/x_dot)*180/np.pi
            Nz=(-(Za+Zt) / (Mass_kg*g))
            Ny=( (Ya+Yt) / (Mass_kg*g))
            Track=y_dot*180/(x_dot*np.pi)
            
            U_prime=dat.U-dat.CW_body_axis[0]
            V_prime=dat.V-dat.CW_body_axis[1]
            W_prime=dat.W-dat.CW_body_axis[2]
            
            Vt=(U_prime**2+V_prime**2+W_prime**2)**0.5
            
            ####Assume it####
            U_dot_prime=U_dot
            V_dot_prime=V_dot
            W_dot_prime=W_dot
            #################
            
            V_inf_dot=((U_prime*U_dot_prime+V_prime*V_dot_prime+W_prime*W_dot_prime)/dat.V_inf)
            Alpha_dot=(U_prime*W_dot_prime-W_prime*U_dot_prime)/(U_prime**2+W_prime**2)
            Beta_dot=(V_dot_prime*dat.V_inf-V_prime*V_inf_dot)/(dat.V_inf*(U_prime**2+W_prime**2)**0.5)
            
        
            [dat.U_dot,dat.V_dot,dat.W_dot,dat.P_dot,dat.Q_dot,dat.R_dot,dat.Phi_dot,dat.Theta_dot,dat.Psi_dot,\
            dat.x_dot,dat.y_dot,dat.z_dot,dat.h_dot,dat.Gamma,dat.Nz,dat.Ny,dat.Track,dat.Vt,dat.Alpha_dot,dat.Beta_dot,dat.V_inf_dot]=\
            [U_dot,V_dot,W_dot,P_dot,Q_dot,R_dot,Phi_dot,Theta_dot,Psi_dot,x_dot,y_dot,z_dot,h_dot,Gamma,Nz,Ny,Track,Vt,Alpha_dot,Beta_dot,V_inf_dot]
            
            ################################################################################

            # =============================================================================
            # U 0.1 ten küçükse simulasyon durur            
            # =============================================================================
            
            break_t=1e6
            
            if dat.Flight_Mode=='Landing' and (U**2+V**2+W**2)**.5<=1:
                break_t=t
            if dat.Flight_Mode=='TakeOff' and dat.Altitude>15:
                break_t=t
            if dat.Flight_Mode=='Normal' and dat.Altitude<1:
                break_t=t
                
            
            if t >= break_t:
                
                for i in range(t,iter_number):
                    
                    [dat.U_arr[i],dat.V_arr[i],dat.W_arr[i],dat.P_arr[i],dat.Q_arr[i],dat.R_arr[i],dat.Phi_rad_arr[i],\
                      dat.Theta_rad_arr[i],dat.Psi_rad_arr[i],dat.x_arr[i],dat.y_arr[i],dat.z_arr[i],dat.HTL_arr[i],\
                          dat.FlaperonL_arr[i],dat.Rudder_arr[i],dat.PLA_arr[i],dat.t_total_arr[i]]\
                        =np.array([U,V,W,P,Q,R,Phi_rad,Theta_rad,Psi_rad,x,y,z,dat.HTL,dat.FlaperonL,dat.Rudder,dat.PLA,dat.t_total])
                        
                        
                    [dat.U_dot_arr[i],dat.V_dot_arr[i],dat.W_dot_arr[i],dat.P_dot_arr[i],dat.Q_dot_arr[i],dat.R_dot_arr[i],dat.Phi_dot_arr[i]\
                    ,dat.Theta_dot_arr[i],dat.Psi_dot_arr[i],dat.x_dot_arr[i],dat.y_dot_arr[i],dat.z_dot_arr[i],dat.h_dot_arr[i]\
                    ,dat.Gamma_arr[i],dat.Nz_arr[i],dat.Ny_arr[i],dat.Track_arr[i],dat.Vt_arr[i],dat.Alpha_dot_arr[i],dat.Beta_dot_arr[i],dat.V_inf_dot_arr[i]]=\
                    [U_dot,V_dot,W_dot,P_dot,Q_dot,R_dot,Phi_dot,Theta_dot,Psi_dot,x_dot,y_dot,z_dot,h_dot,Gamma,Nz,Ny,Track,Vt,Alpha_dot,Beta_dot,V_inf_dot]
                    
                    
                    
                    dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)    
                    dat.CW_body_axis_arr_0[i]=dat.CW_body_axis[0]
                    dat.CW_body_axis_arr_1[i]=dat.CW_body_axis[1]
                    dat.CW_body_axis_arr_2[i]=dat.CW_body_axis[2]
                    
                    dat.x_dot_arr[i]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[0]
                    dat.y_dot_arr[i]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[1]
                    dat.z_dot_arr[i]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[2]
                    
                break
                
            else:
                
                if t==1:
                    [dat.U_dot_arr[0],dat.V_dot_arr[0],dat.W_dot_arr[0],dat.P_dot_arr[0],dat.Q_dot_arr[0],dat.R_dot_arr[0],dat.Phi_dot_arr[0]\
                    ,dat.Theta_dot_arr[0],dat.Psi_dot_arr[0],dat.x_dot_arr[0],dat.y_dot_arr[0],dat.z_dot_arr[0],dat.h_dot_arr[0]\
                    ,dat.Gamma_arr[0],dat.Nz_arr[0],dat.Ny_arr[0],dat.Track_arr[0],dat.Vt_arr[0],dat.Alpha_dot_arr[0],dat.Beta_dot_arr[0],dat.V_inf_dot_arr[0]]=\
                    [U_dot,V_dot,W_dot,P_dot,Q_dot,R_dot,Phi_dot,Theta_dot,Psi_dot,x_dot,y_dot,z_dot,h_dot,Gamma,Nz,Ny,Track,Vt,Alpha_dot,Beta_dot,V_inf_dot]
                    

                [dat.U_arr[t],dat.V_arr[t],dat.W_arr[t],dat.P_arr[t],dat.Q_arr[t],dat.R_arr[t],dat.Phi_rad_arr[t],\
                  dat.Theta_rad_arr[t],dat.Psi_rad_arr[t],dat.x_arr[t],dat.y_arr[t],dat.z_arr[t],dat.HTL_arr[t],\
                      dat.FlaperonL_arr[t],dat.Rudder_arr[t],dat.PLA_arr[t],dat.t_total_arr[t]]\
                    =np.array([U,V,W,P,Q,R,Phi_rad,Theta_rad,Psi_rad,x,y,z,dat.HTL,dat.FlaperonL,dat.Rudder,dat.PLA,dat.t_total])
                    
                    
                [dat.U_dot_arr[t],dat.V_dot_arr[t],dat.W_dot_arr[t],dat.P_dot_arr[t],dat.Q_dot_arr[t],dat.R_dot_arr[t],dat.Phi_dot_arr[t]\
                ,dat.Theta_dot_arr[t],dat.Psi_dot_arr[t],dat.x_dot_arr[t],dat.y_dot_arr[t],dat.z_dot_arr[t],dat.h_dot_arr[t]\
                ,dat.Gamma_arr[t],dat.Nz_arr[t],dat.Ny_arr[t],dat.Track_arr[t],dat.Vt_arr[t],dat.Alpha_dot_arr[t],dat.Beta_dot_arr[t],dat.V_inf_dot_arr[t]]=\
                [U_dot,V_dot,W_dot,P_dot,Q_dot,R_dot,Phi_dot,Theta_dot,Psi_dot,x_dot,y_dot,z_dot,h_dot,Gamma,Nz,Ny,Track,Vt,Alpha_dot,Beta_dot,V_inf_dot]
                   
                dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)    
                dat.CW_body_axis_arr_0[t]=dat.CW_body_axis[0]
                dat.CW_body_axis_arr_1[t]=dat.CW_body_axis[1]
                dat.CW_body_axis_arr_2[t]=dat.CW_body_axis[2]
                
                dat.x_dot_arr[t]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[0]
                dat.y_dot_arr[t]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[1]
                dat.z_dot_arr[t]=np.dot(tm.earth_to_body(dat).T,[dat.U,dat.V,dat.W])[2]
                    
            # =============================================================================
            #            
            # =============================================================================
                
            print(dat.t_total)
        
        
        U_arr_prime=dat.U_arr-dat.CW_body_axis_arr_0
        V_arr_prime=dat.V_arr-dat.CW_body_axis_arr_1
        W_arr_prime=dat.W_arr-dat.CW_body_axis_arr_2
        

        
        dat.Track_deg_arr=np.arctan(dat.y_dot_arr/dat.x_dot_arr)*180/np.pi
        dat.Gamma_deg_arr=np.arctan2(-dat.z_dot_arr,dat.x_dot_arr)*180/np.pi
        
        Aoa_rad_arr=np.arctan2(W_arr_prime,U_arr_prime)
        Beta_rad_arr=np.arctan2(V_arr_prime,(W_arr_prime**2+U_arr_prime**2)**.5)
        dat.V_inf_arr=(U_arr_prime**2+V_arr_prime**2+W_arr_prime**2)**.5
        
        
        [dat.Aoa_arr]=[Aoa_rad_arr*180/np.pi]
        [dat.Beta_arr]=[Beta_rad_arr*180/np.pi]
        
        [dat.Phi_arr]=[dat.Phi_rad_arr*180/np.pi]
        [dat.Theta_arr]=[dat.Theta_rad_arr*180/np.pi]
        [dat.Psi_arr]=[dat.Psi_rad_arr*180/np.pi]
        


        return dat
    
    @classmethod
    def sim_plot(cls,dat):
        
 
        ################PLOTS###################  
        
        [U_arr,V_arr,W_arr,P_arr,Q_arr,R_arr,Theta_arr,Phi_arr,Psi_arr,x_arr,y_arr,z_arr,t_total_arr,Aoa_arr,Beta_arr,V_inf_arr]=\
            [dat.U_arr,dat.V_arr,dat.W_arr,dat.P_arr,dat.Q_arr,dat.R_arr,dat.Theta_arr,dat.Phi_arr,dat.Psi_arr,dat.x_arr,dat.y_arr,dat.z_arr,dat.t_total_arr,dat.Aoa_arr,dat.Beta_arr,dat.V_inf_arr]
            
        HTL_arr=dat.HTL_arr        
        FlaperonL_arr=dat.FlaperonL_arr            
        Rudder_arr=dat.Rudder_arr            
        PLA_arr=dat.PLA_arr            

        
        ###########################################
        
        if dat.sim_plot=='on':
        
            plt.plot(dat.t_total_arr,dat.Aoa_arr,color='r',label="Aoa(deg)")
            plt.plot(t_total_arr,np.full(len(Aoa_arr), dat.Aoa),color='r',label="Aoa_trim(deg)", linestyle='-.')
            plt.plot(t_total_arr,Beta_arr,color='b',label="Beta(deg)")
            plt.plot(t_total_arr,np.full(len(Beta_arr), dat.Beta),color='b',label="Beta_trim(deg)", linestyle='-.')
            
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Aerodynamic Angles")
            plt.title('Aerodynamic Angles simulation')
            plt.show()
            
            ###########################################
            
            plt.plot(t_total_arr,U_arr,color='r',label="U(m/s)")
            plt.plot(t_total_arr,V_arr,color='b',label="V(m/s)")
            plt.plot(t_total_arr,W_arr,color='g',label="W(m/s)")
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Velocity")
            plt.title('Velocity simulation')
            plt.show()
            
            ###########################################
            
            plt.plot(t_total_arr,W_arr,color='g',label="W(m/s)")
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("W Velocity")
            plt.title('W Velocity simulation')
            plt.show()
            
            ###########################################
                
            plt.plot(t_total_arr,P_arr,color='y',label="P(rad/s)")
            plt.plot(t_total_arr,Q_arr,color='b',label="Q(rad/s)")
            plt.plot(t_total_arr,R_arr,color='c',label="R(rad/s)")
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Angular_Velocity")
            plt.title('Angular Velocity simulation')
            plt.show()
                
            ########################################### 
            
            plt.plot(t_total_arr,Theta_arr,color='r',label="Theta(deg)")
            plt.plot(t_total_arr,Phi_arr,color='g',label="Phi(deg)")
            plt.plot(t_total_arr,Psi_arr,color='b',label="Psi(deg)")
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Euler Angles")
            plt.title('Euler Angles simulation')
            plt.show()
                
            ###########################################
            
            plt.plot(t_total_arr,dat.Gamma_deg_arr,color='b',label="Gamma(deg)")
            
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Flight Path Angle Angle")
            plt.title('Gamma Angle simulation')
            plt.show()
            
            ############################################
            
            
            plt.plot(t_total_arr,HTL_arr,color='r',label="HTL(deg)")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Control Surface Deflection Angles")
            plt.title('Control Surface Deflection Angles simulation')
            plt.show()
            
            plt.plot(t_total_arr,FlaperonL_arr,color='g',label="Flaperon(deg)")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Control Surface Deflection Angles")
            plt.title('Control Surface Deflection Angles simulation')
            plt.show()
            
            plt.plot(t_total_arr,Rudder_arr,color='b',label="Rudder(deg)")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Control Surface Deflection Angles")
            plt.title('Control Surface Deflection Angles simulation')
            plt.show()
            
            plt.plot(t_total_arr,PLA_arr,color='k',label="PLA(deg)")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Control Surface Deflection Angles")
            plt.title('Control Surface Deflection Angles simulation')
            plt.show()
    
            ########################################### 
     
            plt.plot(t_total_arr,-z_arr,color='g',label="Altitude")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Position")
            plt.title('Position simulation')
            plt.show()
    
            plt.plot(t_total_arr,x_arr,color='r',label="x")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Position")
            plt.title('Position simulation')
            plt.show()
    
            plt.plot(t_total_arr,y_arr,color='b',label="y")
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.xlabel("Time")
            plt.ylabel("Position")
            plt.title('Position simulation')
            plt.show()
        
            ###########################################
            for i in range(4):
                
                fig = plt.figure()
                ax = plt.axes(projection="3d")
                ax.plot3D(x_arr, y_arr, -z_arr, c='blue')
                #ax.scatter3D(x_arr, y_arr, z_arr, c=z_arr, cmap='hsv');
                ax.set_xlabel('North',c='r')
                ax.set_ylabel('East',c='r')
                ax.set_zlabel('Up',c='r')
             
                limit_max_x=max(x_arr)
                limit_max_y=max(y_arr)
                limit_max_z=max(-z_arr)
            
                limit_min_x=min(x_arr)   
                limit_min_y=min(y_arr)
                limit_min_z=min(-z_arr)
            
                x_interval=limit_max_x-limit_min_x  
                y_interval=limit_max_y-limit_min_y  
                z_interval=limit_max_z-limit_min_z  
            
                total_interval=max(x_interval,y_interval,z_interval)
            
                ax.set_xlim(limit_min_x, limit_min_x+total_interval)
                ax.set_ylim(limit_min_y, limit_min_y+total_interval)
                ax.set_zlim(limit_min_z, limit_min_z+total_interval)
            
                ax.set_title('Position Simulation')
            
                if i==0 : ax.view_init(0, 0)
                if i==1 : ax.view_init(0, 90) 
                if i==2 : ax.view_init(-90, 0) 
                if i==3 : ax.view_init(30, 30) 
                plt.show() 
                      
            ###########################################    
        
            #Print Out to txt file
            
            file=open("TRIM_SIM\\simout.txt","w")
            
            Variable_names=["t_total_arr","x_arr","y_arr","z_arr","Theta_arr","Phi_arr","Psi_arr","FlaperonL_arr","HTL_arr","Rudder_arr","PLA_arr"\
                            ,"Aoa_arr","Beta_arr","U_arr","V_arr","W_arr","P_arr","Q_arr","R_arr","V_inf_arr","Gamma_deg_arr","Track_deg_arr"\
                                ,"P_dot_arr","Q_dot_arr","R_dot_arr","V_inf_dot_arr","Alpha_dot_arr","Beta_dot_arr","h_dot_arr","x_dot_arr","y_dot_arr"\
                                    ,"z_dot_arr","Phi_dot_arr","Theta_dot_arr","Psi_dot_arr","U_dot_arr","V_dot_arr","W_dot_arr","Nz_arr","Ny_arr"]
    
            for iVariable in Variable_names:
                file.write("%s    " %(iVariable))    
                
            for iTime in np.arange(1,len(dat.x_arr)) :
                file.write("\n")
                for iVariable in Variable_names:
                    file.write("%.4f    " % (eval("dat.%s[%d]" % (iVariable,iTime)))) 
                        
            file.close()
            
            current_path = os.getcwd()
            src_path = current_path + '\\TRIM_SIM\\simout.txt'
            dst_path1 = current_path + '\\TRIM_SIM\\Visual3D'+ '\\simout.txt'
            dst_path2 = current_path + '\\TRIM_SIM\\Visual3D\\TestDataStored'+ '\\simout.txt'
            
            shutil.copy(src_path, dst_path1)
            shutil.copy(src_path, dst_path2)
    
            #os.chdir(current_path+'\\TRIM_SIM\\Visual3D')
            #os.system("Main.exe")
            #os.chdir(current_path)
        
                
            
        
        
        
        
        
        
        
        
        
        
        
        
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
