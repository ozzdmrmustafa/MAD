import numpy as np

from TRIM_SIM.TRIM_DEF import Trim_definition
from DYNAMICEQNS.EOM import Eom
from TRIM_SIM.JACOBIAN import Jacobian
from TRIM_SIM.TRANS_MATRIX import Transformation_Matrices as tm
from AERODYNAMICS.GETCOEFFICIENT import GetCoefficient


class TClass: 

    @classmethod    
    def t(cls,dat,geo):

        dat.fx=Eom.eom(dat,geo)
            
        # define the initial value for the error and the starting iteration count
        err = np.linalg.norm(dat.fx)
        iter = 0
      
        if dat.print_info:
            print("***************************\nInıtilization of Iteration {:d}: Error of //{:.6f}//\nFor {:}\n***************************".format(iter,err,dat.xt()))
            
        # perform the Newton-Raphson iteration algorithm
        while err > dat.tolerance and iter < dat.max_num_iter:
            
            
            dat.Aoa= float(np.clip(dat.Aoa,min(geo.Aoa_Limits),max(geo.Aoa_Limits)))   
            dat.Beta =float(np.clip(dat.Beta,min(geo.Beta_Limits),max(geo.Beta_Limits)))
            dat.FlaperonL=float(np.clip(dat.FlaperonL,min(geo.Flaperon_Limits),max(geo.Flaperon_Limits)))
            dat.HTL=float(np.clip(dat.HTL,min(geo.HTL_Limits),max(geo.HTL_Limits)))
            dat.Rudder=float(np.clip(dat.Rudder,min(geo.Rudder_Limits),max(geo.Rudder_Limits)))
            dat.PLA=float(np.clip(dat.PLA,0,geo.Max_engine_hp))
            
            """
            P=float(np.clip(P,-0.316228865,0.316228865))
            Q=float(np.clip(Q,-0.102103329,+0.102103329))
            R=float(np.clip(R,-0.316228865,0.316228865))
            """
           
            dat.Theta=float(np.clip(dat.Theta,-90,+90))
            
            dat.Phi=float(np.clip(dat.Phi,-180,+180))
 
            """
            Psi=float(np.clip(Psi,0,360))
            """
            
            J=Jacobian.jacobian(dat,geo)
            Rank=np.linalg.matrix_rank(J)
            
            if iter==0 and dat.print_info:
                print("Rank :%.0f\nNumber of Equation:%.0f\nNumber of Unknowns:%.0f\n***************************" % (Rank,len(dat.Equations),len(dat.Unknowns)))
            
                
            # perform newton step
            
            dat.x_temp=dat.x_vrb()
    
            dat.x_temp = dat.x_temp - dat.ksi*np.linalg.solve(J,dat.fx)
            
            dat.x_temp_to_variables_and_xt_and_x()
            
            
            # update the function value at the new root estimate
            dat.fx = np.asarray(Eom.eom(dat,geo))
            
            # compute the current root error
            err = np.linalg.norm(dat.fx)
            
            # update the iteration counter
            
            iter = iter + 1

            if dat.print_info:
                print("\nIteration {:d}: Error of //{:.6f}//\nWith an estimate of {:}".format(iter, err, dat.xt()))
            if dat.print_info and err<dat.tolerance:
                print ("\nTrim succesfully completed.\n\n ")
            if err<dat.tolerance:
                dat.xcode=0
            
            # return the root estimate, 2-norm error of the estimate, and iteration count we ended at
            
            
        GetCoefficient.get_coefficient(dat,geo)
    
        
        [Alpha_rad,Beta_rad,Theta_rad,Phi_rad,Psi_rad,Gamma_rad,Track_rad]=\
            [dat.Aoa*np.pi/180,dat.Beta*np.pi/180,dat.Theta*np.pi/180,dat.Phi*np.pi/180,dat.Psi*np.pi/180,dat.Gamma*np.pi/180,dat.Track*np.pi/180]
        [Aoa,Beta,Theta,Phi,Psi,Gamma,Track]=[dat.Aoa,dat.Beta,dat.Theta,dat.Phi,dat.Psi,dat.Gamma,dat.Track]

    
        g=9.806
        
        V_inf=dat.V_inf
        Mass_kg=dat.Mass_kg
        [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]=[dat.Ixx,dat.Iyy,dat.Izz,dat.Ixy,dat.Ixz,dat.Iyz]
        [P,Q,R,P_dot,Q_dot,R_dot]=[dat.P,dat.Q,dat.R,dat.P_dot,dat.Q_dot,dat.R_dot]
        
        [Xa,Ya,Za,La_cg,Ma_cg,Na_cg,Xt,Yt,Zt,Lt_cg,Mt_cg,Nt_cg]=\
            [dat.Xa,dat.Ya,dat.Za,dat.La_cg,dat.Ma_cg,dat.Na_cg,dat.Xt,dat.Yt,dat.Zt,dat.Lt_cg,dat.Mt_cg,dat.Nt_cg]
            
        [Alpha_dot,Beta_dot,V_inf_dot]=[dat.Alpha_dot,dat.Beta_dot,dat.V_inf_dot]
        [Ny,Nz]=[dat.Ny,dat.Nz]
        [Phi_dot,Psi_dot,Theta_dot]=[dat.Phi_dot,dat.Psi_dot,dat.Theta_dot]
        [TEF,HTL,FlaperonL,Rudder,Mach,PLA]=[dat.TEF,dat.HTL,dat.FlaperonL,dat.Rudder,dat.Mach,dat.PLA]
        
        U_prime=V_inf*np.cos(Beta_rad)*np.cos(Alpha_rad)
        V_prime=V_inf*np.sin(Beta_rad)
        W_prime=V_inf*np.cos(Beta_rad)*np.sin(Alpha_rad)
        
        U=U_prime+dat.CW_body_axis[0]
        V=V_prime+dat.CW_body_axis[1]
        W=W_prime+dat.CW_body_axis[2]
        
        
        Nx=(Xa+Xt) / (Mass_kg*g)
        Ny=(Ya+Yt) / (Mass_kg*g)
        Nz=-(Za+Zt) / (Mass_kg*g)
        
        Ax=g*(Nx-np.sin(Theta_rad))
        Ay=g*(Ny+np.cos(Theta_rad)*np.sin(Phi_rad))
        Az=g*(-Nz+np.cos(Theta_rad)*np.cos(Phi_rad))
        
        U_dot=R*V-Q*W-g*np.sin(Theta_rad)+(Xa+Xt)/Mass_kg
        V_dot=-R*U+P*W+g*np.sin(Phi_rad)*np.cos(Theta_rad)+(Ya+Yt)/Mass_kg
        W_dot=Q*U-P*V+g*np.cos(Phi_rad)*np.cos(Theta_rad)+(Za+Zt)/Mass_kg
        
        U_dot_prime=(R*V-Q*W-g*np.sin(Theta_rad)+(Xa+Xt)/Mass_kg)-dat.CW_dot_body_axis[0]
        V_dot_prime=(-R*U+P*W+g*np.sin(Phi_rad)*np.cos(Theta_rad)+(Ya+Yt)/Mass_kg)-dat.CW_dot_body_axis[1]
        W_dot_prime=(Q*U-P*V+g*np.cos(Phi_rad)*np.cos(Theta_rad)+(Za+Zt)/Mass_kg)-dat.CW_dot_body_axis[2]
        
        Alpha_dot=(U_prime*W_dot_prime-W_prime*U_dot_prime)/(U_prime**2+W_prime**2)
    
        Beta_dot=(V_dot_prime*V_inf-V_prime*V_inf_dot)/(V_inf*(U_prime**2+W_prime**2)**0.5)
        
        V_inf_dot=((U_prime*U_dot_prime+V_prime*V_dot_prime+W_prime*W_dot_prime)/V_inf)
        
        P_dot=(Ixz*(Ixx-Iyy+Izz)*P*Q-(Izz*(Izz-Iyy)+Ixz**2)*Q*R+Izz*(La_cg+Lt_cg )+Ixz*(Na_cg+Nt_cg ))/(Ixx*Izz-Ixz**2)
        Q_dot=((Izz-Ixx)*P*R-Ixz*(P**2-R**2)+(Ma_cg+Mt_cg ))/Iyy
        R_dot=(((Ixx-Iyy)*Ixx+(Ixz**2))*(P*Q)-Ixz*(Ixx-Iyy+Izz)*Q*R+Ixz*(La_cg+Lt_cg )+Ixx*(Na_cg+Nt_cg))/(Ixx*Izz-Ixz**2)
        
        
        Phi_dot=P+Q*(np.sin(Phi_rad)+R*np.cos(Phi_rad))*np.tan(Theta_rad)
        Theta_dot=Q*np.cos(Phi_rad)-R*np.sin(Phi_rad)
        Psi_dot=(Q*np.sin(Phi_rad)+R*np.cos(Phi_rad))*(1/np.cos(Theta_rad))
        
        
        x_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[0]
        y_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[1]
        z_dot=np.dot(tm.earth_to_body(dat).T,[U,V,W])[2]
        
        dat.x=0;dat.y=0;dat.z=-dat.Altitude;
        
        dat.h_dot=-z_dot
        h_dot=dat.h_dot
        
        Gamma_rad =np.arcsin(h_dot/((U**2+V**2+W**2)**.5))

        Track_rad=y_dot/x_dot

        V_inf=(U_prime**2+V_prime**2+W_prime**2)**0.5
        
        Crab_Angle=Psi-Track
        
        V_wrt_g=(U**2+V**2+W**2)**.5
        V_wrt_a=(U_prime**2+V_prime**2+W_prime**2)**.5

        
        Gamma=Gamma_rad*180/np.pi
        dat.Gamma=Gamma
        Track=Track_rad*180/np.pi
        
        Qw = -P*np.cos(Alpha_rad)*np.sin(Beta_rad)+Q*np.cos(Beta_rad)-R*np.sin(Alpha_rad)*np.sin(Beta_rad)
        Rw = -P*np.sin(Alpha_rad)+R*np.cos(Alpha_rad)
        
        Throttle=PLA
        
        #total moments at cg
        
        L=La_cg+Lt_cg
        M=Ma_cg+Mt_cg
        N=Na_cg+Nt_cg
        
        
        
        Result= {"Aoa":Aoa,"Beta":Beta,"Theta":Theta,"Phi":Phi,"Psi":Psi,"P":P,"Q":Q,"R":R,"TEF":TEF,"V_wrt_a":V_wrt_a,\
                 "HTL":HTL,"FlaperonL":FlaperonL,"Rudder":Rudder,"Nx":Nx,"Ny":Ny,"Nz":Nz,"Ax":Ax,"Ay":\
                     Ay,"Az":Az,"Gamma":Gamma,"h_dot":h_dot,"U":U,"V":V,"W":W,"Throttle":Throttle,"V_wrt_g":V_wrt_g,\
                         "Alpha_dot":Alpha_dot,"Beta_dot":Beta_dot,"V_inf_dot":V_inf_dot,"Theta_dot":\
                             Theta_dot,"Phi_dot":Phi_dot,"Psi_dot":Psi_dot,"L":L,"M":M,"N":N,"P_dot":\
                                 P_dot,"Q_dot":Q_dot,"R_dot":R_dot,"U_dot":U_dot,"V_dot":V_dot,"W_dot"\
                                     :W_dot,"x_dot":x_dot,"y_dot":y_dot,"z_dot":z_dot,"Mach":Mach,"err":err,\
                                         "Track":Track,"Crab_angle":Crab_Angle,"U_prime":U_prime ,"V_prime":V_prime\
                                             ,"W_prime":W_prime,"Static_Margin":dat.Static_margin,"Xnp":dat.Xnp,"Rw":Rw,"Qw":Qw}
        
        

    

        return [Result,J,dat]
    
    @classmethod
    def main_trim(cls,dat,Trim_expression,geo):
        
        dat.Trim_expression=Trim_expression
        Trim_definition.trim_def(dat,geo)
        dat.xcode=1
        Results=cls.t(dat,geo)
    
        return Results
    
        
