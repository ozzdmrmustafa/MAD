from TRIM_SIM.JACOBIAN import Jacobian as Jcbn
import pandas as pd
import matplotlib.pyplot as plt
import control
# from control.matlab import *
import numpy as np
import warnings
warnings.filterwarnings("ignore")

# =============================================================================
#     Calculation of A and B matrices       
# =============================================================================

class Control:
    
    @classmethod
    def StaContMatrix(cls,dat,geo):
    
        Eqn_data=["U_dot","V_dot","W_dot","Alpha_dot","Beta_dot","V_inf_dot","P_dot","Q_dot","R_dot","Phi_dot","Theta_dot"]   
        StaCont_matrix_base=Jcbn.jacobian(dat,geo)
        Regular_Equation_Order=["","","","","","","","",""]
        StaCont_matrix=np.zeros(shape=(len(dat.Equations),len(dat.Unknowns)))
        
        j=0
        for  i in range(len(Eqn_data)):
            try:
                dat.Equations.index(Eqn_data[i])
                Regular_Equation_Order[j]=Eqn_data[i]
                j=j+1
            except:
                pass

        i=0
        Order=np.zeros(len(dat.Equations))
        for iEqn in dat.Equations:
            Order[i]=Regular_Equation_Order.index(iEqn)
            StaCont_matrix[i][:]=StaCont_matrix_base[int(Order[i])][:]
            i=i+1
            
        A=StaCont_matrix[0:len(dat.Equations),:len(dat.Equations)] 
        
        A[:,3]=A[:,3]*180/np.pi  # Theta deg to rad correction

        if len(dat.Equations)==8:
            A[:,7] =A[:,7]*180/np.pi  # Phi deg to Rad correction
        
        
        
        B=StaCont_matrix[0:len(dat.Equations),len(dat.Equations):len(dat.Unknowns)] 
        C=np.identity(len(dat.Equations))
        D=np.zeros((len(dat.Equations),len(dat.Unknowns)-len(dat.Equations)))
        
# =============================================================================
        #Controller#
        # p=[-.35+.5j,-.35-.5j,-.2+.1j,-.2-.1j,-.3+.3j,-.3-.3j,-.4+.9j,-.4-.9j]
        # K=control.statefbk.place(A, B, p)
        # A=A-B*K
        ############
# =============================================================================
        
        sys=control.statesp.ss(A,B,C,D)
        Eig=control.lti.pole(sys)
        
        Flight_char=np.zeros((7,len(dat.Equations)))
        Time_constant=np.zeros(len(dat.Equations))
        Time_to_double=np.zeros(len(dat.Equations))
        Time_to_half=np.zeros(len(dat.Equations))
        Dmp=np.zeros((len(dat.Equations)))
        Frq=np.zeros((len(dat.Equations)))
        
        for i in range(len(dat.Equations)-1):
            if Eig.imag[i]!=0 and Eig.imag[i]==-Eig.imag[i+1]:
                Frq[i]=(Eig[i]*Eig[i+1])**.5
                Dmp[i]=-(Eig[i]+Eig[i+1])/(2*Frq[i])
                Dmp[i+1]=Dmp[i]
                Frq[i+1]=Frq[i]
                
        for i in range(len(dat.Equations)):
            Time_constant[i]=-1/Eig.real[i]
            Flight_char[0][i]=Eig.real[i]
            Flight_char[1][i]=Eig.imag[i]
            Flight_char[2][i]=Dmp[i]
            Flight_char[3][i]=Frq[i]
            if Eig.real[i]>0:
                Time_to_double[i]=0.693/Eig[i]
                Flight_char[4][i]=Time_to_double[i]
                Flight_char[5][i]=Time_constant[i]
            elif Eig.real[i]<0:
                Time_to_half[i]=0.693*Time_constant[i]
                Flight_char[6][i]=Time_to_half[i]
                Flight_char[5][i]=Time_constant[i]
                        
        Flight_character=pd.DataFrame(Flight_char.T,columns=['Real_root', 'Imag_root', 'Dmp_ratio', 'Nat_Frq', 'Time_to_double', 'Time_constant', 'Time_to_half'])
        
        return[A,B,C,D,Eig,Flight_character,sys]

    @classmethod
    def FlyingQualities(cls,dat,geo):
        
        
        [A,B,C,D,Eig,Flight_character,sys]=cls.StaContMatrix(dat,geo)
        
        if len(dat.Equations)==4 or 'V_inf_dot' in dat.Equations:
            return [A,B,C,D,Eig,Flight_character]

        
    
        X = [x.real for x in Eig]
        Y = [x.imag for x in Eig]
        

  

        """plt.scatter(X,Y, color='red')
        plt.minorticks_on()
        plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
        plt.title(dat.Unknown_type)
        # plt.xlim([-21,1])
        plt.show()"""
        
        
        
        T=np.arange(0,10,0.1)
    
    
        ##Inputs and outputs is in radian##
        
        #['U', 'W', 'Q', 'Theta', 'V', 'P', 'R', 'Phi']#
        X0=[0,0,0,0,0,0,0.0,0]
         
        u=np.zeros((4,len(T)))
        u[0]=u[0]+0 # HT Input 
        u[1]=u[1]+0 # Throttle Input
        u[2]=u[2]+0 # Flaperon_Left Input
        u[3]=u[3]+0 # Rudder Input
        
        Ti, youti = control.timeresp.initial_response(sys, T, X0)
        # Tf,youtf = control.timeresp.forced_response(sys, T, u, X0)
        
        #control.pzmap.pzmap(sys, Plot=True, grid=False, title='Pole Zero Map')
        #PQR radyan others degree
        names=['U', 'W', 'Q', 'Theta_rad', 'V', 'P', 'R', 'Phi_rad']
        
        """
        for i in range(8):
            # plt.plot(Tf,youtf[i],label=names[i],color='r')
            plt.plot(Ti,youti[i],label=names[i],color='r')
            plt.legend(loc="upper right")
            plt.minorticks_on()
            plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
            plt.show()
        #########
        """

        return[A,B,C,D,Eig,Flight_character]
    @classmethod
    def control_inputs(cls,dat,geo):
        
        # =============================================================================
        #  Calc A and B matrix       
        # =============================================================================

        
            # # # =============================================================================
            dat.Unknown_type='Body'
            dat.Unknowns=['U', 'W', 'Q', 'Theta','HTL','PLA']
            dat.Equations=["U_dot","W_dot","Q_dot","Theta_dot"]

            [dat.A_body_long,dat.B_body_long,dat.C_body_long,dat.D_body_long,dat.Eig_body_long,dat.Flight_character_body_long]=cls.FlyingQualities(dat,geo)
            # =============================================================================
        
            # # =============================================================================
            dat.Unknown_type='Body'
            dat.Unknowns=['V', 'P', 'R', 'Phi','FlaperonL','Rudder']
            dat.Equations=["V_dot","P_dot","R_dot","Phi_dot"]
              
            [dat.A_body_lat,dat.B_body_lat,dat.C_body_lat,dat.D_body_lat,dat.Eig_body_lat,dat.Flight_character_body_lat]=cls.FlyingQualities(dat,geo)
            # # =============================================================================
        
            # =============================================================================
            dat.Unknown_type='Body'
            dat.Unknowns=['U', 'W', 'Q', 'Theta', 'V', 'P', 'R', 'Phi', 'HTL','PLA','FlaperonL','Rudder']
            dat.Equations=["U_dot","W_dot","Q_dot","Theta_dot","V_dot","P_dot","R_dot","Phi_dot"]
              
            [dat.A_body,dat.B_body,dat.C_body,dat.D_body,dat.Eig_body,dat.Flight_character_body]=cls.FlyingQualities(dat,geo)
            # =============================================================================
            
            # # ?????????????????????????????????????????????????????????????????
            # # =============================================================================
            dat.Unknown_type='Wind'
            dat.Unknowns=['V_inf','Aoa', 'Q', 'Theta', 'Beta', 'P', 'R', 'Phi', 'HTL','PLA','FlaperonL','Rudder']
            dat.Equations=["V_inf_dot","Alpha_dot","Q_dot","Theta_dot","Beta_dot","P_dot","R_dot","Phi_dot"]
              
            [dat.A_wind,dat.B_wind,dat.C_wind,dat.D_wind,dat.Eig_wind,dat.Flight_character_wind]=cls.FlyingQualities(dat,geo)
            # # ============================================================================
        
            # dat.Unknown_type='Wind'
            # dat.Unknowns=['V_inf','Aoa', 'Q', 'Theta', 'HTL','PLA']
            # dat.Equations=["V_inf_dot","Alpha_dot","Q_dot","Theta_dot"]
              
            # [dat.A_wind_long,dat.B_wind_long,dat.C_wind_long,dat.D_wind_long,dat.Eig_wind_long,dat.Flight_character_wind_long]=cls.FlyingQualities(dat,geo)
            # =============================================================================
            # dat.Unknown_type='Wind'
            # dat.Unknowns=['Beta', 'P', 'R', 'Phi','FlaperonL','Rudder']
            # dat.Equations=["Beta_dot","P_dot","R_dot","Phi_dot"]
              
            # [dat.A_wind_lat,dat.B_wind_lat,dat.C_wind_lat,dat.D_wind_lat,dat.Eig_wind_lat,dat.Flight_character_wind_lat]=cls.FlyingQualities(dat)
            # # =============================================================================
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
