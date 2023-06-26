import numpy as np

class Transformation_Matrices:
    
    def earth_to_body(dat):
        
        Phi_rad=dat.Phi*np.pi/180
        Theta_rad=dat.Theta*np.pi/180
        Psi_rad=dat.Psi*np.pi/180
        
        Trans_E2B=np.array([\
                            
                            [np.cos(Theta_rad)*np.cos(Psi_rad),np.cos(Theta_rad)*np.sin(Psi_rad),-np.sin(Theta_rad)],\
                                
                            [-np.cos(Phi_rad)*np.sin(Psi_rad)+np.sin(Phi_rad)*np.sin(Theta_rad)*np.cos(Psi_rad),\
                            np.cos(Phi_rad)*np.cos(Psi_rad)+np.sin(Phi_rad)*np.sin(Theta_rad)*np.sin(Psi_rad),\
                            np.sin(Phi_rad)*np.cos(Theta_rad)],\
                                
                            [np.sin(Phi_rad)*np.sin(Psi_rad)+np.cos(Phi_rad)*np.sin(Theta_rad)*np.cos(Psi_rad),\
                            -np.sin(Phi_rad)*np.cos(Psi_rad)+np.cos(Phi_rad)*np.sin(Theta_rad)*np.sin(Psi_rad),\
                            np.cos(Phi_rad)*np.cos(Theta_rad)]])
        
        return Trans_E2B
    
    def wheel_to_body(dat):
        
        Phi_rad=dat.Phi*np.pi/180
        Theta_rad=dat.Theta*np.pi/180
        Psi_rad=0
        
        L_Phi=[[1,0,0],[0,np.cos(Phi_rad),np.sin(Phi_rad)],[0,-np.sin(Phi_rad),np.cos(Phi_rad)]]
        L_Theta=[[np.cos(Theta_rad),0,-np.sin(Theta_rad)],[0,1,0],[np.sin(Theta_rad),0,np.cos(Theta_rad)]]
        
    
        
        Trans_Wh2B=np.array([\
                            
                            [np.cos(Theta_rad)*np.cos(Psi_rad),np.cos(Theta_rad)*np.sin(Psi_rad),-np.sin(Theta_rad)],\
                                
                            [-np.cos(Phi_rad)*np.sin(Psi_rad)+np.sin(Phi_rad)*np.sin(Theta_rad)*np.cos(Psi_rad),\
                            np.cos(Phi_rad)*np.cos(Psi_rad)+np.sin(Phi_rad)*np.sin(Theta_rad)*np.sin(Psi_rad),\
                            np.sin(Phi_rad)*np.cos(Theta_rad)],\
                                
                            [np.sin(Phi_rad)*np.sin(Psi_rad)+np.cos(Phi_rad)*np.sin(Theta_rad)*np.cos(Psi_rad),\
                            -np.sin(Phi_rad)*np.cos(Psi_rad)+np.cos(Phi_rad)*np.sin(Theta_rad)*np.sin(Psi_rad),\
                            np.cos(Phi_rad)*np.cos(Theta_rad)]])
        
        return Trans_Wh2B
    
    
    
    
    def wind_to_body(dat):
        
        Aoa_rad=dat.Aoa*np.pi/180
        Beta_rad=dat.Beta*np.pi/180
        
        Trans_W2B=np.array([\
                            
                            [np.cos(Aoa_rad)*np.cos(Beta_rad),-np.cos(Aoa_rad)*np.sin(Beta_rad),-np.sin(Aoa_rad)],\
                            [np.sin(Beta_rad),np.cos(Beta_rad),0],\
                            [np.sin(Aoa_rad)*np.cos(Beta_rad),-np.sin(Aoa_rad)*np.sin(Beta_rad),np.cos(Aoa_rad)]])
        return Trans_W2B
    
    
    def stability_to_body(dat):
        
        Aoa_rad=dat.Aoa*np.pi/180
        
        Trans_S2B=np.array([\
                            [np.cos(Aoa_rad),0,-np.sin(Aoa_rad)],\
                            [0,1,0],\
                            [np.sin(Aoa_rad),0,np.cos(Aoa_rad)]])
        return Trans_S2B
    
    
        
        
