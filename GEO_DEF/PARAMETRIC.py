import numpy as np

#Parametric.parametric

class Parametric:

    def parametric(S_ref,Taper_ratio,Aspect_ratio,LE_sweep_deg,Incidence_deg,Dihedral_deg,Twist_deg,n,Span_percent_cumulative,LE_sweep_deg_arr,Dihedral_deg_arr,Twist_deg_arr,Taper_ratio_arr,Taper_manuel):
        
        
        
        Span_percent_part=np.zeros(shape=n)
        Taper_part=np.zeros(shape=n)
        
        Span_percent_part[0]=Span_percent_cumulative[0]
        Span=(S_ref*Aspect_ratio)**.5   ## Projected wing length over Y body axis
    
    
        
        for i in range (1,n):
            Span_percent_part[i]=Span_percent_cumulative[i]-Span_percent_cumulative[i-1]
        Span_length_part=Span_percent_part*(Span/2)
            
        #######################################################    
        
        if Taper_manuel=="off":
            #for a straight surface Taper ratio should be like follows:
            
            Taper_part[0]=1-(((1-Taper_ratio)/(Span/2))*Span_length_part[0])
            
            for i in range (1,n):
                Taper_part[i]=1-(((Span_length_part[i])/Span_length_part[i-1])*((1/Taper_part[i-1])-1))
                
            Taper_ratio_arr=[]
            
            for i in range (n):
                Taper_ratio_arr.append(Taper_part[i])
        
        
        
        # #İf you want to define taper ratio array manually you can activate following line
        # #################################################################################
        # Taper_ratio_arr=np.array([0.84,0.75,0.6,0.5])
        # #################################################################################
        # # However,to check, you should be sure that "multiplication of all partial taper ratio's should give total taper ratio"
        
        
    
        #######################################################
        Span_arr=Span_length_part
    
        S_mult_total=0
        S_mult_arr=np.zeros(shape=n)
        MAC_mult=0
        S=np.zeros(shape=n)
        C=np.zeros(shape=n+1)
        MAC=np.zeros(shape=n)
    
        Xe=np.zeros(shape=n+1)
        Ye=np.zeros(shape=n+1)
        Ze=np.zeros(shape=n+1)
    
    
        
        Taper=Taper_ratio_arr
    
    
        for i in range(1,n+1):
            
            Taper_perm=1
            for j in range(i-1):
                Taper_perm=Taper[j]*Taper_perm
                
                
            S_mult_arr[i-1]=Taper_perm*(Span_arr[i-1]/Span_arr[0])*((1+Taper[i-1])/(1+Taper[0]))
            
        for i in range(n):
            S_mult_total=S_mult_total+S_mult_arr[i]
            
        S[0]=S_ref/(S_mult_total*2)
    
        for i in range(1,n):
            S[i]=S[i-1]*Taper[i-1]*(Span_arr[i]/Span_arr[i-1])*((1+Taper[i])/(1+Taper[i-1]))
            
            
        for i in range(n):
            C[i]=2*S[i]/(Span_arr[i]*(1+Taper[i]))
        C[n]=Taper[n-1]*C[n-1]
    
        for i in range(n):
            
            MAC[i]=(2/3)*C[i]*(1+Taper[i]+Taper[i]**2)/(1+Taper[i])
            MAC_mult=MAC_mult+MAC[i]*S[i]
            
            
            Xe[i+1]=Span_arr[i]*np.tan(LE_sweep_deg_arr[i]*np.pi/180)+Xe[i]
            Ye[i+1]=Span_arr[i]+Ye[i]
            Ze[i+1]=Span_arr[i]*np.tan(Dihedral_deg_arr[i]*np.pi/180)+Ze[i]
            
        MAC_tot=MAC_mult/(S_ref/2)
            
    
        return [Xe,Ye,Ze,C,Twist_deg_arr,MAC_tot,Span_arr,Span,Taper_ratio_arr]
