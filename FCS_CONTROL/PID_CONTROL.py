# from simple_pid import PID

# =============================================================================
#     PID Control      
# =============================================================================

class Pid_Control:
    
    @staticmethod
    def pid_control(dat,geo):

        if dat.Flight_Mode=='Landing':
            
            # if dat.h_lg_wheel_right < 15 :
            #     dat.PLA=0
            
            if dat.h_lg_wheel_right < 0 : 
                dat.HTL=0   
                dat.PLA=0
                
            elif dat.h_lg_wheel_right > 0 and dat.h_lg_wheel_right < 50 and dat.HTL!=0 :
                
        
                Gain=-0.05
                # Delta_HTL=Gain*((5)-(dat.Theta))
                # dat.HTL=dat.HTL+Delta_HTL
                
                # Gain=0.0025
                # Delta_PLA=Gain*((-1)-(dat.Gamma))
                # dat.PLA=dat.PLA+Delta_PLA
                
                
                # pid = PID(-0.0005, 0, 0.0, setpoint=-1)  #setpoint =aimed value       
                # dat.HTL = pid(dat.Gamma)
                
        elif dat.Flight_Mode=='TakeOff':
            if (dat.U**2+dat.V**2+dat.W**2)**.5 > geo.lift_off_speed and dat.Theta<10: 
                Gain=-3
                dat.HTL=Gain*(10-(dat.Theta))
            else:
                dat.HTL=0
            if dat.t_total<0.1:
                dat.PLA=0
            else:
                dat.PLA=geo.Max_engine_hp
                # print(dat.HTL)
                
                
                
                
        elif dat.Flight_Mode=='NDR':
            if dat.t_total>1:
                dat.HTL=10


    