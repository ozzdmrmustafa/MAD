import numpy as np
from TRIM_SIM.TRANS_MATRIX import Transformation_Matrices as tm
# import sys
# Engine.engine

class Landing_gear:
    
    # Vertical_damping_coeff=dat.Mass_kg*3 #SI unit
    Rolling_friction_coefficient=0.012 # or Table
    
    Brake_percentage=0
    Weight_carring_percentage_right=.425 # Takeoff Stiffness
    Weight_carring_percentage_left=.425
    Weight_carring_percentage_nose=.15
    
# =============================================================================
    braking_friction_coefficient=0.8 #Table(Forward velocity(U))
    ground_condition="dry"
    Maximum_vertical_stroke=0.2 #(m)  it can be calculated from energy equation
    
    @classmethod
    def vertical_stiffness(cls,dat):
        
        
        if dat.Flight_Mode=='Landing':
            cls.Brake_percentage= (0.3 / cls.braking_friction_coefficient)*100 #30 ile 50 arasında değişir \ mil std 3013 total friction coeff = 0.3 dry
            cls.Weight_carring_percentage_right=.33 # Landing Stiffness
            cls.Weight_carring_percentage_left=.33
            cls.Weight_carring_percentage_nose=.33

        Wstatic_nose_wheel=dat.Mass_kg*9.81*cls.Weight_carring_percentage_nose # (N) Nose LG carries %10 of the total weight
        Wstatic_right_wheel=dat.Mass_kg*9.81*cls.Weight_carring_percentage_right # (N)  Right LG carries %45 of the total weight
        Wstatic_left_wheel=dat.Mass_kg*9.81*cls.Weight_carring_percentage_left # (N)  Left LG carries %45 of the total weight
 
    
        Vertical_stiffness_right=Wstatic_right_wheel*3/cls.Maximum_vertical_stroke 
        Vertical_stiffness_left=Wstatic_left_wheel*3/cls.Maximum_vertical_stroke 
        Vertical_stiffness_nose= Wstatic_nose_wheel*3/cls.Maximum_vertical_stroke  ############################????
        
        return [Vertical_stiffness_right,Vertical_stiffness_left,Vertical_stiffness_nose]

    @classmethod
    def partial_landing_gear(cls,dat,Location_lg_body,Vertical_stiffness,Weight_carring_percentage):
        
        
        
        [Alpha_rad,Beta_rad]=[dat.Aoa*np.pi/180,dat.Beta*np.pi/180]
        
        # =============================================================================
        # To use Wind unknowns (Aoa,Beta)       
        # =============================================================================
        if dat.Unknown_type == "Wind":
            
            
            [Alpha_rad,Beta_rad]=[dat.Aoa*np.pi/180,dat.Beta*np.pi/180]
            
            U_prime=dat.V_inf*np.cos(Beta_rad)*np.cos(Alpha_rad) # Crosswind etkisini dahil etmek için bu denklemden rüzgarın hızını çıkar öyle (U,V,W)(yere göre) yu bul.
            V_prime=dat.V_inf*np.sin(Beta_rad)
            W_prime=dat.V_inf*np.cos(Beta_rad)*np.sin(Alpha_rad)
            
            
            dat.CW_body_axis=np.dot(tm.earth_to_body(dat),dat.CW_earth_axis)
            
            U=U_prime+dat.CW_body_axis[0]
            V=V_prime+dat.CW_body_axis[1]
            W=W_prime+dat.CW_body_axis[2]
        
        # =============================================================================
        # To use Body unknowns (U,V,W)       
        # =============================================================================
        if dat.Unknown_type == "Body":
            U=dat.U;V=dat.V;W=dat.W
        # =============================================================================
        #         
        # =============================================================================

        Location_lg_wheel=np.dot(tm.wheel_to_body(dat).T,Location_lg_body)# Location of tip of the landing gear with respect to cg in wheel axis

        Wheel_z_lg_distance=Location_lg_wheel[2]

        h_lg_wheel=dat.Altitude-Wheel_z_lg_distance
        
        Velocity_body_of_tip_point=[U,V,W]+np.cross([dat.P,dat.Q,dat.R],Location_lg_body)
        Velocity_wheel_of_tip_point=np.dot(tm.wheel_to_body(dat).T,Velocity_body_of_tip_point)# Velocity of tip of the landing gear in earth axis
        
        Wheel_x_lg_velocity=Velocity_wheel_of_tip_point[0]
        Wheel_y_lg_velocity=Velocity_wheel_of_tip_point[1]
        Wheel_z_lg_velocity=Velocity_wheel_of_tip_point[2]
        
        # print(Wheel_y_lg_velocity)
        # print(Wheel_x_lg_velocity)

        
        
        Slip_angle=np.arctan2(Wheel_y_lg_velocity,Wheel_x_lg_velocity)*180/np.pi  # Cornering_Angle
        if 0==np.mod(Slip_angle,180):
            Slip_angle=0
            
        # print(Slip_angle)
        
        """        
        if cls.ground_condition=="dry":
            
            if Slip_angle<40 and Slip_angle>=0:
               # =============================================================================
               Side_friction_coeff=-(Slip_angle**2-40*Slip_angle)/500 #Table(Slip_angle)      
               # =============================================================================
            elif Slip_angle>-40 and Slip_angle<0:
                # =============================================================================
               Side_friction_coeff=(Slip_angle**2+40*Slip_angle)/500 #Table(Slip_angle)    
            else:
                sys.exit("Landing Gear Slip angle is out of the realistic range.. %d" % Slip_angle)
               # =============================================================================
        if cls.ground_condition=="wet":
            # =============================================================================
            Side_friction_coeff=-((Slip_angle**2-40*Slip_angle)/500)*47/80 #Table(Slip_angle)      
            # =============================================================================
        """
        # =============================================================================
        Side_friction_coeff=0 # Üstteki comment e alınmış satırları açarsan bunu sil
        # =============================================================================
        

        if h_lg_wheel >= 0:
        
            Xlanding=0
            Ylanding=0
            Zlanding=0
            Mx_landing=0
            My_landing=0
            Mz_landing=0
            
        else:
            
            Vertical_damping_coeff=dat.Gross_weight*2.5 #SI unit
            Wheel_Z_force_landing=Vertical_stiffness*h_lg_wheel-Vertical_damping_coeff*Wheel_z_lg_velocity
            
            # print("damping force",cls.Vertical_damping_coeff*Wheel_z_lg_velocity)
            # print("friction force",Vertical_stiffness*h_lg_wheel)
            
            
            if cls.Brake_percentage==0:
                Wheel_X_force_landing=cls.Rolling_friction_coefficient*Wheel_Z_force_landing
            else:
                Wheel_X_force_landing=(cls.Brake_percentage/100)* cls.braking_friction_coefficient*Wheel_Z_force_landing+cls.Rolling_friction_coefficient*Wheel_Z_force_landing
            
            Wheel_max_Y_force_landing=Side_friction_coeff*Wheel_Z_force_landing
            
            Wheel_Y_aero_force=np.dot(tm.wheel_to_body(dat).T,[dat.Xaero,dat.Yaero,dat.Zaero])[1]
            
            
            if abs(Wheel_Y_aero_force) < abs(Wheel_max_Y_force_landing):
                
                Wheel_Y_force_landing=-Wheel_Y_aero_force*Weight_carring_percentage
            else:
                Wheel_Y_force_landing=Wheel_max_Y_force_landing
            
            # print("a",Wheel_Y_force_landing)
            
            F_landing_wheel_arr=[Wheel_X_force_landing,Wheel_Y_force_landing,Wheel_Z_force_landing]
            M_landing_wheel_arr=np.cross(Location_lg_wheel,F_landing_wheel_arr)
            
            
            F_landing_body_arr=np.dot(tm.wheel_to_body(dat),F_landing_wheel_arr)
            M_landing_body_arr=np.dot(tm.wheel_to_body(dat),M_landing_wheel_arr)
            
            Xlanding=F_landing_body_arr[0]
            Ylanding=F_landing_body_arr[1]
            
            # print("b",Ylanding)
            # print("c",dat.Yaero)
            Zlanding=F_landing_body_arr[2]
            Mx_landing=M_landing_body_arr[0]
            My_landing=M_landing_body_arr[1]
            Mz_landing=M_landing_body_arr[2]
 
        
        return [Xlanding,Ylanding,Zlanding,Mx_landing,My_landing,Mz_landing,h_lg_wheel]
    
    @classmethod
    def landing_gear(cls,dat,geo):
        
        [Vertical_stiffness_right,Vertical_stiffness_left,Vertical_stiffness_nose]=cls.vertical_stiffness(dat)
        
        
        # [Xlanding_R,Ylanding_R,Zlanding_R,Mx_landing_R,My_landing_R,Mz_landing_R,h_lg_wheel_right]\
        #     =cls.partial_landing_gear(dat,(geo.Location_lg_body_right-np.asarray([dat.x_cg,dat.y_cg,dat.z_cg]))\
        #                               ,Vertical_stiffness_right,cls.Weight_carring_percentage_right)
        

        # [Xlanding_L,Ylanding_L,Zlanding_L,Mx_landing_L,My_landing_L,Mz_landing_L,h_lg_wheel_left]=\
        #     cls.partial_landing_gear(dat,geo.Location_lg_body_left-np.asarray([dat.x_cg,dat.y_cg,dat.z_cg])\
        #                               ,Vertical_stiffness_left,cls.Weight_carring_percentage_left)
                
                
        # [Xlanding_Nose,Ylanding_Nose,Zlanding_Nose,Mx_landing_Nose,My_landing_Nose,Mz_landing_Nose,h_lg_wheel_nose]\
        #     =cls.partial_landing_gear(dat,geo.Location_lg_body_nose-np.asarray([dat.x_cg,dat.y_cg,dat.z_cg])\
        #                               ,Vertical_stiffness_nose,cls.Weight_carring_percentage_nose)
                
        [Xlanding_R,Ylanding_R,Zlanding_R,Mx_landing_R,My_landing_R,Mz_landing_R,h_lg_wheel_right]\
            =cls.partial_landing_gear(dat,geo.Location_lg_body_right\
                                      ,Vertical_stiffness_right,cls.Weight_carring_percentage_right)
        

        [Xlanding_L,Ylanding_L,Zlanding_L,Mx_landing_L,My_landing_L,Mz_landing_L,h_lg_wheel_left]=\
            cls.partial_landing_gear(dat,geo.Location_lg_body_left\
                                      ,Vertical_stiffness_left,cls.Weight_carring_percentage_left)
                
                
        [Xlanding_Nose,Ylanding_Nose,Zlanding_Nose,Mx_landing_Nose,My_landing_Nose,Mz_landing_Nose,h_lg_wheel_nose]\
            =cls.partial_landing_gear(dat,geo.Location_lg_body_nose\
                                      ,Vertical_stiffness_nose,cls.Weight_carring_percentage_nose)
                
        
        # print("n",h_lg_wheel_nose)
        # print("r",h_lg_wheel_right)
        # print("l",h_lg_wheel_left)
        
        dat.h_lg_wheel_right=h_lg_wheel_right
        
        dat.Xlanding=Xlanding_R+Xlanding_L+Xlanding_Nose
        dat.Ylanding=Ylanding_R+Ylanding_L+Ylanding_Nose

        
        # print("a",Ylanding_R)
        # print("b",dat.Yaero)
        # print("c",Ylanding_L)
        # print("d",dat.Yaero)
        # print("e",Ylanding_Nose)
        # print("f",dat.Yaero)
        # print("g",dat.Ylanding)


        dat.Zlanding=Zlanding_R+Zlanding_L+Zlanding_Nose
        
        dat.Mx_landing=Mx_landing_R+Mx_landing_L+Mx_landing_Nose
        dat.My_landing=My_landing_R+My_landing_L+My_landing_Nose
        dat.Mz_landing=Mz_landing_R+Mz_landing_L+Mz_landing_Nose
