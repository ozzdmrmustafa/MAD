from AERODYNAMICS.AVL import Avl 
from WEIGHT_BALANCE.WEIGHTMODULE import WeightModule
from LANDINGSYSTEMS.LANDINGGEAR import Landing_gear
from PROPULSION.ENGINE import Engine

# GetCoefficient.get_coefficient

class GetCoefficient:

    @staticmethod
    def get_coefficient(dat,geo):

        #Mach     =0   #used in aerodynamic coefficient calculation for compresibility  # if it is below 0.2 it doesnt affect the result so much
        #if Mach is not below 0.2 then deactivite above line
        
        WeightModule.weight_module(dat,geo)     

        Avl.avl(dat,geo)

        Engine.engine(dat,geo)

        Landing_gear.landing_gear(dat,geo)
        
        if dat.Flight_Mode=='Landing':
            if dat.h_lg_wheel_right >= 0:
                dat.Xt=dat.Thrust
            else: 
                dat.Xt=0
        else:
            dat.Xt=dat.Thrust
            
        dat.Yt=0
        dat.Zt=0 
        
        # print("Xt",dat.Xt)
    
        dat.Lt_cg=0
        dat.Mt_cg=0
        dat.Nt_cg=0 	
        
        dat.Xa=dat.Xaero+dat.Xlanding
        dat.Ya=dat.Yaero+dat.Ylanding
        
        # print("******************t",dat.Xt)
        # print("******************a",dat.Xaero)
        # print("******************l",dat.Xlanding)

        dat.Za=dat.Zaero+dat.Zlanding
        
        dat.La_cg=dat.Laero_cg+dat.Mx_landing
        dat.Ma_cg=dat.Maero_cg+dat.My_landing
        dat.Na_cg=dat.Naero_cg+dat.Mz_landing
        
        
        # return [Xa,Ya,Za,La_cg,Ma_cg,Na_cg,Xt,Yt,Zt,Lt_cg,Mt_cg,Nt_cg,Mach,rho,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]