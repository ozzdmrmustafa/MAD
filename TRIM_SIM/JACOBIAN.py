import numpy as np
from DYNAMICEQNS.EOM import Eom

# Jacobian.jacobian

class Jacobian:
    perturb=0.1
    
    @classmethod
    def perturbation(cls,dat,index,Perturbation_coeff):
        executable=('''dat.%s=dat.%s+%f*cls.perturb''' % (index,index,Perturbation_coeff) )
        exec(executable)
        
    @classmethod
    def jacobian(cls,dat,geo):
        
# =============================================================================
#             
# =============================================================================
        s=0
        F=Eom.eom(dat,geo)

        J = np.zeros(shape=(len(dat.Equations),len(dat.Unknowns)))

        # for deriv_variable in dat.xt():
        for deriv_variable in dat.Unknowns:

            
            cls.perturbation(dat,deriv_variable,1)

            F_perturb_forward=Eom.eom(dat,geo)

            cls.perturbation(dat,deriv_variable,-2)
            
            F_perturb_backward=Eom.eom(dat,geo)
               
            F_deriv=(F_perturb_forward-F_perturb_backward)/(2*cls.perturb)

            cls.perturbation(dat,deriv_variable,1)
           
# =============================================================================
#             
# =============================================================================

            for i in range(len(dat.Equations)):
                
                J[i,s]=(F_deriv[i])
 
            s=s+1

        return np.asarray(J)
