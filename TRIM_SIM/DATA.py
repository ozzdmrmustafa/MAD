class data:
    

# =============================================================================
#   Tclass Constants
# =============================================================================

    max_num_iter=6
    tolerance=1e-2 # it should be 1e-3
    ksi=1
    print_info=True
    
# =============================================================================
# 
# =============================================================================

    

    @classmethod
    def xt(cls):   # xt çağrıldığı zaman variable ların güncel değerlerini çeker 
        y=eval(cls.xt_final)
        return y
    
    
    @classmethod
    def x_vrb(cls): # x çağrıldığı zaman xt nin güncel değerlerini çeker
        [*y]=[*cls.xt().values()]
        return y
    
    
    @classmethod
    def xt_to_variables_and_x(cls):
        
        k=[]
        
        for i in cls.xt().keys():
            k.append(i)
        for i in range(0,len(cls.Unknowns)):
            exec("cls.%s=%f" % (k[i],cls.x_vrb()[i]))
            
    @classmethod
    def x_temp_to_variables_and_xt_and_x(cls):
        
        k=[]
        
        for i in cls.xt().keys():
            k.append(i)
        for i in range(0,len(cls.Unknowns)):
            exec("cls.%s=%f" % (k[i],cls.x_temp[i]))
        cls.x_vrb() # x_temp değerlerini variableye atadıktan sonra variable değerlerini de xt ve x e geri atama yapar


        
    