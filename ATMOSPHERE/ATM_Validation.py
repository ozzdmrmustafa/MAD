import numpy as np
from matplotlib import pyplot as plt

rho_r=[]
temperature_r=[]

Altitude=5 # m
for Altitude in np.arange(1,19500):
    rho_std=1.225*0.00194
    B=0
    A=288.15
    Altitude_ft=Altitude*3.28
       
    if Altitude_ft < 36089.2:
       A=288.15
       B=-1.9812*(10)**(-3)
       I=0.24179285
       J=-1.6624675*(10)**(-6)
       L=4.2558797
       rho_std=(I+J*Altitude_ft)**L
       
    #between from 36,0892 to 65,6168 feet altitude
    elif Altitude_ft < 65616.8:
       A=216.65
       B=0
       M=4.0012122*(10)**(-3)
       N=-48.063462*(10)**(-6)
       rho_std=(M)*np.exp(N*Altitude_ft)
                      
    T_std=A+B*Altitude_ft
    T_resultant=T_std+0   #Kelvin
    rho_resultant=rho_std/(1+(0)/T_std)   #slug/ft^3
       
    
    rho_r.append(rho_resultant*515.379) #kg/m^3
    temperature_r.append(T_resultant) #Kelvin
    
plt.plot(temperature_r,np.arange(1,19500),color='g')

plt.minorticks_on()
plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
plt.ylabel("Altitude (meters)")
plt.xlabel("Temperature (Kelvin)")
plt.title('Temperature Model')
plt.show()

plt.plot(rho_r,np.arange(1,19500),color='g')

plt.minorticks_on()
plt.grid(b=True, which='both', color='b', linestyle='-.', lw =0.2 )
plt.ylabel("Altitude (meters)")
plt.xlabel("Density (kg/m3)")
plt.title('Density Model')
plt.show()


