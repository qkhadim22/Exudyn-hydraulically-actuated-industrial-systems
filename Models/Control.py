from Models.Container import *



# Control signal 1
def uref(t):
    
    
    #Lets comment this part and call 
    Lifting_Time_Start_1    = 1.00          # Start of lifting mass, m
    Lifting_Time_End_1      = 2.00          # End of lifting mass, m
    Lowering_Time_Start_1   = 3           # Start of lowering mass, m
    Lowering_Time_End_1     = 7           # End of lowering mass, m
    Lowering_Time_Start_2   = 10          # Start of lowering mass, m
    Lowering_Time_End_2     = 14.5            # End of lowering mass, m
    Lowering_Time_Start_3   = 16         # Start of lowering mass, m
    Lowering_Time_End_3     = 17           # End of lowering mass, m
    Lowering_Time_Start_4   = 18         # Start of lowering mass, m
    Lowering_Time_End_4     = 19           # End of lowering mass, m

    if Lifting_Time_Start_1 <= t < Lifting_Time_End_1:
        u = -10
    elif Lowering_Time_Start_1 <= t < Lowering_Time_End_1:
        u = 10
    elif Lowering_Time_Start_2 <= t < Lowering_Time_End_2:
         u = -10
    elif Lowering_Time_Start_3 <= t < Lowering_Time_End_3:
         u = 10
    elif Lowering_Time_Start_4 <= t < Lowering_Time_End_4:
         u = -10
    else:
        u = 0
    
    return u


# Control signal 1
def uref_1(t):
    
    
    #Lets comment this part and call 
    Lifting_Time_Start_1    = 2          # Start of lifting mass, m
    Lifting_Time_End_1      = 6          # End of lifting mass, m
    Lowering_Time_Start_1   = 8           # Start of lowering mass, m
    Lowering_Time_End_1     = 10.7     # End of lowering mass, m
    Lowering_Time_Start_2   = 13          # Start of lowering mass, m
    Lowering_Time_End_2     = 16            # End of lowering mass, m
    Lowering_Time_Start_3   = 17        # Start of lowering mass, m
    Lowering_Time_End_3     = 18          # End of lowering mass, m
    Lowering_Time_Start_4   = 18.5         # Start of lowering mass, m
    Lowering_Time_End_4     = 19           # End of lowering mass, m

    if Lifting_Time_Start_1 <= t < Lifting_Time_End_1:
        u = 10
    elif Lowering_Time_Start_1 <= t < Lowering_Time_End_1:
        u = -10
    elif Lowering_Time_Start_2 <= t < Lowering_Time_End_2:
         u = 10
    elif Lowering_Time_Start_3 <= t < Lowering_Time_End_3:
         u = -10
    # elif Lowering_Time_Start_4 <= t < Lowering_Time_End_4:
    #      u = -10
    else:
        u = 0
    
    return u


# Control signal 2
def uref_2(t):
    
   Lifting_Time_Start_1  = 1.0          # Start of lifting mass, m
   Lifting_Time_End_1    = 3.0            # End of lifting mass, m
   Lowering_Time_Start_1 = 4.0         # Start of lowering mass, m
   Lowering_Time_End_1   = 6.6           # End of lowering mass, m
   Lowering_Time_Start_2 = 8         # Start of lowering mass, m
   Lowering_Time_End_2   = 10           # End of lowering mass, m
   Lowering_Time_Start_3 = 12         # Start of lowering mass, m
   Lowering_Time_End_3 = 15          # End of lowering mass, m
   Lowering_Time_Start_4 = 16         # Start of lowering mass, m
   Lowering_Time_End_4 = 18          # End of lowering mass, m

   if Lifting_Time_Start_1 <= t < Lifting_Time_End_1:
       u = 10
   elif Lowering_Time_Start_1 <= t < Lowering_Time_End_1:
       u = -10
   elif Lowering_Time_Start_2 <= t < Lowering_Time_End_2:
       u = 10
   elif Lowering_Time_Start_3 <= t < Lowering_Time_End_3:
       u = -10
   elif Lowering_Time_Start_4 <= t < Lowering_Time_End_4:
         u = 10
   else:
       u = 0
    #u = ExpData[t,5]

   return u


# Control signal 1
def Pump(t):
    
    """
    Lets comment this part and call 
    Lifting_Time_Start_1 = 0.5          # Start of lifting mass, m
    Lifting_Time_End_1 = 0.7           # End of lifting mass, m
    Lowering_Time_Start_1 = 0.8         # Start of lowering mass, m
    Lowering_Time_End_1 = 6.8           # End of lowering mass, m
    Lowering_Time_Start_2 = 7.0         # Start of lowering mass, m
    Lowering_Time_End_2 = 7.2            # End of lowering mass, m
    Lowering_Time_Start_3 = 7.5         # Start of lowering mass, m
    Lowering_Time_End_3 = 9.5           # End of lowering mass, m

    if Lifting_Time_Start_1 <= t < Lifting_Time_End_1:
        u = -1
    elif Lowering_Time_Start_1 <= t < Lowering_Time_End_1:
        u = 1
    elif Lowering_Time_Start_2 <= t < Lowering_Time_End_2:
        u = -1
    elif Lowering_Time_Start_3 <= t < Lowering_Time_End_3:
        u = 1
    else:
        u = 0
    """

    #pP = ExpData[t,9]*1e5
    pP = 100e5
   
    return pP