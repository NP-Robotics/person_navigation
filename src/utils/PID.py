def calculate_PID(value, gain, prev_val):
    P = gain["P"] * value
    I = 0
    D = 0
    if(prev_val):
        D = gain["D"] * (P - prev_val) 
    PID = P + I + D
    return PID