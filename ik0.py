import math

def move_to_pos(y, z):
    # b = math.atan2(y, x) * (180 / math.pi)
    l = y
    h = math.sqrt(45**2 + 45**2)
    phi = math.atan(z / l) * (180 / math.pi)
    theta = math.acos((h / 2) / 75) * (180 / math.pi)
    
    a1 = phi + theta
    a2 = phi - theta 

    return a1, a2

print(move_to_pos(45, 45))