import math

def inverse_kinematics(x, y, l1=45, l2=45):
    r = math.sqrt(x**2 + y**2)

    if r > (l1 + l2) or r < abs(l1 - l2):
        return None, None

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    theta2_a = math.acos(cos_theta2)
    theta2_b = -math.acos(cos_theta2)
    
    k1_a = l1 + l2 * math.cos(theta2_a)
    k2_a = l2 * math.sin(theta2_a)
    theta1_a = math.atan2(y, x) - math.atan2(k2_a, k1_a)
    
    k1_b = l1 + l2 * math.cos(theta2_b)
    k2_b = l2 * math.sin(theta2_b)
    theta1_b = math.atan2(y, x) - math.atan2(k2_b, k1_b)

    # theta1_a_deg = math.degrees(theta1_a)
    theta2_a_deg = math.degrees(theta2_a)
    theta1_b_deg = math.degrees(theta1_b)
    # theta2_b_deg = math.degrees(theta2_b)

    # theta1_a_deg = abs(int(theta1_a_deg))
    theta1_b_deg = abs(int(180 - theta1_b_deg))

    theta2_a_deg = abs(int(180 - theta2_a_deg)) + 60
    # theta2_b_deg = abs(abs(int(180 - theta2_b_deg)))
    
    return theta1_b_deg, theta2_a_deg

# 40,20 => 86, 59
# 45,20 => 80, 66
# 50,20 => 75, 73
# 55,20 => 69, 81
for axes in [(45, 45), (40, 20), (45, 20), (50, 20), (55, 20)]:
    a, b = inverse_kinematics(axes[0], axes[1])
    print(axes, ':', a, b)
