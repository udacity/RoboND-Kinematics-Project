# Calculate R0_3 knowing the value of q1,q2,q3
T0_3 = T0_3.evalf(subs=({q1: q1, q2: q2, q3: q3}))
R0_3 = T0_3[0:2,0:2]

# Calculate T0_6 from RPY, roll, pitch, yaw
R0_6 = R_z(roll)*R_y(pitch)*R_x(yaw)

# Calculate T3_6 based on the previous calculations
R3_6 = R0_6*(R0_3**-1)

# Calculate Euler angles from R3_6
theta5 = 
