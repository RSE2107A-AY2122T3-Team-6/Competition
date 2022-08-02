# Code description
***
## Lane detection
1. cmd.py: testing cmd_vel 
2. path.py: original code with angles (testing)
3. path_safe.py: original code with angles, finalised speed values
4. path_with_P.py: new code with 0.5 speed
5. path_with_PD.py: new code with 0.8 speed
6. path_with_PID.py: new code with 0.2 speed



# Final_testing for competition
## Lane detection safe values
1. linear speed = 0.1
2. angular speed = 0.05
3. Kp = 0.003
4. Kd = 0.01
5. interested_region = [
    (0,435),
    (0,420),
    (580,420),
    (580,440)
]
