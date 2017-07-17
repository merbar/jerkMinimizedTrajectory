import numpy as np
from scipy.linalg import solve

# TODO - complete this function
def JMT(start, end, T):
    """
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    """
    i_pos = start[0]
    i_vel = start[1]
    i_acc = start[2]
    e_pos = end[0]
    e_vel = end[1]
    e_acc = end[2]
    T_2 = T**2
    T_3 = T**3
    T_4 = T**4
    T_5 = T**5

    # solve for a3, a4 and a5
    A = np.array([[T_3, T_4, T_5], [3*T_2, 4*T_3, 5*T_4], [6*T, 12*T_2, 20*T_3]])
    b0 = e_pos - (i_pos + i_vel*T + 0.5*i_acc*T_2)
    b1 = e_vel - (i_vel*T + i_acc*T)
    b2 = e_acc - i_acc
    b = np.array([b0, b1, b2])
    a3, a4, a5 = np.linalg.solve(A,b)
    #a3, a4, a5 = solve(A, b)
    #x = solve(A, b)

    a0 = i_pos
    a1 = i_vel
    a2 = 0.5*i_acc

    return [a0, a1, a2, a3, a4, a5]