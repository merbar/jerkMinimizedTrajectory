import numpy as np
from numpy.linalg import inv
from numpy.linalg import matrix_rank

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
    use_optimized = True

    i_pos = float(start[0])
    i_vel = float(start[1])
    i_acc = float(start[2])
    e_pos = float(end[0])
    e_vel = float(end[1])
    e_acc = float(end[2])
    T_2 = float(T**2)
    T_3 = float(T**3)
    T_4 = float(T**4)
    T_5 = float(T**5)

    if use_optimized:
        # optimized method presented in udacity course
        # solve only for a3, a4 and a5
        A = np.array([[T_3, T_4, T_5], [3*T_2, 4*T_3, 5*T_4], [6*T, 12*T_2, 20*T_3]])
        b0 = e_pos - (i_pos + i_vel*T + 0.5*i_acc*T_2)
        b1 = e_vel - (i_vel + i_acc*T)
        b2 = e_acc - i_acc
        b = np.array([b0, b1, b2])
        a3, a4, a5 = inv(A).dot(b)

        a0 = i_pos
        a1 = i_vel
        a2 = 0.5*i_acc
    else:
        # full solve
        A = np.array([[0,0,0,0,0,1], [T_5,T_4,T_3,T_2,T,1], [0,0,0,0,1,0], [5*T_4,4*T_3,3*T_2,2*T,1,0], [0,0,0,2,0,0], [20*T_3,12*T_2,6*T,2,0,0]])
        b = np.array([i_pos, e_pos, i_vel, e_vel, i_acc, e_acc])
        a5, a4, a3, a2, a1, a0 = inv(A).dot(b)

    return [a0, a1, a2, a3, a4, a5]