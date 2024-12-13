import numpy as np
import pytest
from locomotion_mpc.trajectory import Trajectory

def test_trajectory():
    traj = Trajectory()
    traj.q_trajectory = np.array(range(11))
    traj.v_trajectory = np.array(range(11))
    traj.tau_trajectory = np.array(range(11))
    traj.F_trajectory = np.array([range(11)])
    traj.time_traj = 0.1*np.array(range(11))

    traj.print()

    t = 0.75
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    print(q)

    assert q == 7.5
    assert v == 7.5
    assert tau == 7.5
    assert F == 7.5

    t = 0
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    assert q == 0
    assert v == 0
    assert tau == 0
    assert F == 0

    t = 1
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    assert q == 10
    assert v == 10
    assert tau == 10
    assert F == 10

    t = 0.95
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    assert q == 9.5
    assert v == 9.5
    assert tau == 9.5
    assert F == 9.5

test_trajectory()
