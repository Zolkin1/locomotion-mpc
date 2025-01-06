import numpy as np
import pytest
from locomotion_mpc.trajectory import Trajectory

def test_trajectory():
    nq = 2
    nv = 2
    ntau = 2
    nf = 3
    nt = 11
    traj = Trajectory(nq, nv, ntau, nf, nt)

    for i in range(nt):
        traj.q_trajectory[i,:] = i*np.ones((nq,))
        traj.v_trajectory[i,:] = i*np.ones((nq,))
        traj.tau_trajectory[i,:] = i*np.ones((ntau,))
        traj.F_trajectory[i, :] = i*np.ones((nf,))
    traj.time_traj = 0.1*np.array(range(nt))

    traj.print()

    t = 0.75
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    print(q)

    for i in range(nq):
        assert q[i] == 7.5

    for i in range(nv):
        assert v[i] == 7.5

    for i in range(ntau):
        assert tau[i] == 7.5

    for i in range(nf):
        assert F[i] == 7.5

    t = 0
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    for i in range(nq):
        assert q[i] == 0

    for i in range(nv):
        assert v[i] == 0

    for i in range(ntau):
        assert tau[i] == 0

    for i in range(nf):
        assert F[i] == 0

    t = 1
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    for i in range(nq):
        assert q[i] == 10

    for i in range(nv):
        assert v[i] == 10

    for i in range(ntau):
        assert tau[i] == 10

    for i in range(nf):
        assert F[i] == 10

    t = 0.92
    q = traj.get_config(t)
    v = traj.get_vel(t)
    tau = traj.get_torque(t)
    F = traj.get_force(t, 0)

    print(q)
    for i in range(nq):
        assert abs(q[i] - 9.2) < 1e-5

    for i in range(nv):
        assert abs(v[i] - 9.2) < 1e-5

    for i in range(ntau):
        assert abs(tau[i] - 9.2) < 1e-5

    for i in range(nf):
        assert abs(F[i] - 9.2) < 1e-5

# def test_vector_trajectory():
    # traj = Trajectory()
    # traj.q_trajectory = np.array([range(11)])
    # traj.v_trajectory = np.array(range(11))
    # traj.tau_trajectory = np.array(range(11))
    # traj.F_trajectory = np.array([range(11)])
    # traj.time_traj = 0.1*np.array(range(11))

    # traj.print()


test_trajectory()
# test_vector_trajesctory()