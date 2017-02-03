"""
# SE3D distribution package
#
# Copyright Emanuele Ruffaldi, Scuola Superiore Sant'Anna 2016
#
# Missing functions: sample unscented sqrt

"""
import math
import numpy as np


def _quaternion_from_matrix(matrix, isprecise=False):
    """Summary
    
    Args:
        matrix (TYPE): Description
        isprecise (bool, optional): Description
    
    Returns:
        TYPE: Description
    """
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 1, 2, 3
            if M[1, 1] > M[0, 0]:
                i, j, k = 2, 3, 1
            if M[2, 2] > M[i, i]:
                i, j, k = 3, 1, 2
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
        # NEED MORMALIZE
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                      [m01+m10,     m11-m00-m22, 0.0,         0.0],
                      [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                      [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    n = np.linalg.norm(q)
    if n > 1.0:
        q = q/n
    # taken from ransformations.py so w first
    return (q[0], q[1:])


def _quaternion_matrix(quaternion_w_xyz):
    """Summary
    
    Args:
        quaternion_w_xyz (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    _EPS = 1e-9
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> np.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> np.allclose(M, np.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> np.allclose(M, np.diag([1, -1, -1, 1]))
    True

    From transformations so it is (w,(xyz))
    """
    quaternion = (quaternion_w_xyz[0], quaternion_w_xyz[1][
                  0], quaternion_w_xyz[1][1], quaternion_w_xyz[1][2])
    quaternion = quaternion / np.linalg.norm(quaternion)
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [0.0,                 0.0,                 0.0, 1.0]])

# sin expansion:
# syms x real
# simplify(taylor(sin(x)/x,x,0,'Order',6))


def asinc(x):
    """Summary
    
    Args:
        x (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    if math.fabs(x) < 1e-10:
        x2 = x*x
        return x2*x2/120 - x2/6 + 1
    else:
        return math.sin(x)/x


def rodriguez2rot(omega):
    """Summary
    
    Args:
        omega (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    theta = np.linalg.norm(omega)
    if math.fabs(theta) < 1e-10:
        A = 1.0
        B = 0.5
        C = 1/6.0
        S = np.zeros((3, 3))
        R = np.identity(3)
    else:
        A = asinc(theta)
        B = (1-math.cos(theta))/(theta*theta)
        C = (1-A)/(theta*theta)
        S = skew(omega)
        R = np.identity(3) + A*S + B*np.dot(S, S)
    return R, S, B, C


def rot2rodriguez(R):
    """Summary
    
    Args:
        R (TYPE): Description
    
    Returns:
        TYPE: Description
    """

    if abs((R.trace()-1)/2)>=1:
        theta=0
    else:
        theta = math.acos((R.trace()-1)/2)
    A = asinc(theta)
    small = math.fabs(theta) < 1e-10
    SO = (1/(2*A))*(R-R.transpose())  # % =skew(omega)
    if small:
        B = 0.5
        iV = np.identity(3)
    else:
        B = (1-math.cos(theta))/(theta*theta)
        # % use directly skew of omega
        iV = np.identity(3) - 0.5*SO + 1/(theta**2)*(1 - A/2/B)*np.dot(SO, SO)
    omega = [SO[2, 1], SO[0, 2], SO[1, 0]]
    return omega, small, iV


def skew(v):
    """Summary
    
    Args:
        v (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

# Sets the distribution specifying mean and variance
#
# Internally the group is stored  as flattened


def se3d_set(g, S):
    """Summary
    
    Args:
        g (TYPE): Description
        S (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    #assert(length(g) == 4,'group has to be in 4x4 form')
    #assert(length(S) == 6,'covariance has to be in 6x6 form')
    return dict(mean=g, Sigma=S)
#
# Logarithm of the group: the algebra
#
# Emanuele Ruffaldi 2016


def se3_log(x):
    """Computes the algebra via logarithm
    
    Args:
        x (matrix 4x4): input
    
    Returns:
        vector 6x1: output logarithm
    """
    R = x[0:3, 0:3]
    t = x[0:3, 3]
    omega, small, iV = rot2rodriguez(R)
    u = np.dot(iV, t)
    y = np.zeros(6)
    y[0:3] = omega
    y[3:6] = u
    return y

def se3_exp(x):
    """exponentiates an algebra to obtain group
    
    Args:
        x (vector 6x1): input
    
    Returns:
        matrix 4x4: output
    """
    omega = x[0:3]
    u = x[3:6]
    R, S, B, C = rodriguez2rot(omega)
    V = np.identity(3) + B*S + C*np.dot(S, S)
    y = np.identity(4)
    y[0:3, 0:3] = R
    y[0:3, 3] = np.dot(V, u)
    return y


def se3_mul(a, b):
    """produce of two groups
    
    Args:
        a (matrix 4x4): first
        b (matrix 4x4): second
    
    Returns:
        matrix 4x4: output
    """
    return np.dot(a, b)

# Inverse of the group


def se3_inv(x):
    """inversion
    
    Args:
        x (matrix 4x4): input
    
    Returns:
        matrix 4x4: output
    """
    R = x[0:3, 0:3]
    t = x[0:3, 3]
    y = np.identity(4)
    y[0:3, 0:3] = R.transpose()
    y[0:3, 3] = - np.dot(y[0:3, 0:3], t)
    return y


def se3_fromRotT(R, t):
    """Summary
    
    Args:
        R (TYPE): Description
        t (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    y = np.identity(4)
    y[0:3, 0:3] = R
    y[0:3, 3] = t
    return y


def se3_getrpyurdf(X):
    """Summary
    
    Args:
        X (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    w, xyz = se3_getquat(X)
    x, y, z = xyz
    original = (math.atan2(2 * (w*x + y*z), 1 - 2 * (x**2 + y**2)),
                math.asin(2 * (w*y - z*x)),
                math.atan2(2 * (w*z + x*y), 1 - 2 * (y**2 + z**2)))
    return original


def se3_fromQuatT(q, t):
    """Summary
    
    Args:
        q (TYPE): Description
        t (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    y = _quaternion_matrix(q)
    y[0:3, 3] = t
    return y


def se3_fromRvecT(R, t):
    """Summary
    
    Args:
        R (TYPE): Description
        t (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    if type(R) is list:
        R = np.array(R)
    if type(t) is list:
        t = np.array(t)
    y = np.identity(4)
    y[0:3, 0:3] = rodriguez2rot(R)[0]
    y[0:3, 3] = t.transpose()
    return y


def se3_dist(a, b):
    """Summary
    
    Args:
        a (TYPE): Description
        b (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    return np.linalg.norm(se3_log(se3_mul(se3_inv(a), b)))


def se3_gett(X):
    """Summary
    
    Args:
        X (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    return X[0:3, 3]


def se3_getrvec(X):
    """Summary
    
    Args:
        X (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    return rot2rodriguez(X[0:3, 0:3])[0]


def se3_getquat(X):
    """Summary
    
    Args:
        X (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    return _quaternion_from_matrix(X, True)


def se3_getR(X):
    """Extracts rotation
    
    Args:
        X (TYPE): Description
    
    Returns:
        TYPE: Description
    """
    return X[0:3, 0:3]


def se3_adj(x):
    """Adjoint computation
    
    Args:
        x (group): first
    
    Returns:
        adjoint matrix: the adjoint of the group
    """
    R = x[0:3, 0:3]
    t = x[0:3, 3]
    y = np.zeros((6, 6,))
    y[0:3, 0:3] = R
    y[0:3, 3:6] = np.dot(skew(t), R)
    y[3:6, 3:6] = R
    return y


def se3d_inv(a):
    """Inversion of first
    
    Args:
        a (distribution): first
    
    Returns:
        distribution: the inverse transformation
    """
    ga = a["mean"]
    ca = a["Sigma"]
    A = se3_adj(ga)
    return se3d_set(se3_inv(ga), np.dot(A, np.dot(ca, np.transpose(A))))


def se3d_mul(a, b):
    """Composition of transformations
    
    Args:
        a (distribution): first
        b (distribution): second
    
    Returns:
        distribution: result product
    """
    ga = a["mean"]
    gb = b["mean"]
    ca = a["Sigma"]
    cb = b["Sigma"]
    A = se3_adj(ga)
    return se3d_set(se3_mul(ga, gb), ca + np.dot(A, np.dot(cb, np.transpose(A))))


def se3d_fuse(a, b):
    """Fuses two poses with equal weights
    
    Args:
        a (distribution): first
        b (distribution): second
    
    Returns:
        distribution: the fused result
    """
    ga = a["mean"]
    gb = b["mean"]
    ca = a["Sigma"]
    cb = b["Sigma"]
    #cy = c0 - c0/(c0 + c1)*c0;
    cy = c0 - numoy.dot(np.dot(c0, np.linalg.inv(c0+c1)), c0)
    v = se3_log(np.dot(gb, se3_inv(g0)))
    gy = np.dot(se3_exp(np.dot(np.dot(gy, np.linalg.inv(g1)), v)), g0)
    return se3d_set(gy, cy)

def se3d_est(x, steps, gk=None):
    """Estimates a transformation (SE3 group) distribution from an array of transformations (SE3 group)
    Args:
        x (group array): array of transformations
        steps (int): number of steps for the iterative algorithm
        gk (group, optional): initial point, otherwise the first element

    Returns: 
        distribution: a SE3 distribution
    """
    N = len(x)

    if gk is None:
        gk = x[0]  # % starting => random selection

    for k in range(0, steps):
        igk = se3_inv(gk)
        ma = np.zeros(6)
        for i in range(0, N):
            v_ik = se3_log(se3_mul(x[i], igk))
            ma = ma + v_ik.transpose()
        ma = ma / N
        gk = se3_mul(se3_exp(ma), gk)

    Sk = np.zeros((6, 6))
    igk = se3_inv(gk)
    for i in range(0, N):
        v_ik = se3_log(se3_mul(x[i], igk))
        Sk = Sk + np.dot(v_ik, v_ik.transpose())
    if N > 2:
        Sk = Sk / (N-1)  # unbiased estimator
    return se3d_set(gk, Sk)

if __name__ == '__main__':
    def dump(x, n):
        """Summary
        
        Args:
            x (TYPE): Description
            n (TYPE): Description
        
        Returns:
            TYPE: Description
        """

    a = se3_fromRvecT(0.5*np.array([1, 0, 0]), np.array([0.2, 0.3, 0.0]))
    b = se3_fromRvecT(np.array([0, 0, 0]), np.array([0.8, 0.3, 0.2]))
    dump(a, "a")
    dump(b, "b")
    ea = se3_log(a)
    print "log a:", ea
    aea = se3_exp(ea)
    dump(aea, "exp log a")
    c = se3_mul(a, b)
    dump(c, "a * b")
    print "log c:", se3_log(c)
    ia = se3_inv(a)
    dump(ia, "inv a")
    E = se3d_est([a, b], 5)
    dump(E["mean"], "Emean")

    iE = se3d_inv(E)
    dump(iE["mean"], "iE")