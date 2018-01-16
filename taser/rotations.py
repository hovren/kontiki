import numpy as np

def quat_to_rotation_matrix(q):
    """Convert unit quaternion to rotation matrix
    
    Parameters
    -------------
    q : (4,) ndarray
            Unit quaternion, scalar as first element

    Returns
    ----------------
    R : (3,3) ndarray
            Rotation matrix
    
    """
    q = q.flatten()
    if not (q.size == 4 and np.isclose(np.linalg.norm(q), 1.0)):
        raise ValueError("Not a unit quaternion")
    qq = q ** 2
    R = np.array([[qq[0] + qq[1] - qq[2] - qq[3], 2*q[1]*q[2] -
                   2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
                  [2*q[1]*q[2] + 2*q[0]*q[3], qq[0] - qq[1] + qq[2] -
                   qq[3], 2*q[2]*q[3] - 2*q[0]*q[1]],
                  [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1],
                   qq[0] - qq[1] - qq[2] + qq[3]]])
    return R


def axis_angle_to_quat(r_or_n, theta=None):
    if theta is None:
        theta = np.linalg.norm(r_or_n)
        n = r_or_n / theta
    else:
        n = r_or_n
        
    assert np.isclose(np.linalg.norm(n), 1.0)
    w = np.cos(0.5 * theta)
    xyz = n * np.sin(0.5 * theta)
    return np.hstack((w, xyz))


def rotation_matrix_to_quat(R):
    threshold = np.sqrt(0.1)
    q1 = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
    if q1 >= threshold:
        q2 = (R[2,1] - R[1,2]) / (4*q1)
        q3 = (R[0,2] - R[2,0]) / (4*q1)
        q4 = (R[1,0] - R[0,1]) / (4*q1)
    else:
        q2 = np.sqrt(1 + R[0,0] - R[1,1] - R[2,2]) / 2
        if q2 >= threshold:
            q1 = (R[2,1] - R[1,2]) / (4*q2)
            q3 = (R[0,1] + R[1,0]) / (4*q2)
            q4 = (R[0,2] + R[2,0]) / (4*q2)
        else:
            q3 = np.sqrt(1 - R[0,0] + R[1,1] - R[2,2]) / 2
            if q3 >= threshold:
                q1 = (R[0,2] - R[2,0]) / (4*q3)
                q2 = (R[0,1] + R[1,0]) / (4*q3)
                q4 = (R[1,2] + R[2,1]) / (4*q3)
            else:
                q4 = np.sqrt(1 - R[0,0] - R[1,1] + R[2,2]) / 2
                q1 = (R[1,0] - R[0,1]) / (4*q4)
                q2 = (R[0,2] + R[2,0]) / (4*q4)
                q3 = (R[1,2] + R[2,1]) / (4*q4)
    return np.array([q1, q2, q3, q4])


def rotation_matrix_to_axis_angle(R):
    """Convert a 3D rotation matrix to a 3D axis angle representation
    
    Parameters
    ---------------
    R : (3,3) array
        Rotation matrix
        
    Returns
    ----------------
    v : (3,) array
        (Unit-) rotation angle
    theta : float
        Angle of rotations, in radians
    
    Note
    --------------
    This uses the algorithm as described in Multiple View Geometry, p. 584
    """
    assert R.shape == (3,3)
    from numpy.testing import assert_almost_equal
    assert_almost_equal(np.linalg.det(R), 1.0, err_msg="Not a rotation matrix: determinant was not 1")
    S, V = np.linalg.eig(R)
    k = np.argmin(np.abs(S - 1.))
    s = S[k]
    assert_almost_equal(s, 1.0, err_msg="Not a rotation matrix: No eigen value s=1")
    v = np.real(V[:, k]) # Result is generally complex
    
    vhat = np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]])
    sintheta = 0.5 * np.dot(v, vhat)
    costheta = 0.5 * (np.trace(R) - 1)
    theta = np.arctan2(sintheta, costheta)
    
    return (v, theta)


def quat_mult(q1, q2):
    a1, b1, c1, d1 = q1
    a2, b2, c2, d2 = q2

    a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2

    return np.array([a, b, c, d])


def quat_conj(q):
    qc = q.copy()
    qc[1:] *= -1
    return qc


def random_quaternion():
    q = np.random.uniform(-1, 1, size=4)
    q /= np.linalg.norm(q)
    return q


def identity_quaternion():
    return np.array([1., 0, 0, 0])


def procrustes(X, Y, remove_mean=False):
    """Orthogonal procrustes problem solver
    
    The procrustes problem  finds the best rotation R, and translation t
    where
        X = R*Y + t
    
    The number of points in X and Y must be at least 2.
    For the minimal case of two points, a third point is temporarily created
    and used for the estimation.
    
    Parameters
    -----------------
    X : (3, N) ndarray
            First set of points
    Y : (3, N) ndarray
            Second set of points
    remove_mean : bool
            If true, the mean is removed from X and Y before solving the
            procrustes problem. Can yield better results in some applications.
            
    Returns
    -----------------
    R : (3,3) ndarray
            Rotation component
    t : (3,) ndarray
            Translation component (None if remove_mean is False)
 """

    assert X.shape == Y.shape
    assert X.shape[0] > 1

    # Minimal case, create third point using cross product
    if X.shape[0] == 2:
        X3 = np.cross(X[:,0], X[:,1], axis=0)
        X = np.hstack((X, X3 / np.linalg.norm(X3)))
        Y3 = np.cross(Y[:,0], Y[:,1], axis=0)
        Y = np.hstack((Y, Y3 / np.linalg.norm(Y3)))


    D, N = X.shape[:2]
    if remove_mean:
        mx = np.mean(X, axis=1).reshape(D, 1)
        my = np.mean(Y, axis=1).reshape(D, 1)
        Xhat = X - mx
        Yhat = Y - my
    else:
        Xhat = X
        Yhat = Y


    (U, S, V) = np.linalg.svd((Xhat).dot(Yhat.T))

    Dtmp = np.eye(Xhat.shape[0])
    Dtmp[-1,-1] = np.linalg.det(U.dot(V))

    R_est = U.dot(Dtmp).dot(V)

    # Now X=R_est*(Y-my)+mx=R_est*Y+t_est
    if remove_mean:
        t_est= mx - R_est.dot(my)
    else:
        t_est = None
    return (R_est, t_est)

    
def rotation_between_vectors(a, b):
    "Find one R such that b = R @ a"
    a = a.ravel()
    b = b.ravel()
    anorm = np.linalg.norm(a)
    bnorm = np.linalg.norm(b)
    if not np.isclose(anorm, bnorm):
        raise ValueError("Input vectors had different lengths!")
    a = a / anorm
    b = b / bnorm
    v = np.cross(a.ravel(), b.ravel())
    s = np.linalg.norm(v)
    if np.isclose(s, 0.0):
        return np.eye(3)
    c = a @ b
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + vx + (vx @ vx) * (1-c) / (s**2)
    return R    
