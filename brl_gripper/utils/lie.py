import numpy as np

def skew(w):
    """ Returns the skew-symmetric matrix of a 3D vector. """
    return np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])

def unskew(W):
    """ Returns the 3D vector from a skew-symmetric matrix. """
    return np.array([W[2, 1], W[0, 2], W[1, 0]])

def expso3(w):
    """ Returns the matrix exponential of a skew-symmetric matrix. """
    theta = np.linalg.norm(w)
    
    w_skew = skew(w)
    if theta < 1e-5:
        return np.eye(3) + w_skew + (1/2) * (w_skew @ w_skew)
    else:
        return np.eye(3) + np.sin(theta) / theta * w_skew + (1 - np.cos(theta)) / (theta**2) * (w_skew @ w_skew)