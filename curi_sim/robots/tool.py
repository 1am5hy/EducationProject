import numpy as np

def make_pose(translation, rotation):
    """
    Makes a homogeneous pose matrix from a translation vector and a rotation matrix.
    Args:
        translation (np.array): (x,y,z) translation value
        rotation (np.array): a 3x3 matrix representing rotation
    Returns:
        pose (np.array): a 4x4 homogeneous matrix
    """
    pose = np.zeros((4, 4))
    pose[:3, :3] = rotation
    pose[:3, 3] = translation
    pose[3, 3] = 1.0
    return pose

def pose_inv(pose):
    """
    Computes the inverse of a homogeneous matrix corresponding to the pose of some
    frame B in frame A. The inverse is the pose of frame A in frame B.
    Args:
        pose (np.array): 4x4 matrix for the pose to inverse
    Returns:
        np.array: 4x4 matrix for the inverse pose
    """

    # Note, the inverse of a pose matrix is the following
    # [R t; 0 1]^-1 = [R.T -R.T*t; 0 1]

    # Intuitively, this makes sense.
    # The original pose matrix translates by t, then rotates by R.
    # We just invert the rotation by applying R-1 = R.T, and also translate back.
    # Since we apply translation first before rotation, we need to translate by
    # -t in the original frame, which is -R-1*t in the new frame, and then rotate back by
    # R-1 to align the axis again.

    pose_inv = np.zeros((4, 4))
    pose_inv[:3, :3] = pose[:3, :3].T
    pose_inv[:3, 3] = -pose_inv[:3, :3].dot(pose[:3, 3])
    pose_inv[3, 3] = 1.0
    return pose_inv

def pose_in_A_to_pose_in_B(pose_A, pose_A_in_B):
    """
    Converts a homogenous matrix corresponding to a point C in frame A
    to a homogenous matrix corresponding to the same point C in frame B.
    Args:
        pose_A (np.array): 4x4 matrix corresponding to the pose of C in frame A
        pose_A_in_B (np.array): 4x4 matrix corresponding to the pose of A in frame B
    Returns:
        np.array: 4x4 matrix corresponding to the pose of C in frame B
    """

    # pose of A in B takes a point in A and transforms it to a point in C.

    # pose of C in B = pose of A in B * pose of C in A
    # take a point in C, transform it to A, then to B
    # T_B^C = T_A^C * T_B^A
    return pose_A_in_B.dot(pose_A)

def inertia_in_A_to_in_B(T,I,m):
    R = T[:3,:3]
    p = T[3,:3]
    I_B = I + m*(p.T*p*np.eye(3)-p*p.T)
    I_B = R*I_B*R.T
    return I_B


# T = np.array([[1,0,0,0.003875],[0,1,0,0.002081],[0,0,1,0],[0,0,0,1]])
# m=4.970684
# I = np.array([[0.70337,-0.000139,0.006772],[-0.000139,0.70661,0.019169],[0.006772,0.019169,0.009117]])
# I_B=inertia_in_A_to_in_B(T,I,m)


# T = np.array([[1,0,0,-3.141e-03],[0,1,0,-2.872e-02],[0,0,1,3.495e-03],[0,0,0,1]])
# m=0.646926
# xx=7.9620e-03
# xy=-3.9250e-03
# xz=1.0254e-02
# yy=2.8110e-02
# yz=7.0400e-04
# zz=2.5995e-02
# I = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
# I_B=inertia_in_A_to_in_B(T,I,m)

# T = np.array([[1,0,0,2.7518e-02],[0,1,0,3.9252e-02],[0,0,1,-6.6502e-02],[0,0,0,1]])
# m=3.228604
# xx=3.7242e-02
# xy=-4.7610e-03
# xz=-1.1396e-02
# yy=3.6155e-02
# yz=-1.2805e-02
# zz=1.0830e-02
# I = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
# I_B=inertia_in_A_to_in_B(T,I,m)

# T = np.array([[1,0,0,-5.317e-02],[0,1,0,1.04419e-01],[0,0,1,2.7454e-02],[0,0,0,1]])
# m=3.587895
# xx=2.5853e-02
# xy=7.7960e-03
# xz=-1.3320e-03
# yy=1.9552e-02
# yz=8.6410e-03
# zz=2.8323e-02
# I = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
# I_B=inertia_in_A_to_in_B(T,I,m)

# T = np.array([[1,0,0,-1.1953e-02],[0,1,0,4.1065e-02],[0,0,1,-3.8437e-02],[0,0,0,1]])
# m=1.225946
# xx=3.5549e-02
# xy=-2.1170e-03
# xz=-4.0370e-03
# yy=2.9474e-02
# yz=2.2900e-04
# zz=8.6270e-03
# I = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
# I_B=inertia_in_A_to_in_B(T,I,m)

# T = np.array([[1,0,0,6.0149e-02],[0,1,0,-1.4117e-02],[0,0,1,-1.0517e-02],[0,0,0,1]])
# m=1.666555
# xx=1.9640e-03
# xy=1.0900e-04
# xz=-1.1580e-03
# yy=4.3540e-03
# yz=3.4100e-04
# zz=5.4330e-03
# I = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
# I_B=inertia_in_A_to_in_B(T,I,m)
#
# T = np.array([[1,0,0,1.0517e-02],[0,1,0,-4.252e-03],[0,0,1,6.1597e-02],[0,0,0,1]])
# m=7.35522e-01
# xx=1.2516e-02
# xy=-4.2800e-04
# xz=-1.1960e-03
# yy=1.0027e-02
# yz=-7.4100e-04
# zz=4.8150e-03
# I = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
# I_B=inertia_in_A_to_in_B(T,I,m)
# print(I_B)








