# =============================================================================
#                        NUVEM DE PONTOS COM KINECT V2
#                         BY: JEFFERSON S. ALMEIDA
#                             DATA: 24/02/2020
#                E-MAIL: jeffersonsilva@lapisco.ifce.edu.br
#          Adaptado de: https://github.com/JayHuangYC/VirtualMirror
# =============================================================================

import freenect
import numpy as np
import time
import os, sys, stat

def depth2xyzuv(depth, u=None, v=None):
    """
  Return a point cloud, an Nx3 array, made by projecting the kinect depth map
    through intrinsic / extrinsic calibration matrices
  Parameters:https://github.com/JayHuangYC/VirtualMirror
    depth - comes directly from the kinect
  Returns:
    xyz - 3D world coordinates in meters (Nx3)
  """
    # Build a 3xN matrix of the d,u,v data
    C = np.vstack((u.flatten(), v.flatten(), depth.flatten(), 0 * u.flatten() + 1))

    # Project the duv matrix into xyz using xyz_matrix()
    X, Y, Z, W = np.dot(xyz_matrix(), C)
    X, Y, Z = X / W, Y / W, Z / W

    # Project the duv matrix into U,V rgb coordinates using rgb_matrix() and xyz_matrix()
    # U,V,_,W = np.dot(uv_matrix(), np.dot(xyz_matrix(),C))
    # U,V = U/W, V/W
    super_xyz = np.vstack((X, Y, Z, 0 * Z + 1))
    # print super_xyz.shape
    U, V, W = np.dot(uv_matrix(), super_xyz)
    U, V = U / W, V / W

    xyz = np.vstack((X, Y, Z)).transpose()
    # Z<0 -> Z>0
    xyz = xyz[Z > 0, :]

    uv = np.vstack((U, V)).transpose()
    # Z<0 -> Z>0
    uv = uv[Z > 0, :]

    # Return both the XYZ coordinates and the UV coordinates
    # print(xyz)
    # print(uv)
    return xyz, uv


def uv_matrix():
    """
  Returns a matrix you can use to project XYZ coordinates (in meters) into
      U,V coordinates in the kinect RGB image
  """
    # +(0,2),-(1,2),-(2,2) => flip the image to align with scene point
    rot = np.array([[0.99999892027314474, -0.00051357037360716667, +0.0013768434973232286],
                    [0.00051845618194043942, 0.99999356237038495, -0.0035505522069703959],
                    [0.001375011175291316, -0.0035512622063665761, -0.99999274891421563]])

    trans = np.array([[1.9985e-02, -7.44237e-04, -1.0916736e-02]])
    # -trans. -> +trans.
    m = np.hstack((rot, trans.transpose()))
    '''m = np.vstack((m, np.array([[0,0,0,1]])))
  KK = np.array([[520.97092069697146, 0.0, 318.40565581396697, 0],
                 [0.0, 517.85544366622719, 263.46756370601804, 0],
                 [0, 0, 0, 1],
                 [0, 0, 1, 0]])
  '''
    KK = np.array([[520.97092069697146, 0.0, 318.40565581396697],
                   [0.0, 517.85544366622719, 263.46756370601804],
                   [0, 0, 1]])
    m = np.dot(KK, (m))
    # print m.shape
    # print m
    return m


def xyz_matrix():
    fx = 588.5168602060173
    fy = 584.73028132692866
    a = -0.0030711
    b = 3.3309495
    cx = 320.22664144213843
    cy = 241.98395817513071
    # +(2,3),
    mat = np.array([[-1 / fx, 0, 0, cx / fx],
                    [0, -1 / fy, 0, cy / fy],
                    [0, 0, 0, 1],
                    [0, 0, a, b]])
    # print mat
    return mat

# timer
for t in range(3):
    time.sleep(1)  # time in seconds
    print(t)

(depth, _) = freenect.sync_get_depth()
(rgb, _) = freenect.sync_get_video()

X, Y = np.meshgrid(range(640), range(480))
d = 2  # downsampling if need
projpts = depth2xyzuv(depth[::d, ::d], X[::d, ::d], Y[::d, ::d])
xyz, uv = projpts

row, col = xyz.shape

points = []
for i in range(row):
    X = xyz[i, 0]
    Y = xyz[i, 1]
    Z = xyz[i, 2]
    points.append("%f %f %f\n" % (X, Y, Z))

with open("/home/jefferson/cloud.txt", "w") as txt:
        txt.write("{}".format("".join(points)))

os.system("sudo chmod 777 /home/jefferson/cloud.txt")
os.system("sudo chown jefferson /home/jefferson/cloud.txt")

print("ok")
freenect.sync_stop()
