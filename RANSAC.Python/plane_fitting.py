import numpy as np
from ransac import *

def augment(xyz):
    axyz = np.ones((len(xyz), 4))
    axyz[:, :3] = xyz
    return axyz

def estimate(xyz):
    axyz = augment(xyz[:3])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold

if __name__ == '__main__':
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits import mplot3d

    def plot_plane(a, b, c, d):
        xx, yy = np.mgrid[:10, :10]
        return xx, yy, (-d - a * xx - b * yy) / c

    #n = 100
    #max_iterations = 100
    #goal_inliers = n * 0.3  
    #xyzs = np.random.random((n, 3)) * 10
    #xyzs[:50, 2:] = xyzs[:50, :1]       
    

    xyzList = []
    xyz = np.array([[20, 0,	0], 
                     [10, -10,	0], 
                     [10, 10,	0]])
    xyzList.append(xyz)
    
    xyz = np.array([[20, 0,	3], 
                     [10, -10,	2], 
                     [10, 10,	2]])
    xyzList.append(xyz)
    
    xyz = np.array([[20, -10,	0.2], 
                     [20, 0,	0.2], 
                     [20, 10,	0.2],
                     [15, -10,	0.15], 
                     [15, 0,	0.15], 
                     [15, 10,	0.15],
                     [10, -10,	0.1], 
                     [10, 10,	0.1], 
                     [20, 18,	1.7],
                     [15, -15,	1.2]])
    xyzList.append(xyz)
    
   
    for xyz in xyzList:
        fig = plt.figure()
        ax = mplot3d.Axes3D(fig)
        n = xyz.shape[0]
        goal_inliers = 0.5 * n
        max_iterations = 100
        ax.scatter3D(xyz.T[0], xyz.T[1], xyz.T[2]) 
    
        # Run RANSAC
        m, b = run_ransac(xyz, estimate, lambda x, y: is_inlier(x, y, 0.01), n, goal_inliers, max_iterations)
        a, b, c, d = m
        xx, yy, zz = plot_plane(a, b, c, d)
        ax.plot_surface(xx, yy, zz, color=(0, 1, 0, 0.5))    
        plt.show()
