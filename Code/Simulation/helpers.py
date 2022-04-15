import numpy as np
import matplotlib.pyplot as plt
def TransformationToPN(T):
    point = [row[3] for row in T][0:3]
    normal1 = np.transpose(np.array([0, 1, 0, 1]))
    normal = np.matmul(np.array(T), normal1)
    normal = normal[0:3]
    normal = normal/np.linalg.norm(normal)
    return point, normal

# def PNToTransformation(point, normal):
#     A = normal[0]
#     B = normal[1]
#     C = normal[2]
#     D = -np.dot(point, normal)
#     nx = normal[0]
#     ny = normal[1]
#     nz = normal[2]

#     T = [[ny/np.sqrt(nx**2+ny**2), -ny/np.sqrt(nx**2+ny**2), 0, point[0]], 
#     [nx*nz/np.sqrt(nx**2+ny**2), ny*nz/np.sqrt(nx**2+ny**2), -np.sqrt(nx**2+ny**2), point[1]],
#     [nx, ny, nz, point[2]],
#     [0, 0, 0, 1]]
#     return T

def plotLine(point, point2, ax, n=100, color = 'green'):
    x1 = point[0]
    x2 = point2[0]
    y1 = point[1]
    y2 = point2[1]
    z1 = point[2]
    z2 = point2[2]
    ax.plot([x1, x2], [y1, y2], [z1, z2], color = color)
   
def plotPlaneNew(T, ax, n = 100):
    dim1 = np.linspace(-10, 10, n)
    dim2 = np.linspace(-10, 10, n)
    plot1 = np.zeros([n, n])
    plot2 = np.zeros([n, n])
    plot3 = np.zeros([n, n])
    mesh1, mesh2 = np.meshgrid(dim1, dim2)
    for i in range(n):
        for j in range(n):
            pb =  np.transpose(np.array([mesh1[i][j], 0, mesh2[i][j], 1]))
            ps =   np.matmul(np.array(T), np.array(pb))
            plot1[i][j] = ps[0]
            plot2[i][j] = ps[1]
            plot3[i][j] = ps[2]
    ax.plot_surface(plot1, plot2, plot3)
    #buggy stuff because if we have a tranformation matrix which doesn't rotate along a certain axis we'll have issues setting axis limits
    if np.allclose(plot2, np.zeros([n, n])):
        ax.set_box_aspect(aspect = (1,1,1))
    else:
        ax.set_box_aspect((np.ptp(plot1), np.ptp(plot2), np.ptp(plot3)))

def plotTransformationM(T):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    point, normal = TransformationToPN(T)
    point2 = np.add(point,normal)
    plotLine(point, point2, ax)
    plotPlaneNew(T, ax)
    plt.xlabel("x")
    plt.ylabel("y")
    ax.set_zlabel('z')
    return ax
    
def closestPoint(point1, planepoint, normal):
    v = np.subtract(point1, planepoint)
    dist = np.dot(v, normal)
    print("Distance is: %.3f" % dist)
    return np.subtract(point1, dist*normal)

def eeToPlane(ee, T, ax):
    point, normal = TransformationToPN(T)
    closest = closestPoint(ee, point, normal)
    plotLine(ee, closest, ax, color = 'blue')
    ax.plot(ee[0], ee[1], ee[2], color = 'red', marker = 'o', markersize = 12)

# T1 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
# plotTransformationM(T1)
# T2 = [[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
# plotTransformationM(T2)
T3 = [[1, 0, 0, 0], [0, np.cos(15), np.sin(15), 0], [0, -np.sin(15), np.cos(15), 0], [0, 0, 0, 1]]
ax = plotTransformationM(T3)
eeToPlane([3, 3, 3], T3, ax)
eeToPlane([-3, -3, -3], T3, ax)
# T4 = [[np.cos(15), 0, np.sin(15), 0], [0, 1, 0 , 0], [-np.sin(15), 0 , np.cos(15), 0], [0, 0, 0, 1]]
# plotTransformationM(T4)

plt.show()


 

