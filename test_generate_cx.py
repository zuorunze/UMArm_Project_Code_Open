import numpy as np
import matplotlib.pyplot as plt


def fibonacci_sphere(n):
    points = np.zeros((n, 3))
    phi = np.pi * (3. - np.sqrt(5))  # golden angle

    for i in range(n):
        y = 1 - (i / float(n - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)     # radius at y

        theta = phi * i  # golden angle increment

        x = np.cos(theta) * radius
        z = np.sin(theta) * radius

        points[i] = [x, y, z]

    return points


def generate_Cx(x_vec, y_vec, z_vec, x_val, y_val, z_val):
    
    P = np.eye(3)
    P[:,0] = x_vec
    P[:,1] = y_vec
    P[:,2] = z_vec
    LAMB = np.eye(3)
    
    LAMB[0,0] = x_val
    LAMB[1,1] = y_val
    LAMB[2,2] = z_val

    return P @ LAMB @ np.linalg.inv(P)


if __name__ == "__main__":
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.view_init(elev=10, azim=180)  # Elevation: 30 degrees, Azimuth: 45 degrees



    Cx = np.array(
    [[ 0.02099985, -0.00040353, -0.01932757],
     [-0.00040353,  0.06023204,  0.00775993],
     [-0.01932757,  0.00775993,  0.02326683],]) * 50
    
    # x_axis = np.array([1,0,0],dtype=float)
    # y_axis = np.array([0,1,0],dtype=float)
    # z_axis = np.array([0,0,1],dtype=float)

    # x_val = 1
    # y_val = 2
    # z_val = 3


    # Cx = generate_Cx(x_axis, y_axis, z_axis, x_val, y_val, z_val)

    # print(Cx)


    eigenvalues, eigenvectors = np.linalg.eig(Cx)
    U, s, Vh = np.linalg.svd(Cx)

    print("Cx: \n", Cx)

    print("eigenvalue:\n", eigenvalues, '\n', "eigenvectors: \n", eigenvectors)
    print("sigma:\n", s)
    print("leftvec:\n", U)
    print("rightvec:\n", Vh)

    num_points = 200
    fvec = fibonacci_sphere(num_points)

    # cvd: compliance virtual displacement
    cvd_vec = np.zeros((num_points, 3))

    for i in range(num_points):
        # NOTE: fvec should be given in robot frame!

        cvd_vec[i] = Cx @ fvec[i]

    color_list = [(0,0,0)]*len(fvec)

    for i in range(len(fvec)):
        color_list[i] = (np.random.rand(), np.random.rand(), np.random.rand(), 0.2)
    
    
    

    frame_origins = np.zeros((len(fvec), 3))

    ax.quiver(frame_origins[:,0], frame_origins[:,1], frame_origins[:,2],
              fvec[:,0], fvec[:,1], fvec[:,2],
              color=color_list, arrow_length_ratio=0.1)
    
    ax.quiver(frame_origins[:,0], frame_origins[:,1], frame_origins[:,2],
              cvd_vec[:,0], cvd_vec[:,1], cvd_vec[:,2],
              color=color_list, arrow_length_ratio=0.1)
    
    longaxis = Cx @ Vh[0,:]
    ax.quiver(0,0,0,
              *longaxis,
              color='darkgreen',length=1, linewidth=3, arrow_length_ratio=0.1)
    axis2 = Cx @ Vh[1,:]
    ax.quiver(0,0,0,
              *axis2,
              color='green',length=1, linewidth=3, arrow_length_ratio=0.1)
    axis3 = Cx @ Vh[2,:]
    ax.quiver(0,0,0,
              *axis3,
              color='lightgreen',length=1, linewidth=3, arrow_length_ratio=0.1)

    ax.scatter(fvec[:,0], fvec[:,1], fvec[:,2], c='purple', s=5)
    ax.scatter(cvd_vec[:,0], cvd_vec[:,1], cvd_vec[:,2], c='green', s=5)
    
    
    eig1 = eigenvectors[:,0]
    ax.quiver(0,0,0,
              *eig1,
              color='red',length=5, linewidth=2, arrow_length_ratio=0.1)
    eig2 = eigenvectors[:,1]
    ax.quiver(0,0,0,
              *eig2,
              color='pink',length=5, linewidth=2, arrow_length_ratio=0.1)
    eig3 = eigenvectors[:,2]
    ax.quiver(0,0,0,
              *eig3,
              color='yellow',length=5, linewidth=2, arrow_length_ratio=0.1)
    
    
    eigenval_diagonal = np.array([
        [eigenvalues[0],0,0],
        [0,eigenvalues[1], 0],
        [0,0,eigenvalues[2]]
    ])

    recon_cx = eigenvectors @ eigenval_diagonal @ np.linalg.inv(eigenvectors)

    print("reconstructed cx: \n", recon_cx)
    
    
    


    # Define the origin
    origin = [0, 0, 0]

    # Define the axis vectors
    x_axis = [1, 0, 0]
    y_axis = [0, 1, 0]
    z_axis = [0, 0, 1]

    

    # Plot the arrows
    ax.quiver(*origin, *x_axis, color='r', length=1, normalize=True, arrow_length_ratio=0.1)
    ax.quiver(*origin, *y_axis, color='g', length=1, normalize=True, arrow_length_ratio=0.1)
    ax.quiver(*origin, *z_axis, color='b', length=1, normalize=True, arrow_length_ratio=0.1)

    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])
    ax.set_aspect('equal')
    plt.show()
    