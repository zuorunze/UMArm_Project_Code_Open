import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection


import numpy as np
from numpy import sqrt
import exponentials_mk5

def print_frame(param, theta0123, initpos_homo, ax):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param

    JD_H = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-(1.)*JD],[0.,0.,0.,1.]]

    initpos_jd_shifted = initpos_homo @ JD_H

    e1234 = exponentials_mk5.get_e1234(param, theta0123)
    e12 = exponentials_mk5.get_e12(param, theta0123)

    frame1 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-UC1],[0,0,0,1]], dtype=float)

    frame2 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2)],[0,0,0,1]], dtype=float)

    pi = np.pi
    RZ45 = np.array([
    [np.cos(pi/4), -np.sin(pi/4), 0],
    [np.sin(pi/4),  np.cos(pi/4), 0],
    [0,             0,            1],
    ], dtype=float)

    frame2[0:3,0:3] = RZ45

    frame1 = initpos_jd_shifted @ frame1
    frame2 = initpos_jd_shifted @ e1234 @ frame2

    origin1 = frame1[0:3,3]
    origin2 = frame2[0:3,3]

    arrow_l = 0.2
    arrow_ratio = 0.1
    # Plot the arrows
    ax.quiver(*origin1, *frame1[0:3,0], color='steelblue', length=arrow_l, normalize=True, arrow_length_ratio=arrow_ratio, linewidth=2)
    ax.quiver(*origin1, *frame1[0:3,1], color='lightskyblue', length=arrow_l, normalize=True, arrow_length_ratio=arrow_ratio, linewidth=2)
    # ax.quiver(*origin1, *frame1[0:3,2], color='b', length=arrow_l, normalize=True, arrow_length_ratio=arrow_ratio)

    # Plot the arrows
    ax.quiver(*origin2, *frame2[0:3,0], color='steelblue', length=arrow_l, normalize=True, arrow_length_ratio=arrow_ratio, linewidth=2)
    ax.quiver(*origin2, *frame2[0:3,1], color='lightskyblue', length=arrow_l, normalize=True, arrow_length_ratio=arrow_ratio, linewidth=2)
    # ax.quiver(*origin2, *frame2[0:3,2], color='b', length=arrow_l, normalize=True, arrow_length_ratio=arrow_ratio)


    return

def print_ee(ee_param, theta12, initpos_homo):
    ee1x, ee1y, ee1z, ee2x, ee2y, ee2z, gstx, gsty, gstz = ee_param

    ee_e12 = exponentials_mk5.get_ee_e12(np.array([ee1x, ee1y, ee1z]), np.array([ee2x, ee2y, ee2z]), theta12)
    ee_e1 = exponentials_mk5.get_ee_e1(np.array([ee1x, ee1y, ee1z]), theta12[0])

    gsp0_00 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    gsp0_01 = np.array([[1,0,0,ee1x],[0,1,0,ee1y],[0,0,1,ee1z],[0,0,0,1]])
    gsp0_02 = np.array([[1,0,0,ee2x],[0,1,0,ee2y],[0,0,1,ee2z],[0,0,0,1]])

    gst0 = np.eye(4)
    gst0[0:3, 3] = np.array([gstx, gsty, gstz])

    
    p0 = (initpos_homo @ gsp0_00)[0:3,3]
    p1 = (initpos_homo @ gsp0_01)[0:3,3]
    p2 = (initpos_homo @ ee_e1 @ gsp0_02)[0:3,3]

    eepos = initpos_homo @ ee_e12 @ gst0
    p3 = eepos[0:3,3]

    x = [p0[0], p1[0], p2[0], p3[0]]
    y = [p0[1], p1[1], p2[1], p3[1]]
    z = [p0[2], p1[2], p2[2], p3[2]]

    lines = np.array([
        [(p0[0],p0[1],p0[2]), (p1[0],p1[1],p1[2])],
        [(p1[0],p1[1],p1[2]), (p2[0],p2[1],p2[2])],
        [(p2[0],p2[1],p2[2]), (p3[0],p3[1],p3[2])],
    ])

    line_collection = Line3DCollection(lines, colors='k', linewidths=0.5)


    return eepos,x,y,z,line_collection


def print_link_details_color(param, theta0123, initpos_homo):

    


    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param

    JD_H = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-(1.)*JD],[0.,0.,0.,1.]]

    initpos_jd_shifted = initpos_homo @ JD_H

    e1234 = exponentials_mk5.get_e1234(param, theta0123)
    e12 = exponentials_mk5.get_e12(param, theta0123)

    # points for scatter plot
    x=[]
    y=[]
    z=[]

    # Bottom Joint 
    gsb0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # joint plate center
        [[1,0,0,JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,-JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,-JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,0],[0,0,1,-UC1],[0,0,0,1]]] # U-Joint center
    
    bottom_nodes = [None]*len(gsb0)

    for i in range(len(gsb0)):
        gsbtheta = gsb0[i]
        bottom_nodes[i] = initpos_jd_shifted @ gsbtheta

    for i in range(len(gsb0)):
        x.append(bottom_nodes[i][0][3])
        y.append(bottom_nodes[i][1][3])
        z.append(bottom_nodes[i][2][3])

    # Link
    gsl0 = [
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 0 - the top actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,sqrt(2)/2*AO2],[0,1,0,sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 1
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,+sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 2
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 3
        [[1,0,0,+sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 4
        
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 5 - the bottom actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 6
        [[1,0,0,0],[0,1,0,AO1],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 7
        [[1,0,0,-AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 8
        [[1,0,0,0],[0,1,0,-AO1],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 9
        ]
    
    link_nodes = [None]*len(gsl0)

    for i in range(len(gsl0)):
        gsltheta = e12 @ gsl0[i]
        link_nodes[i] = initpos_jd_shifted @ gsltheta

    for i in range(len(gsl0)):
        x.append(link_nodes[i][0][3])
        y.append(link_nodes[i][1][3])
        z.append(link_nodes[i][2][3])

    gst0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2)],[0,0,0,1]], # 0- U-Joint center of the bottom joint
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 1- Joint plate center of the bottom joint
        [[1,0,0,sqrt(2)/2*JA2],[0,1,0,sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 2
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,+sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 3
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 4
        [[1,0,0,+sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 5
        ]
    
    top_nodes = [None]*len(gst0)

    for i in range(len(gst0)):
        gsttheta = e1234 @ gst0[i]
        top_nodes[i] = initpos_jd_shifted @ gsttheta

    for i in range(len(gst0)):
        x.append(top_nodes[i][0][3])
        y.append(top_nodes[i][1][3])
        z.append(top_nodes[i][2][3])

    lgsb = len(gsb0)
    lgsbl = len(gsb0) + len(gsl0)

    top_joint_lines = [
        [(x[3],y[3],z[3]),(x[1],y[1],z[1])],
        [(x[4],y[4],z[4]),(x[2],y[2],z[2])],
        [(x[0],y[0],z[0]),(x[5],y[5],z[5])],

        
    ]


    bottom_joint_lines = [
        

        [(x[lgsbl+1],y[lgsbl+1],z[lgsbl+1]),(x[lgsbl+0],y[lgsbl+0],z[lgsbl+0])],
        [(x[lgsbl+2],y[lgsbl+2],z[lgsbl+2]),(x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsbl+3],y[lgsbl+3],z[lgsbl+3]),(x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]

    

    link_lines = [
        [(x[5],y[5],z[5]), (x[lgsb],y[lgsb],z[lgsb])],
        [(x[lgsb+5],y[lgsb+5],z[lgsb+5]),(x[lgsbl+0],y[lgsbl+0],z[lgsbl+0])],


        [(x[lgsb+1],y[lgsb+1],z[lgsb+1]),(x[lgsb+3],y[lgsb+3],z[lgsb+3])],
        [(x[lgsb+2],y[lgsb+2],z[lgsb+2]),(x[lgsb+4],y[lgsb+4],z[lgsb+4])],
        [(x[lgsb+6],y[lgsb+6],z[lgsb+6]),(x[lgsb+8],y[lgsb+8],z[lgsb+8])],
        [(x[lgsb+7],y[lgsb+7],z[lgsb+7]),(x[lgsb+9],y[lgsb+9],z[lgsb+9])],

        [(x[lgsb+0],y[lgsb+0],z[lgsb+0]),(x[lgsb+5],y[lgsb+5],z[lgsb+5])],
    ]

    
    # Lower group actuator
    top_actuator_lines = [
        [(x[1],y[1],z[1]), (x[lgsb+6],y[lgsb+6],z[lgsb+6])],
        [(x[2],y[2],z[2]), (x[lgsb+7],y[lgsb+7],z[lgsb+7])],
        [(x[3],y[3],z[3]), (x[lgsb+8],y[lgsb+8],z[lgsb+8])],
        [(x[4],y[4],z[4]), (x[lgsb+9],y[lgsb+9],z[lgsb+9])],
    ]



    # Upper group actuator

    bottom_actuator_lines = [
        [(x[lgsb+1],y[lgsb+1],z[lgsb+1]), (x[lgsbl+2],y[lgsbl+2],z[lgsbl+2])],
        [(x[lgsb+2],y[lgsb+2],z[lgsb+2]), (x[lgsbl+3],y[lgsbl+3],z[lgsbl+3])],
        [(x[lgsb+3],y[lgsb+3],z[lgsb+3]), (x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsb+4],y[lgsb+4],z[lgsb+4]), (x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]
    
    
    top_link_lines_seg = np.array(top_joint_lines)
    top_link_lines_collection = Line3DCollection(top_link_lines_seg, colors='royalblue', linewidths=5)

    bottom_link_lines_seg = np.array(bottom_joint_lines)
    bottom_link_lines_collection = Line3DCollection(bottom_link_lines_seg, colors='lightskyblue', linewidths=5)

    link_lines_seg = np.array(link_lines)
    link_lines_collection = Line3DCollection(link_lines_seg, colors='gray', linewidths=2)

    top_actuator_lines_seg = np.array(top_actuator_lines)
    top_actuator_lines_collection = Line3DCollection(top_actuator_lines_seg, colors=('royalblue', 0.8), linewidths=8)

    bottom_actuator_lines_seg = np.array(bottom_actuator_lines)
    bottom_actuator_lines_collection = Line3DCollection(bottom_actuator_lines_seg, colors=('lightskyblue', 0.8), linewidths=8)

    return [top_nodes[1],x,y,z,top_link_lines_collection, link_lines_collection, bottom_link_lines_collection, bottom_actuator_lines_collection,top_actuator_lines_collection]



def print_link_color(param, theta0123, initpos_homo, c1='royalblue', c2='lightcoral', c3='lightcoral', sz=1):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param

    JD_H = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-(1.)*JD],[0.,0.,0.,1.]]

    initpos_jd_shifted = initpos_homo @ JD_H

    e1234 = exponentials_mk5.get_e1234(param, theta0123)
    e12 = exponentials_mk5.get_e12(param, theta0123)

    # points for scatter plot
    x=[]
    y=[]
    z=[]

    # Bottom Joint 
    gsb0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # joint plate center
        [[1,0,0,JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,-JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,-JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,0],[0,0,1,-UC1],[0,0,0,1]]] # U-Joint center
    
    bottom_nodes = [None]*len(gsb0)

    for i in range(len(gsb0)):
        gsbtheta = gsb0[i]
        bottom_nodes[i] = initpos_jd_shifted @ gsbtheta

    for i in range(len(gsb0)):
        x.append(bottom_nodes[i][0][3])
        y.append(bottom_nodes[i][1][3])
        z.append(bottom_nodes[i][2][3])

    # Link
    gsl0 = [
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 0 - the top actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,sqrt(2)/2*AO2],[0,1,0,sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 1
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,+sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 2
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 3
        [[1,0,0,+sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 4
        
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 5 - the bottom actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 6
        [[1,0,0,0],[0,1,0,AO1],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 7
        [[1,0,0,-AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 8
        [[1,0,0,0],[0,1,0,-AO1],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 9
        ]
    
    link_nodes = [None]*len(gsl0)

    for i in range(len(gsl0)):
        gsltheta = e12 @ gsl0[i]
        link_nodes[i] = initpos_jd_shifted @ gsltheta

    for i in range(len(gsl0)):
        x.append(link_nodes[i][0][3])
        y.append(link_nodes[i][1][3])
        z.append(link_nodes[i][2][3])

    gst0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2)],[0,0,0,1]], # 0- U-Joint center of the bottom joint
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 1- Joint plate center of the bottom joint
        [[1,0,0,sqrt(2)/2*JA2],[0,1,0,sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 2
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,+sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 3
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 4
        [[1,0,0,+sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 5
        ]
    
    top_nodes = [None]*len(gst0)

    for i in range(len(gst0)):
        gsttheta = e1234 @ gst0[i]
        top_nodes[i] = initpos_jd_shifted @ gsttheta

    for i in range(len(gst0)):
        x.append(top_nodes[i][0][3])
        y.append(top_nodes[i][1][3])
        z.append(top_nodes[i][2][3])

    lgsb = len(gsb0)
    lgsbl = len(gsb0) + len(gsl0)

    link_lines = [
        [(x[3],y[3],z[3]),(x[1],y[1],z[1])],
        [(x[4],y[4],z[4]),(x[2],y[2],z[2])],
        [(x[0],y[0],z[0]),(x[5],y[5],z[5])],

        [(x[5],y[5],z[5]), (x[lgsb],y[lgsb],z[lgsb])],

        [(x[lgsb+1],y[lgsb+1],z[lgsb+1]),(x[lgsb+3],y[lgsb+3],z[lgsb+3])],
        [(x[lgsb+2],y[lgsb+2],z[lgsb+2]),(x[lgsb+4],y[lgsb+4],z[lgsb+4])],
        [(x[lgsb+6],y[lgsb+6],z[lgsb+6]),(x[lgsb+8],y[lgsb+8],z[lgsb+8])],
        [(x[lgsb+7],y[lgsb+7],z[lgsb+7]),(x[lgsb+9],y[lgsb+9],z[lgsb+9])],

        [(x[lgsb+0],y[lgsb+0],z[lgsb+0]),(x[lgsb+5],y[lgsb+5],z[lgsb+5])],

        [(x[lgsb+5],y[lgsb+5],z[lgsb+5]),(x[lgsbl+0],y[lgsbl+0],z[lgsbl+0])],

        [(x[lgsbl+1],y[lgsbl+1],z[lgsbl+1]),(x[lgsbl+0],y[lgsbl+0],z[lgsbl+0])],
        [(x[lgsbl+2],y[lgsbl+2],z[lgsbl+2]),(x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsbl+3],y[lgsbl+3],z[lgsbl+3]),(x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]

    link_lines_seg = np.array(link_lines)
    link_lines_collection = Line3DCollection(link_lines_seg, colors=c1, linewidths=2 * sz)

    # Lower group actuator
    top_actuator_lines = [
        [(x[1],y[1],z[1]), (x[lgsb+6],y[lgsb+6],z[lgsb+6])],
        [(x[2],y[2],z[2]), (x[lgsb+7],y[lgsb+7],z[lgsb+7])],
        [(x[3],y[3],z[3]), (x[lgsb+8],y[lgsb+8],z[lgsb+8])],
        [(x[4],y[4],z[4]), (x[lgsb+9],y[lgsb+9],z[lgsb+9])],
    ]

    top_actuator_lines_seg = np.array(top_actuator_lines)
    top_actuator_lines_collection = Line3DCollection(top_actuator_lines_seg, colors=c2, linewidths=0.5 * sz)

    # Upper group actuator

    bottom_actuator_lines = [
        [(x[lgsb+1],y[lgsb+1],z[lgsb+1]), (x[lgsbl+2],y[lgsbl+2],z[lgsbl+2])],
        [(x[lgsb+2],y[lgsb+2],z[lgsb+2]), (x[lgsbl+3],y[lgsbl+3],z[lgsbl+3])],
        [(x[lgsb+3],y[lgsb+3],z[lgsb+3]), (x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsb+4],y[lgsb+4],z[lgsb+4]), (x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]

    bottom_actuator_lines_seg = np.array(bottom_actuator_lines)
    bottom_actuator_lines_collection = Line3DCollection(bottom_actuator_lines_seg, colors=c3, linewidths=0.5 * sz)

    return [top_nodes[1],x,y,z,link_lines_collection,bottom_actuator_lines_collection,top_actuator_lines_collection]


def print_link(param, theta0123, initpos_homo):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param

    JD_H = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-(1.)*JD],[0.,0.,0.,1.]]

    initpos_jd_shifted = initpos_homo @ JD_H

    # * -1 is just a patch, get over with it

    e1234 = exponentials_mk5.get_e1234(param, theta0123)
    e12 = exponentials_mk5.get_e12(param, theta0123)

    # points for scatter plot
    x=[]
    y=[]
    z=[]

    textpos_se4 = []

    # Bottom Joint 
    gsb0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # joint plate center
        [[1,0,0,JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,-JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,-JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,0],[0,0,1,-UC1],[0,0,0,1]]] # U-Joint center
    
    TEXT_ARMLEN = JA1 * 1.5

    gsb0_text = [
        [[1,0,0,TEXT_ARMLEN],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,TEXT_ARMLEN],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,-TEXT_ARMLEN],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,-TEXT_ARMLEN],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
    ] # U-Joint center
    
    bottom_nodes = [None]*len(gsb0)

    for i in range(len(gsb0)):
        gsbtheta = gsb0[i]
        bottom_nodes[i] = initpos_jd_shifted @ gsbtheta

    for i in range(len(gsb0)):
        x.append(bottom_nodes[i][0][3])
        y.append(bottom_nodes[i][1][3])
        z.append(bottom_nodes[i][2][3])

    for i in range(4):
        textpos_se4.append(initpos_jd_shifted @ gsb0_text[i])

    # Link
    gsl0 = [
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 0 - the top actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 6
        [[1,0,0,0],[0,1,0,AO1],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 7
        [[1,0,0,-AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 8
        [[1,0,0,0],[0,1,0,-AO1],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 9
        
        
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 5 - the bottom actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,sqrt(2)/2*AO2],[0,1,0,sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 1
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,+sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 2
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 3
        [[1,0,0,+sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 4
        ]
    
    
    link_nodes = [None]*len(gsl0)

    for i in range(len(gsl0)):
        gsltheta = e12 @ gsl0[i]
        link_nodes[i] = initpos_jd_shifted @ gsltheta

    for i in range(len(gsl0)):
        x.append(link_nodes[i][0][3])
        y.append(link_nodes[i][1][3])
        z.append(link_nodes[i][2][3])

    gst0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2)],[0,0,0,1]], # 0- U-Joint center of the bottom joint
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 1- Joint plate center of the bottom joint
        [[1,0,0,sqrt(2)/2*JA2],[0,1,0,sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 2
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,+sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 3
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 4
        [[1,0,0,+sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 5
        ]
    
    TEXT_ARMLEN = sqrt(2)/2*JA2 * 1.5

    gst0_text = [
        [[1,0,0,TEXT_ARMLEN],[0,1,0,TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 2
        [[1,0,0,TEXT_ARMLEN],[0,1,0,TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 3
        [[1,0,0,TEXT_ARMLEN],[0,1,0,TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 4
        [[1,0,0,TEXT_ARMLEN],[0,1,0,TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 5
    ]
    
    top_nodes = [None]*len(gst0)

    for i in range(len(gst0)):
        gsttheta = e1234 @ gst0[i]
        top_nodes[i] = initpos_jd_shifted @ gsttheta

    for i in range(len(gst0)):
        x.append(top_nodes[i][0][3])
        y.append(top_nodes[i][1][3])
        z.append(top_nodes[i][2][3])

    for i in range(len(gst0_text)):
        textpos_se4.append(initpos_jd_shifted @ e1234 @ gst0_text[i])

    lgsb = len(gsb0)
    lgsbl = len(gsb0) + len(gsl0)

    link_lines = [
        [(x[3],y[3],z[3]),(x[1],y[1],z[1])],
        [(x[4],y[4],z[4]),(x[2],y[2],z[2])],
        [(x[0],y[0],z[0]),(x[5],y[5],z[5])],

        [(x[lgsb+1],y[lgsb+1],z[lgsb+1]),(x[lgsb+3],y[lgsb+3],z[lgsb+3])],
        [(x[lgsb+2],y[lgsb+2],z[lgsb+2]),(x[lgsb+4],y[lgsb+4],z[lgsb+4])],
        [(x[lgsb+6],y[lgsb+6],z[lgsb+6]),(x[lgsb+8],y[lgsb+8],z[lgsb+8])],
        [(x[lgsb+7],y[lgsb+7],z[lgsb+7]),(x[lgsb+9],y[lgsb+9],z[lgsb+9])],

        [(x[lgsb+0],y[lgsb+0],z[lgsb+0]),(x[lgsb+5],y[lgsb+5],z[lgsb+5])],

        [(x[lgsbl+1],y[lgsbl+1],z[lgsbl+1]),(x[lgsbl+0],y[lgsbl+0],z[lgsbl+0])],
        [(x[lgsbl+2],y[lgsbl+2],z[lgsbl+2]),(x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsbl+3],y[lgsbl+3],z[lgsbl+3]),(x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]

    link_lines_seg = np.array(link_lines)
    link_lines_collection = Line3DCollection(link_lines_seg, colors='g', linewidths=1.5)

    # Lower group actuator
    top_actuator_lines = [
        [(x[1],y[1],z[1]), (x[lgsb+1],y[lgsb+1],z[lgsb+1])],
        [(x[2],y[2],z[2]), (x[lgsb+2],y[lgsb+2],z[lgsb+2])],
        [(x[3],y[3],z[3]), (x[lgsb+3],y[lgsb+3],z[lgsb+3])],
        [(x[4],y[4],z[4]), (x[lgsb+4],y[lgsb+4],z[lgsb+4])],
    ]

    top_actuator_lines_seg = np.array(top_actuator_lines)
    top_actuator_lines_collection = Line3DCollection(top_actuator_lines_seg, colors='r', linewidths=0.5)

    # Upper group actuator

    bottom_actuator_lines = [
        [(x[lgsb+6],y[lgsb+6],z[lgsb+6]), (x[lgsbl+2],y[lgsbl+2],z[lgsbl+2])],
        [(x[lgsb+7],y[lgsb+7],z[lgsb+7]), (x[lgsbl+3],y[lgsbl+3],z[lgsbl+3])],
        [(x[lgsb+8],y[lgsb+8],z[lgsb+8]), (x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsb+9],y[lgsb+9],z[lgsb+9]), (x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]

    bottom_actuator_lines_seg = np.array(bottom_actuator_lines)
    bottom_actuator_lines_collection = Line3DCollection(bottom_actuator_lines_seg, colors='b', linewidths=0.5)

    return [top_nodes[1],x,y,z,link_lines_collection,bottom_actuator_lines_collection,top_actuator_lines_collection]







def print_link_mk7(param, theta0123, initpos_homo):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param

    JD_H = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-(1.)*JD],[0.,0.,0.,1.]]

    initpos_jd_shifted = initpos_homo @ JD_H

    # * -1 is just a patch, get over with it

    e1234 = exponentials_mk5.get_e1234(param, theta0123)
    e12 = exponentials_mk5.get_e12(param, theta0123)

    # points for scatter plot
    x=[]
    y=[]
    z=[]

    textpos_se4 = []

    # Bottom Joint 
    gsb0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # joint plate center
        [[1,0,0,-JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,-JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,+JA1],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,+JA1],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,0],[0,0,1,-UC1],[0,0,0,1]]] # U-Joint center
    
    TEXT_ARMLEN = JA1 * 1.5

    gsb0_text = [
        [[1,0,0,-TEXT_ARMLEN],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,-TEXT_ARMLEN],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,TEXT_ARMLEN],[0,1,0,0],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
        [[1,0,0,0],[0,1,0,TEXT_ARMLEN],[0,0,1,0],[0,0,0,1]], # actuator attach point on joint plate
    ] # U-Joint center
    
    bottom_nodes = [None]*len(gsb0)

    for i in range(len(gsb0)):
        gsbtheta = gsb0[i]
        bottom_nodes[i] = initpos_jd_shifted @ gsbtheta

    for i in range(len(gsb0)):
        x.append(bottom_nodes[i][0][3])
        y.append(bottom_nodes[i][1][3])
        z.append(bottom_nodes[i][2][3])

    for i in range(4):
        textpos_se4.append(initpos_jd_shifted @ gsb0_text[i])

    # Link
    gsl0 = [
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 0 - the top actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,-AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 6
        [[1,0,0,0],[0,1,0,-AO1],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 7
        [[1,0,0,+AO1],[0,1,0,0],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 8
        [[1,0,0,0],[0,1,0,+AO1],[0,0,1,-1*(UC1+AA1)],[0,0,0,1]], # 9
        
        
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 5 - the bottom actuator attach center on the link
        # actuator attach points on the link
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 1
        [[1,0,0,+sqrt(2)/2*AO2],[0,1,0,-sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 2
        [[1,0,0,+sqrt(2)/2*AO2],[0,1,0,+sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 3
        [[1,0,0,-sqrt(2)/2*AO2],[0,1,0,+sqrt(2)/2*AO2],[0,0,1,-1*(UC1+AA1+LL)],[0,0,0,1]], # 4
        ]
    
    
    link_nodes = [None]*len(gsl0)

    for i in range(len(gsl0)):
        gsltheta = e12 @ gsl0[i]
        link_nodes[i] = initpos_jd_shifted @ gsltheta

    for i in range(len(gsl0)):
        x.append(link_nodes[i][0][3])
        y.append(link_nodes[i][1][3])
        z.append(link_nodes[i][2][3])

    gst0 = [[[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2)],[0,0,0,1]], # 0- U-Joint center of the bottom joint
        [[1,0,0,0],[0,1,0,0],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 1- Joint plate center of the bottom joint
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 2
        [[1,0,0,+sqrt(2)/2*JA2],[0,1,0,-sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 3
        [[1,0,0,+sqrt(2)/2*JA2],[0,1,0,+sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 4
        [[1,0,0,-sqrt(2)/2*JA2],[0,1,0,+sqrt(2)/2*JA2],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 5
        ]
    
    TEXT_ARMLEN = sqrt(2)/2*JA2 * 1.5

    gst0_text = [
        [[1,0,0,-TEXT_ARMLEN],[0,1,0,-TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 2
        [[1,0,0,+TEXT_ARMLEN],[0,1,0,-TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]], # 3
        [[1,0,0,+TEXT_ARMLEN],[0,1,0,+TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 4
        [[1,0,0,-TEXT_ARMLEN],[0,1,0,+TEXT_ARMLEN],[0,0,1,-1*(UC1+AA1+LL+AA2+UC2)],[0,0,0,1]],# 5
    ]
    
    top_nodes = [None]*len(gst0)

    for i in range(len(gst0)):
        gsttheta = e1234 @ gst0[i]
        top_nodes[i] = initpos_jd_shifted @ gsttheta

    for i in range(len(gst0)):
        x.append(top_nodes[i][0][3])
        y.append(top_nodes[i][1][3])
        z.append(top_nodes[i][2][3])

    for i in range(4):
        textpos_se4.append(initpos_jd_shifted @ e1234 @ gst0_text[i])

    lgsb = len(gsb0)
    lgsbl = len(gsb0) + len(gsl0)

    link_lines = [
        [(x[3],y[3],z[3]),(x[1],y[1],z[1])],
        [(x[4],y[4],z[4]),(x[2],y[2],z[2])],
        [(x[0],y[0],z[0]),(x[5],y[5],z[5])],

        [(x[lgsb+1],y[lgsb+1],z[lgsb+1]),(x[lgsb+3],y[lgsb+3],z[lgsb+3])],
        [(x[lgsb+2],y[lgsb+2],z[lgsb+2]),(x[lgsb+4],y[lgsb+4],z[lgsb+4])],
        [(x[lgsb+6],y[lgsb+6],z[lgsb+6]),(x[lgsb+8],y[lgsb+8],z[lgsb+8])],
        [(x[lgsb+7],y[lgsb+7],z[lgsb+7]),(x[lgsb+9],y[lgsb+9],z[lgsb+9])],

        [(x[lgsb+0],y[lgsb+0],z[lgsb+0]),(x[lgsb+5],y[lgsb+5],z[lgsb+5])],

        [(x[lgsbl+1],y[lgsbl+1],z[lgsbl+1]),(x[lgsbl+0],y[lgsbl+0],z[lgsbl+0])],
        [(x[lgsbl+2],y[lgsbl+2],z[lgsbl+2]),(x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsbl+3],y[lgsbl+3],z[lgsbl+3]),(x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ]

    link_lines_seg = np.array(link_lines)
    link_lines_collection = Line3DCollection(link_lines_seg, colors='g', linewidths=1.5)

    # Lower group actuator
    top_actuator_lines = [
        [(x[1],y[1],z[1]), (x[lgsb+1],y[lgsb+1],z[lgsb+1])],
        [(x[2],y[2],z[2]), (x[lgsb+2],y[lgsb+2],z[lgsb+2])],
        [(x[3],y[3],z[3]), (x[lgsb+3],y[lgsb+3],z[lgsb+3])],
        [(x[4],y[4],z[4]), (x[lgsb+4],y[lgsb+4],z[lgsb+4])],
    ]

    top_actuator_lines_seg = np.array(top_actuator_lines)
    top_actuator_lines_collection = Line3DCollection(top_actuator_lines_seg, colors='r', linewidths=0.5)

    # Upper group actuator

    bottom_actuator_lines = np.array([
        [(x[lgsb+6],y[lgsb+6],z[lgsb+6]), (x[lgsbl+2],y[lgsbl+2],z[lgsbl+2])],
        [(x[lgsb+7],y[lgsb+7],z[lgsb+7]), (x[lgsbl+3],y[lgsbl+3],z[lgsbl+3])],
        [(x[lgsb+8],y[lgsb+8],z[lgsb+8]), (x[lgsbl+4],y[lgsbl+4],z[lgsbl+4])],
        [(x[lgsb+9],y[lgsb+9],z[lgsb+9]), (x[lgsbl+5],y[lgsbl+5],z[lgsbl+5])],
    ])

    bottom_actuator_lines_seg = np.array(bottom_actuator_lines)
    bottom_actuator_lines_collection = Line3DCollection(bottom_actuator_lines_seg, colors='b', linewidths=0.5)

    textpos_se4 = np.array(textpos_se4)

    return [top_nodes[1],x,y,z, link_lines_collection,bottom_actuator_lines_collection,top_actuator_lines_collection, textpos_se4]