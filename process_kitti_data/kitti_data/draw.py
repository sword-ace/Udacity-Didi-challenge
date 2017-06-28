# <todo> not updated from previous version yet !!!!

# from net.common import *
# from net.processing.boxes3d import *
#
# #https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT
# # COLOR = {
# #    'Car': (0,0,1.), 'Van': (1.,0,1.), 'Truck': (1.,0,1.),
# #    'Pedestrian': (1.,1.,0), 'Person (sitting)': (1.,0.5,0),
# #    'Cyclist': (0,0,1.),
# #    'Tram': (0,1.,0),
# #    'Misc': (0.5,0.5,0.5)
# # }
#
# COLOR = {
#    'Car':               (1.,  1.,  1. ),
#    'Van':               (1.,  1.,  1. ),
#    'Truck':             (1.,  1.,  1. ),
#    'Pedestrian':        (1.,  1.,  1. ),
#    'Person (sitting)':  (1.,  1.,  1. ),
#    'Cyclist':           (1.,  1.,  1. ),
#    'Tram':              (1.,  1.,  1. ),
#    'Misc':              (0.5, 0.5, 0.5),
#    'DontCare':          (0.5, 0.5, 0.5)
# }
#
#
# ##MLAB
# MM_AZI=180
# MM_DIS=90
# #MM_FPOINT=[ 12.0909996 , -1.04700089, -2.03249991]
# MM_FPOINT=[ 28 , 0, -2.03249991]
#
#
#
#
#
# ##http://stackoverflow.com/questions/26690932/opencv-rectangle-with-dotted-or-dashed-lines
# def draw_dotted_line(img, pt1, pt2, color, thickness=1,gap=20):
#
#     dist =((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)**.5
#     pts= []
#     for i in  np.arange(0,dist,gap):
#         r=i/dist
#         x=int((pt1[0]*(1-r)+pt2[0]*r)+.5)
#         y=int((pt1[1]*(1-r)+pt2[1]*r)+.5)
#         p = (x,y)
#         pts.append(p)
#
#     if gap==1:
#         for p in pts:
#             cv2.circle(img,p,thickness,color,-1,cv2.LINE_AA)
#     else:
#         def pairwise(iterable):
#             "s -> (s0, s1), (s2, s3), (s4, s5), ..."
#             a = iter(iterable)
#             return zip(a, a)
#
#         for p, q in pairwise(pts):
#             cv2.line(img,p, q, color,thickness,cv2.LINE_AA)
#
#
#
#
# def draw_dotted_poly(img, pts, color, thickness=1, gap=20):
#
#     s=pts[0]
#     e=pts[0]
#     pts.append(pts.pop(0))
#     for p in pts:
#         s=e
#         e=p
#         draw_dotted_line(img,s,e,color,thickness,gap)
#
#
# def draw_dotted_rect(img, pt1, pt2, color, thickness=1, gap=20):
#     pts = [pt1,(pt2[0],pt1[1]),pt2,(pt1[0],pt2[1])]
#     draw_dotted_poly(img, pts, color, thickness, gap)
#
#
#
# def draw_shadow_text(img, text, pt,  fontScale, color, thickness, color1=None, thickness1=None):
#
#     if color1 is None: color1=(0,0,0)
#     if thickness1 is None: thickness1 = thickness+2
#
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     cv2.putText(img, text, pt, font, fontScale, color1, thickness1, cv2.LINE_AA)
#     cv2.putText(img, text, pt, font, fontScale, color,  thickness,  cv2.LINE_AA)
#
# #---------------------------------------------------------------------
#
#
#
# def draw_lidar(fig, lidar, is_grid=False, is_top_region=True, is_fov=True):
#
#     pxs=lidar[:,0]
#     pys=lidar[:,1]
#     pzs=lidar[:,2]
#     prs=lidar[:,3]
#
#     mlab.points3d(
#         pxs, pys, pzs, prs,
#         mode='point',  # 'point'  'sphere'
#         #colormap=(0.7,0.7,0.7),  #'gnuplot',  #'bone',  #'spectral',  #'copper',
#         color=(0.5,0.5,0.5),
#         scale_factor=1,
#         figure=fig)
#
#     #draw grid
#     if is_grid:
#         mlab.points3d(0, 0, 0, color=(1,1,1), mode='sphere', scale_factor=0.2)
#
#         for y in np.arange(-50,50,1):
#             x1,y1,z1 = -50, y, 0
#             x2,y2,z2 =  50, y, 0
#             mlab.plot3d([x1, x2], [y1, y2], [z1,z2], color=(0.5,0.5,0.5), tube_radius=None, line_width=1, figure=fig)
#
#         for x in np.arange(-50,50,1):
#             x1,y1,z1 = x,-50, 0
#             x2,y2,z2 = x, 50, 0
#             mlab.plot3d([x1, x2], [y1, y2], [z1,z2], color=(0.5,0.5,0.5), tube_radius=None, line_width=1, figure=fig)
#
#     #draw axis
#     if 1:
#         axes=np.array([
#             [2.,0.,0.,0.],
#             [0.,2.,0.,0.],
#             [0.,0.,2.,0.],
#         ],dtype=np.float64)
#
#         mlab.points3d(0, 0, 0, color=(1,1,1), mode='sphere', scale_factor=0.2)
#         mlab.plot3d([0, axes[0,0]], [0, axes[0,1]], [0, axes[0,2]], color=(1,0,0), tube_radius=None, figure=fig)
#         mlab.plot3d([0, axes[1,0]], [0, axes[1,1]], [0, axes[1,2]], color=(0,1,0), tube_radius=None, figure=fig)
#         mlab.plot3d([0, axes[2,0]], [0, axes[2,1]], [0, axes[2,2]], color=(0,0,1), tube_radius=None, figure=fig)
#
#     if is_fov:
#         fov=np.array([
#             [math.fabs(TOP_Y_MIN*math.tan(TOP_FOV/180*math.pi)), TOP_Y_MIN, 0.,0.],
#             [math.fabs(TOP_Y_MAX*math.tan(TOP_FOV/180*math.pi)), TOP_Y_MAX, 0.,0.],
#         ],dtype=np.float64)
#
#         mlab.plot3d([0, fov[0,0]], [0, fov[0,1]], [0, fov[0,2]], color=(1,1,1), tube_radius=None, line_width=1, figure=fig)
#         mlab.plot3d([0, fov[1,0]], [0, fov[1,1]], [0, fov[1,2]], color=(1,1,1), tube_radius=None, line_width=1, figure=fig)
#
#     #draw top_image feature area
#     if is_top_region:
#         x1 = TOP_X_MIN
#         x2 = TOP_X_MAX
#         y1 = TOP_Y_MIN
#         y2 = TOP_Y_MAX
#         mlab.plot3d([x1, x1], [y1, y2], [0,0], color=(0.5,0.5,0.5), tube_radius=None, line_width=1, figure=fig)
#         mlab.plot3d([x2, x2], [y1, y2], [0,0], color=(0.5,0.5,0.5), tube_radius=None, line_width=1, figure=fig)
#         mlab.plot3d([x1, x2], [y1, y1], [0,0], color=(0.5,0.5,0.5), tube_radius=None, line_width=1, figure=fig)
#         mlab.plot3d([x1, x2], [y2, y2], [0,0], color=(0.5,0.5,0.5), tube_radius=None, line_width=1, figure=fig)
#
#     mlab.orientation_axes()
#     mlab.view(azimuth=MM_AZI,elevation=None,distance=MM_DIS,focalpoint=MM_FPOINT)#2.0909996 , -1.04700089, -2.03249991
#     #print(mlab.view())
#
#
#
# def draw_boxes3d(fig, boxes3d, is_number=False, color=(1,1,1), line_width=2):
#
#     if boxes3d.shape==(8,3): boxes3d=boxes3d.reshape(1,8,3)
#
#     num = len(boxes3d)
#     for n in range(num):
#         b = boxes3d[n]
#
#         if is_number:
#             mlab.text3d(b[0,0], b[0,1], b[0,2], '%d'%n, scale=(1, 1, 1), color=color, figure=fig)
#         for k in range(0,4):
#
#             #http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
#             i,j=k,(k+1)%4
#             mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
#
#             i,j=k+4,(k+1)%4 + 4
#             mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
#
#             i,j=k,k+4
#             mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
#
#     mlab.view(azimuth=MM_AZI,elevation=None,distance=MM_DIS,focalpoint=MM_FPOINT)#2.0909996 , -1.04700089, -2.03249991
#
#
# def draw_box3d_on_top(image, boxes3d, color=(255,255,255), thickness=1, is_fov=True):
#
#     if boxes3d.shape==(8,3): boxes3d=boxes3d.reshape(1,8,3)
#
#     num =len(boxes3d)
#     for n in range(num):
#         projection = box3d_to_top_projection(boxes3d[n])
#         u0,v0=projection[0]
#         u1,v1=projection[1]
#         u2,v2=projection[2]
#         u3,v3=projection[3]
#         cv2.line(image, (u0,v0), (u1,v1), color, thickness, cv2.LINE_AA)
#         cv2.line(image, (u1,v1), (u2,v2), color, thickness, cv2.LINE_AA)
#         cv2.line(image, (u2,v2), (u3,v3), color, thickness, cv2.LINE_AA)
#         cv2.line(image, (u3,v3), (u0,v0), color, thickness, cv2.LINE_AA)
#
#     if is_fov:
#         top_fov = round(math.fabs(TOP_Y_MIN*math.tan(TOP_FOV/180. *math.pi))/TOP_Y_DIVISION)
#
#         height, width,_ = image.shape
#         cv2.line(image, (0,    height-top_fov), (width//2,height-1), (128,128,128), 1, cv2.LINE_AA)
#         cv2.line(image, (width,height-top_fov), (width//2,height-1), (128,128,128), 1, cv2.LINE_AA)
