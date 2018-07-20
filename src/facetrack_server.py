import numpy as np
import cv2
import cv2.cv as cv
import facetracker
from video import create_capture
from common import clock, draw_str
import itertools
import scipy.io as sio
from mayavi import mlab
from geventwebsocket.handler import WebSocketHandler
from gevent import pywsgi, sleep

import matplotlib.pyplot as plt
from time import sleep

AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 2
ws = None
cam = None
tracker = None

ace_fn = None
con_fn = None
tri_fn  = None

tracker = None
conns = None
trigs = None

cam = None

shape3D = None
ms = None


help_message = '''
USAGE: facedetect.py [--cascade <cascade_fn>] [--nested-cascade <cascade_fn>] [<video_source>]
'''

def print_cood(data):
    print("---------------")
    for idx in xrange(0, 66):
        print(str(data[0,idx]) + "," + str(data[1,idx]) + "," + str(data[2,idx]))
    print("---------------")

def isClosed(data, threshold):
    max_val = max(data)
    min_val = min(data)
    diff = max_val - min_val
    print(diff)
    if diff < threshold:
        return True
    else:
        return False

def myapp(environ, start_response):
    global ws
    global cam
    global tracker
    global face_fn
    global con_fn
    global tri_fn
    global tracker
    global conns
    global trigs
    global cam
    global shape3D
    global ms

    ws = environ['wsgi.websocket']
    print('enter!')

    ws.send("hoge!")


    plot_flag = False
    try:
        while True:
            t = clock()

            ret, img = cam.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            if tracker.update(gray):
                draw_str(img, (20, 40), 'pos: %.1f, %.1f' % tracker.getPosition())
                draw_str(img, (20, 60), 'scale: %.1f ' % tracker.getScale())
                draw_str(img, (20, 80), 'orientation: %.1f, %.1f, %.1f' % tracker.getOrientation())
                tracker.getScale()
                tracker.getOrientation()
                #img = tracker.draw(img, conns, trigs)
                img = tracker.draw(img, conns, None)

                shape3D = tracker.get3DShape().reshape((3, 66))
                #print_cood(shape3D)

                print(t)

                # #left eye (check max - min on Y axis)
                # if isClosed(shape3D[1][36:41].tolist(), 2) == True:
                #     print("left eye closed")
                #
                # #right eye (check max - min on Y axis)
                # if isClosed(shape3D[1][42:47].tolist(), 2) == True:
                #     print("right eye closed")

                # rip (check max - min on Y axis)
                if isClosed(shape3D[1][60:65].tolist(), 0.7) == True:
                    print("rip closed")
                    if ws != None:
                        ws.send("close")
                        sleep(0.3)
                else:
                    print("rip open")
                    if ws != None:
                        ws.send("open")

                # comment out
                #print shape3D.min(), shape3D.max()

                 #comment out because need less
                 #ms.set(x=shape3D[0, :] , y=shape3D[1, :], z=shape3D[2, :])
            else:
                tracker.setWindowSizes((11, 9, 7))

            dt = clock() - t

            draw_str(img, (20, 20), 'time: %.1f ms' % (dt*1000))
            cv2.imshow('facedetect', img)

            if 0xFF & cv2.waitKey(5) == 27:
                break
    except:
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    import sys, getopt
    print help_message

    args, video_src = getopt.getopt(sys.argv[1:], '', ['face=', 'con=', 'tri='])
    try: video_src = video_src[0]
    except: video_src = 0
    args = dict(args)
    face_fn = args.get('--con', r"../external/FaceTracker/model/face.tracker")
    con_fn = args.get('--con', r"../external/FaceTracker/model/face.con")
    tri_fn  = args.get('--tri', r"../external/FaceTracker/model/face.tri")

    tracker = facetracker.FaceTracker(face_fn)
    conns = facetracker.LoadCon(con_fn)
    trigs = facetracker.LoadTri(tri_fn)

    cam = create_capture(video_src)
    tracker.setWindowSizes((7,))

    shape3D = np.random.randn(3, 66)
    l = mlab.points3d(shape3D[0, :], shape3D[1, :], shape3D[2, :])
    ms = l.mlab_source

    print("Server is running on localhost:8001...")
    server = pywsgi.WSGIServer(('0.0.0.0', 8001), myapp, handler_class=WebSocketHandler)
    server.serve_forever()
