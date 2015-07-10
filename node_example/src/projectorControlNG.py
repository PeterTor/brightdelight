#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
import threading
import math
from threading import Timer, Thread
# OpenGL
try:
    from OpenGL.GL import *
    from OpenGL.GLUT import*
    from OpenGL.GLU import *
    rospy.loginfo("[Vis] OpenGL successfully imported!")
except:
    rospy.logerr("[Vis] PyOpenGL not installed properly. Exiting...")
    exit(1)

g_fViewDistance = 2.14
g_Width = 600
g_Height = 600

g_nearPlane = 1.
g_farPlane = 1000.

action = ""
xStart = yStart = 0.
zoom = 65.

xRotate = 0.
yRotate = 0.
zRotate = 0.

xTrans = 0.
yTrans = 0.
#def for stripe behaivior
translate=20
translateSpeed=0.3

showCrossWalk=False


#--------
# VIEWER
#--------


def init():
    glEnable(GL_NORMALIZE)
    glLightfv(GL_LIGHT0,GL_POSITION,[ .0, 10.0, 10., 0. ] )
    glLightfv(GL_LIGHT0,GL_AMBIENT,[ .0, .0, .0, 1.0 ]);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[ 1.0, 1.0, 1.0, 1.0 ]);
    glLightfv(GL_LIGHT0,GL_SPECULAR,[ 1.0, 1.0, 1.0, 1.0 ]);
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)
    glShadeModel(GL_SMOOTH)
    resetView()


def resetView():
    global zoom, xRotate, yRotate, zRotate, xTrans, yTrans
    zoom = 65.
    xRotate = 0.
    yRotate = 0.
    zRotate = 0.
    xTrans = 0.
    yTrans = 0.
    glutPostRedisplay()

frame=0
time=0
timebase=0
eyeX=0
eyeY=-3.3
centerX=0
centerY=0
centerZ=0
def display():
    global frame,time,timebase,eyeX,eyeY,centerX,centerY,centerZ
    # Clear frame buffer and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # Set up viewing transformation, looking down -Z axis
    #glMatrixMode(GL_PROJECTION)
    #glLoadIdentity()
    #gluLookAt(eyeX, eyeY, -g_fViewDistance, 0, 0, 0, -.1, 0, 0)   #-.1,0,0
    # Set perspective (also zoom)
    #glMatrixMode(GL_MODELVIEW)
    # Render the scene
    #polarView()
    #scenemodel()
    if(showCrossWalk==False):
        displayRoad()
    else:
        displaycrosswalk()
    frame+=1
    time=glutGet(GLUT_ELAPSED_TIME);
    if time - timebase > 1000 :
		#print"FPS: ",frame*1000.0/(time-timebase)
		print"km/h: ",((frame*1000.0/(time-timebase))*translateSpeed)*3.6
		timebase = time;
		frame = 0;
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(zoom, float(g_Width)/float(g_Height), g_nearPlane, g_farPlane)
    gluLookAt(eyeX, eyeY, -g_fViewDistance, centerX, centerY, centerZ, 0, 1, 0)   #-.1,0,0
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glutSwapBuffers()
crosswalkrotation=0
step =0
counter=0

def displaycrosswalk():
    global crosswalkrotation,step,counter
    position=-8
    crosswalkrotation-=20
    if crosswalkrotation <-180:
	counter+=1
	crosswalkrotation=0
    if counter>16:
	counter=0

    for i in range (16):
        glLoadIdentity()
	glTranslatef(position, 0, 0.0)
	if i==counter:
		glRotated(crosswalkrotation,0,1,0)
	
        glBegin(GL_QUADS)
	glColor3d(255,255,255)
        glVertex3f(-0.25, 1.5, 0.0)
        glVertex3f(0.25, 1.5, 0.0)
        glVertex3f(0.25, -1.5, 0.0)
        glVertex3f(-0.25, -1.5, 0.0)
        glEnd();
        
        position +=1.0
def displaycrosswalkShadowWave():
    global crosswalkrotation,step,counter
    position=-8
    crosswalkrotation-=20
    if crosswalkrotation <-180:
	counter+=1
	crosswalkrotation=0
    if counter>16:
	counter=0

    for i in range (16):
	if i!=counter:
        	glLoadIdentity()
		glTranslatef(position, 0, 0.0)
	
	        glBegin(GL_QUADS)
		glColor3d(255,255,255)
	        glVertex3f(-0.25, 1.5, 0.0)
	        glVertex3f(0.25, 1.5, 0.0)
	        glVertex3f(0.25, -1.5, 0.0)
       		glVertex3f(-0.25, -1.5, 0.0)
        	glEnd();
        
        position +=1.0



def displayRoad():
    global translate,translateSpeed
    position=translate
    for i in range(40):
        makeStripe(position)
	#print "stripe %s." % i
 	position+=9
    translate=translate-translateSpeed
    if translate <-200:
        translate=0

#groesse von verkersstreifen: http://www.dsgs.de/uploads/archiv/gesetze/leitfaden_fahrbahnmarkierung_dvr__dsgs.pdf seite 29
def makeStripe(translate):
    #mittelstreifen
    glLoadIdentity()
    glTranslatef(1.90, translate-5, 0.0)
    glBegin(GL_QUADS)
    glColor3d(255,255,255)
    glVertex3f(-0.15, 1.5, 0.0)
    glVertex3f(0.15, 1.5, 0.0)
    glVertex3f(0.15, -1.5, 0.0)
    glVertex3f(-0.15, -1.5, 0.0)
    glEnd();
    
    #seitenstreifen rechts
    glLoadIdentity()
    glTranslatef(-1.90, 0, 0.0)
    
    glBegin(GL_QUADS)
    glColor3d(255,0,0)
    glVertex3f(-0.15, 200.5, 0.0)
    glVertex3f(0.15, 200.5, 0.0)
    glVertex3f(0.15, -100.5, 0.0)
    glVertex3f(-0.15, -100.5, 0.0)
    glEnd();
    
    #seitenstreifen links
    glLoadIdentity()
    glTranslatef(4.75, 0, 0.0)
    
    glBegin(GL_QUADS)
    glColor3d(255,0,0)
    glVertex3f(-0.15, 200.5, 0.0)
    glVertex3f(0.15, 200.5, 0.0)
    glVertex3f(0.15, -100.5, 0.0)
    glVertex3f(-0.15, -100.5, 0.0)
    glEnd();
    

def reshape(width, height):
    global g_Width, g_Height
    g_Width = width
    g_Height = height
    glViewport(0, 0, g_Width, g_Height)


def polarView():
    glTranslatef( yTrans/100., 0.0, 0.0 )
    glTranslatef(  0.0, -xTrans/100., 0.0)
    glRotatef( -zRotate, 0.0, 0.0, 1.0)
    glRotatef( -xRotate, 1.0, 0.0, 0.0)
    glRotatef( -yRotate, .0, 1.0, 0.0)

def initGL(width, height):
    global quadratic

    quadratic = quadratic = gluNewQuadric()

    glClearColor(0.0, 0.0, 0.0, 0.0)   # Clear background color to black.
    glClearDepth(1.0)                  # Enable clearing of the depth buffer.
    glDepthFunc(GL_LESS)               # Type of depth test.
    glEnable(GL_DEPTH_TEST)            # Enable depth testing.
    glShadeModel(GL_SMOOTH)            # Enable smooth color shading.

    glMatrixMode(GL_PROJECTION)   # Specify which matrix is the current matrix.
    glLoadIdentity()              # Reset the projection matrix.

    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)
def keypressed(key,x,y):
    global eyeX,eyeY
    if key=="w":
        eyeY=eyeY+1
    if key=="s":
        eyeY=eyeY-1
    if key=="a":
        eyeX=eyeX+1
    if key=="d":
        eyeX=eyeX-1
    if key=="p":
	sys.exit(0)

    print eyeY
    print math.atan2(g_fViewDistance,eyeY)*180/3.1415


##
## THREADS FOR SUBSCRIBER AND OPENGL
##
class CommandoSubscriber(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running and not rospy.is_shutdown():
            rospy.Subscriber('Speed', Twist, self.SpeedChanger)
            rospy.Subscriber('ShowCrossWalk', Twist, self.ShowCrossWalk)
            rospy.Subscriber('ShowStripes', Twist, self.ShowStripes)
    	    #rospy.Subscriber('start', Twist, self.start)
            rospy.spin()
    def SpeedChanger(self,msg):
	    global translateSpeed
	    translateSpeed = msg.linear.x
        #global zoom, xStart, yStart, xRotate, yRotate, zRotate, xTrans, yTrans
        #xRotate=msg.linear.x
        #yRotate=msg.linear.y
        #glutPostRedisplay()
    def ShowCrossWalk(self,msg):
        global showCrossWalk
        showCrossWalk=True
    def ShowStripes(self,msg):
        global showCrossWalk
        showCrossWalk=False
	
class Visualizer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        global window
        while self.running and not rospy.is_shutdown():
            glutInit(sys.argv)

            glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
            glutInitWindowSize(640, 480)
            glutInitWindowPosition(0, 0)
	    
            # Initialize window so we can close it later.
            window = glutCreateWindow("Display Projection")

            # Register the drawing function with glut.
            glutDisplayFunc(display)

            # When doing nothing, redraw scene.
            glutIdleFunc(display)

            # Register the function called when window is resized.
            glutReshapeFunc(reshape)

            # Register the function called when key is pressed.
            glutKeyboardFunc(keypressed)

            # Initialize window.
            initGL(640, 480)
	    #glutFullScreen()
            # Start event processing engine.
            glutMainLoop()




def main():
    rospy.init_node('Controller')


    try:
        cmd = CommandoSubscriber()
        cmd.start()
        vis = Visualizer()
        vis.start()
        raw_input("Hit <p> to quit.")

        # Stop the loops.
        cmd.running = False
        vis.running = False

        # Wait for threads to finish jobs.
        cmd.join()
        vis.join()

    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    main()




