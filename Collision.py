import numpy as np
import time
from graphics import *

class ball:

    def __init__(self,x,y,xv,yv,rad,mass):
        self.xpos = float(x)
        self.ypos = float(y)
        self.xvel = float(xv)
        self.yvel = float(yv)
        self.radius = float(rad)
        self.mass = float(mass)
        self.ivel = 0

    def update(self, time):
        ## Updates position
        self.xpos = self.xpos + time*self.xvel
        self.ypos = self.ypos + time*self.yvel

    def collide(self, collider):
        colx = collider.xpos
        coly = collider.ypos
        ## Checks if radius of balls overlap to see if they collide
        distance = ((colx-self.xpos)**2+(coly-self.ypos)**2)
        if distance < (collider.getrad()+self.radius)**2:
            return True

    def resolvecol(self, collider):
        ## This all looks terrifying but it's basically the momentum formula copy-pasted
        vel1 = np.sqrt(self.xvel**2 + self.yvel**2)
        vel2 = self.ivel
        theta1 = np.arctan2(self.yvel, self.xvel)
        theta2 = self.itheta
        gamma = np.arctan2(collider.getY()-self.ypos, collider.getX()-self.xpos)
        vela = (vel1*np.cos(theta1-gamma)*(self.mass-collider.getmass()))+(2*collider.getmass()*vel2*np.cos(theta2-gamma))
        velx = (vela/(self.mass+collider.getmass()))*np.cos(gamma)+vel1*np.sin(theta1-gamma)*np.cos(gamma+np.pi/2)
        vely = (vela/(self.mass+collider.getmass()))*np.sin(gamma)+vel1*np.sin(theta1-gamma)*np.sin(gamma+np.pi/2)
        self.xvel = velx
        self.vely = vely

    def edgefix(self,collider):
        ## Moves the balls to the edge so the collision doesn't loop infinitely
        ## Has to be made due to inaccuracy of interval
        dis = np.sqrt((collider.getY()-self.ypos)**2 + (collider.getX()-self.xpos)**2)
        gamma = np.arctan2(collider.getY()-self.ypos, collider.getX()-self.xpos)
        gap = collider.getrad()+self.radius - dis
        self.xpos = self.xpos + (gap)*np.cos(gamma + np.pi)
        self.ypos = self.ypos + (gap)*np.sin(gamma + np.pi)

    def imprint(self,collider):
        ## Records pre-collision values
        self.ivel = np.sqrt(collider.getXv()**2 + collider.getYv()**2)
        self.itheta = np.arctan2(collider.getYv(), collider.getXv())
        
    def getY(self):
        return self.ypos

    def getX(self):
        return self.xpos

    def getYv(self):
        return self.yvel

    def getXv(self):
        return self.xvel

    def getrad(self):
        return self.radius

    def getmass(self):
        return self.mass

def intialize(win):
    ## Creates balls for GUI
    circles = []
    for i in range(len(balls)):
        circles = circles + [Circle(Point(balls[i].getX(),balls[i].getY()),balls[i].getrad())]
        circles[i].draw(win)
    return circles

def updater(circles,win,i,interval):
    ## Updates position of balls and GUI
    circles[i].undraw()
    balls[i].update(interval)
    circles[i] = Circle(Point(balls[i].getX(),balls[i].getY()),balls[i].getrad())
    circles[i].draw(win)
    for k in range(len(circles)):
        if k != i:
            if balls[i].collide(balls[k]):
                collisionprocess(k,i)

def collisionprocess(k,i):
    ## Puts all the collsion processes together
    balls[i].imprint(balls[k])
    balls[k].imprint(balls[i])
    balls[i].resolvecol(balls[k])
    balls[i].edgefix(balls[k])
    balls[k].resolvecol(balls[i])

def store(texts,c,i):
    texts[c] = texts[c] + "Run %6.0f" % (i) + " %11.2f" % (balls[c].getX()) + "m" + " %13.2f" % (balls[c].getY()) + "m" \
               + " %13.2f" % (balls[c].getXv()) + "m" + " %12.2f" % (balls[c].getYv()) + "m" + "\n"

def setup():
    global balls
    ## ball(x,y,xvel,yvel,rad,mass)
    balls = [ball(5,5,10,0,1,2),ball(10,5,0,0,1,1)]

def main():
    setup()

    ## Sets up GUI
    win = GraphWin("Collision", 500, 500)
    win.setCoords(-25,-25,25,25)
    circles = intialize(win)

    ## Interval of Simulation
    interval = 0.01

    ## Number of intervals
    length = 100

    outfile = open("balls", "w")

    ## Text storage
    texts = ['']*len(balls)
    header = "Interval     X Position     Y Position     X velocity    Y velocity"
    ## Interval of recording
    rinterval = 10
    p = rinterval
    
    for i in range(length):
        for k in range(len(circles)):
            updater(circles,win,k,interval)
            time.sleep(0.01)

            if (p >=rinterval):
                    p = 0
                    store(texts,k,i)
            else:
                p = p + 1

    print header
    outfile.write(header + "\n")
    for i in range(len(texts)):
        print "Body" + str(i+1)
        outfile.write("Body" + str(i+1) + "\n")
        print texts[i]
        outfile.write(texts[i] + "\n")
            
    time.sleep(0.1)
    win.close()

if __name__ == '__main__':
    main()
