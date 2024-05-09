
import pygame
import math
import rospy
from simple_pid import PID
from djitellopy import Tello
from nav_msgs.msg import Odometry
import tf
import threading
import json
import time
from dt_apriltags import Detector
import numpy
import os
import cv2
import scipy.spatial.transform 

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
camera_params = ( 921.17072,919.018377 ,459.904345 ,351.238301 )

#tello = Tello(host='192.168.10.1')
#tello.connect()
#print(tello.get_battery())
#tello.streamon()


#telloVideo = cv2.VideoCapture("udp://192.168.10.1:11111")

def callback(msg):
    global dronePos
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw_rad = euler[2] - (math.pi/2)

    if (yaw_rad < -(math.pi)): 
        yaw_rad += (2 * math.pi)

    dronePos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_rad]

def json_waypoint(xPID, yPID, yawPID, waypoints):
    global run
    xPID.setpoint = waypoints[0][0]
    yPID.setpoint = waypoints[0][1]
    
    CurrI = 1
    lastTime = time.time()

    while(run):
        currTime = time.time()
        if (currTime - lastTime >= 3):
            if CurrI == len(waypoints):
                run = False
                break

            xPID.setpoint = waypoints[CurrI][0]
            yPID.setpoint = waypoints[CurrI][1]
            # Print the current waypoint values
            print("Waypoint {}: ({}, {})".format(CurrI, waypoints[CurrI][0], waypoints[CurrI][1]))
            
            CurrI = CurrI + 1
            #CurrI += 1

            lastTime = currTime


class Background(pygame.sprite.Sprite):
    def __init__(self, image, location, scale):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(image)
        self.image = pygame.transform.rotozoom(self.image, 0, scale)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location   


def realtime(xPID, yPID, yawPID):
    global run
    xPID.setpoint = 0
    yPID.setpoint = 0
    
    index = 0
    MAP_SIZE_COEFF = 1
    Y_OFFSET = 284
    X_OFFSET = 494
    X_COF = -0.006498 # -0.004008, -0.00271
    Y_COF = -0.006498 # -0.004008, -0.00271

    #pygame.init()
    screen = pygame.display.set_mode([960, 540])
    screen.fill((255, 255, 255))

    bground = Background('lab.png', [0, 0], 0.5)
    screen.blit(bground.image, bground.rect)

    while (run):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                boundaryx = (pos[1] - Y_OFFSET)*Y_COF
                boundaryy = (pos[0] - X_OFFSET)*X_COF
                if (boundaryx < 1 and boundaryx > -1 ) and (boundaryy < 1 and boundaryy > -1):
                    print('point is good')


                
                    xPID.setpoint = boundaryx
                    yPID.setpoint = boundaryy
                #toAppend = ((pos[1] - Y_OFFSET)*Y_COF, (pos[0]- X_OFFSET)*X_COF)
                
                #if index > 0:
                    #pygame.draw.line(screen, (255, 0, 0), (yPID/X_COF + X_OFFSET, xPID/Y_COF + Y_OFFSET), pos, 2)
                #index += 1
        

def images(telloVideo):
    global img, run, ok

    while(run):
        ret, temp = telloVideo.read()

        if ret:
            ok = True
            img = temp
        else:
            ok = False






def main():
    global dronePos, run, img, ok
    img = None
    ok = False

    rospy.init_node('myNode')
    rospy.Subscriber('/mocap_node/tello_09/Odom', Odometry, callback)

    myTel = Tello(host='192.168.10.1')
    myTel.connect()
    print(myTel.get_battery())
    myTel.streamon()

    telloVideo = cv2.VideoCapture("udp://192.168.10.1:11111")
    telloVideo.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    userInput = input("Use real-time Pathing? (y/n) or 'exit'\n")

    if userInput == "y":
        useRealtime = True
    elif userInput == 'exit':
        myTel.end()
    else:
        useRealtime = False

    
    myTel.takeoff()

    #time.sleep('2')
    
    pygame.init()

    WIN_WIDTH = 500
    WIN_HEIGHT = 500

    
    

    # Open the JSON file
    with open('waypoint.json', 'r') as f:
        data = json.load(f)

    # Access the 'wp' and 'pos' lists from the loaded JSON data
    waypoints = data['pos']


    run = True

    xPos = 0
    yPos = 0
    xVel = 0
    yVel = 0
    xAcl = 0
    yAcl = 0
    xConst = (WIN_HEIGHT/2) / 1
    yConst = (WIN_WIDTH/2) / 1.5

    dronePos = [0, 0, 0]

    xPID = PID(105, 0.6, 40, setpoint=0)
    yPID = PID(105, 0.6, 40, setpoint=0)
    yawPID = PID(105, 0.6, 40, setpoint=0)


    if (useRealtime):
        t1 = threading.Thread(target=realtime, args=[xPID, yPID, yawPID])
        t1.start()
    else:
        t1 = threading.Thread(target=json_waypoint, args=[xPID, yPID, yawPID, waypoints])
        t1.start()
        win = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))
        pygame.display.set_caption("Dronsito")
        win.fill((0,0,0))


    t2 = threading.Thread(target=images, args=[telloVideo])
    t2.start()


    time.sleep(0.05)

    
    victims = []


    while(not ok):
        pass

    
    startT = time.time() 
    camTime = startT + 1/20

    while run:
        if (not useRealtime):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False

        xAcl = xPID(dronePos[0])
        yAcl = yPID(dronePos[1])
        #yawValue = yawPID(dronePos[2])

        rotated_x = xAcl * math.cos(-dronePos[2]) - yAcl * math.sin(-dronePos[2])
        rotated_y = xAcl * math.sin(-dronePos[2]) + yAcl * math.cos(-dronePos[2])

        myTel.send_rc_control(round(-rotated_y), round(rotated_x), 0, 20) #round(-yawValue)

        #myTel.send_keepalive()

    #camera code with april tags

    #ret, img = telloVideo.read()
        
        

        

        #img = myTel.get_frame_read().frame

        #ret, img = telloVideo.read()

        
        if (time.time() - camTime > 1/5 and ok):

            try:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                tags = at_detector.detect(img, True, camera_params, 0.1508)

                if (len(tags) > 0):

                    tagPos = tags[0].pose_t
                    print('pose:', tagPos)

                        #r - > [0, 0, 0] angles
                        #pose[0]= left to right
                        #pose[2] = howfar away (forward and backward)
                        #angles[1] = yaw 
            
                    tagX = (tagPos[2][0]+0.05) * math.cos(dronePos[2]) + tagPos[0][0] * math.sin(dronePos[2])
                    tagY = (tagPos[2][0]+0.05) * math.sin(dronePos[2]) - tagPos[0][0] * math.cos(dronePos[2])


                    vicX = dronePos[0]+tagX
                    vicY = dronePos[1]+tagY

                    save = True

                    for i, v in enumerate(victims):
                        if (math.sqrt((v[0]-vicX)**2 + (v[1]-vicY)**2) < 1):
                            save = False

                            victims[i] = ((v[0]+vicX)/2,(v[1]+vicY)/2)

                            break

                    if save:
                        victims.append((vicX, vicY))
            except:
                pass


            camTime = time.time()
        

        #tello.send_control_command("command")

        pygame.display.update()
  
        while(time.time() - startT <= 1/20):
            pass

        startT = time.time()

    
            

    
    myTel.land()
    myTel.streamoff()
    myTel.end()
    pygame.quit()

    print('Victims Found at:', victims)



if __name__ == '__main__': 
    print("Starting main()")
    rospy.init_node('myNode')
    main()
    print("Done!!")
