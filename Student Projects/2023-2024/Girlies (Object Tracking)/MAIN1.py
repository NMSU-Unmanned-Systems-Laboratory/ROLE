from djitellopy import Tello
import time
import cv2

width = 720
height = 480
StartCounter = 1

thres = 0.55
nmsThres = 240
classNames = []
classFile = 'coco.names'

with open(classFile, 'rt') as f:
    classNames = f.read().split('\n')
#print(classNames)
configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

warning_threshold = 500

me = Tello()
me.connect()
me.takeoff()

#me.for_back_velocity = 0
#me.left_right_velocity = 0
#me.up_down_velocity = 0
#me.yaw_velocity = 0
#me.speed = 10
#me.

print("Battery level:", me.get_battery())


me.streamon()

stop = False

objDist = float('inf')
objCoef = 0.003 # This is a number to take an object's square pixels and convert it to distance in meters away
# May be better to have coefficients for each width and height instead of square pixels, but this should work for now

while True:
    startT = time.time()

    img = me.get_frame_read().frame
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    classIds, confs, bbox = net.detect(img,confThreshold = thres, nmsThreshold=nmsThres)


    try:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cv2.rectangle(img, box, color=(0, 255, 0))
            cv2.putText(img, f'{classNames[classId - 1].upper()}{round(conf*100, 2)}',
                    (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                    1, (0,255,0),2)
           
            if classId == 13:

                center_x = width // 2  
                center_y = height // 2  
                tolerance = 0  
                forward_speed = 20  
                # Calculate object distance from center of the frame
                obj_dist_x = box[0] + box[2] // 2 - center_x
                obj_dist_y = box[1] + box[3] // 2 - center_y
                # Adjust drone's movements based on object position
                if abs(obj_dist_x) > tolerance:
                    if obj_dist_x > 0:
                        me.send_rc_control(0, forward_speed, 0, 20)
                    else:
                        me.send_rc_control(0, forward_speed, 0, -20)
                elif abs(obj_dist_y) > tolerance:
                        if obj_dist_y > 0:
                            me.send_rc_control(0, forward_speed, -20, 0)                              
                        else:
                            me.send_rc_control(0, forward_speed, 20, 0)  # Move up
                else:
                    me.send_rc_control(0, forward_speed, 0, 0)  

                # Display the warning message and set the stop flag if needed   
                if box[2] > warning_threshold:
                    cv2.putText(img, 'Warning: Large Object!', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    print("Large object detected, stopping...")
                    stop = True

            else:
                me.send_rc_control(0,0,0,0)

    except Exception as e:
        print("Error:",e)


    if ((cv2.waitKey(1) & 0xFF == ord('q')) or stop):
        break

    cv2.imshow('MyResult', img)
    while (time.time() - startT<= 1/10):
        pass
cv2.destroyAllWindows()
me.streamoff()
me.land()
me.end()
