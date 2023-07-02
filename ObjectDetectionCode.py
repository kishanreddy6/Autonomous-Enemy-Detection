# import the OpenCV library for computer vision
import cv2
import RPi.GPIO as GPIO
from time import sleep
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TrigPin=16
GPIO.setup(TrigPin,GPIO.IN)
LEDPin=21
GPIO.setup(LEDPin,GPIO.OUT)
ServoPin=20
GPIO.setup(ServoPin,GPIO.OUT)

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters_create()
camera = cv2.VideoCapture(0)

pwm=GPIO.PWM(ServoPin, 50)
pwm.start(0)
    
while True:

    _, img = camera.read()
    sleep(0.1)
    inputValue=GPIO.input(TrigPin)
    
    if(inputValue == True):
        # creates an "img" var that takes in a camera frame
#         _, img = camera.read()
#         sleep(0.1)

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # detect aruco tags within the frame
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

        # draw box around aruco marker within camera frame
        img = cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
        print(markerIds)
        
        if (int (markerIds or 50) >= 10 and (markerIds or 50)<=20):
            
            GPIO.output(LEDPin, GPIO.HIGH)
            sleep(1)
            GPIO.output(LEDPin, GPIO.LOW)
            sleep(1)
            pwm.ChangeDutyCycle(6.5) # move to 90 deg forward to hit object
            sleep(0.5)
            pwm.ChangeDutyCycle(1.5) # move back to original position
            sleep(0.5)
            pwm.ChangeDutyCycle(0) # to make jitters go
            sleep(0.5)
            PwmDuty=0
        
        else:
            
            GPIO.output(LEDPin, GPIO.HIGH)
            sleep(0.1)
            GPIO.output(LEDPin, GPIO.LOW)
            sleep(0.1)
            GPIO.output(LEDPin, GPIO.HIGH)
            sleep(0.1)
            GPIO.output(LEDPin, GPIO.LOW)
            sleep(1)
                      
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()