import cv2
import sys

# Get user supplied values
#imagePath = sys.argv[1]
#cascPath = sys.argv[2]

imagePath = 'abba.png'
cascFacePath = 'haarcascade_frontalface_default.xml'
cascEyePath = 'haarcascade_eye.xml'

# Create the haar cascade
faceCascade = cv2.CascadeClassifier(cascFacePath)
eyeCascade = cv2.CascadeClassifier(cascEyePath)

# Read the image
image = cv2.imread(imagePath)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect faces in the image
faces = faceCascade.detectMultiScale(
    gray,
    scaleFactor=1.1,
    minNeighbors=5,
    minSize=(30, 30),
    flags = cv2.cv.CV_HAAR_SCALE_IMAGE
)   

print "Found {0} faces!".format(len(faces))

# Draw a rectangle around the faces
for (x, y, w, h) in faces:
    cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = image[y:y+h, x:x+w]
    eyes = eyeCascade.detectMultiScale(roi_gray)
    for (ex,ey,ew,eh) in eyes:
        cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

cv2.imshow("Faces found", image)
cv2.waitKey(0)
