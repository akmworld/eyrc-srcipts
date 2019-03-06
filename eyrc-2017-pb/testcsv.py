import csv
import cv2
color='Green'
shape='Circle'
f=open('test.csv')
reader=csv.reader(f)
for row in reader:
    if row[0]==color and row[1]==shape:
        flower=cv2.imread(row[2],-1)
            
cv2.imshow('flower',flower)
cv2.waitKey(0)
cv2.destroyAllWindows()
