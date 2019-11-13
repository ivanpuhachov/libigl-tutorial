# curves in maya -> points in csv file

import pymel.core as pm
import maya.cmds as mc
import csv
sel = pm.selected()
Points = []


for curve in sel:
    crvShape = curve.getShape()
    cv = 0
    while (cv <= (crvShape.numCVs()-1)):
        point = crvShape.getCV(cv)
        cv = cv+1
        Points.append([point[0],point[1],point[2]])
print(Points)

csvFile = open('/home/ivan/projects/libigl-tutorial/cpp_input/mayaOut.csv', 'wb')
# with open("~/temp/3t.csv","wb") as csvFile:
writer = csv.writer(csvFile,quoting=csv.QUOTE_MINIMAL)
for i in range(0,len(Points),1):
    writer.writerow((str(Points[i][0]),str(Points[i][1]),str(Points[i][2])))

csvFile.close()
