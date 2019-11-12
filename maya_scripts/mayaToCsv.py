# does not work with nurbscurvecv

import pymel.core as pm
import maya.cmds as mc
import csv
#select the curve and call
sel = pm.selected()
crv = sel[1].getPosition(1)
print(crv)



# def getBezierCVPosAndTan():
#
#     cvLocs = []
#     cvTans = []
#     cv=0
#     while (cv <= (crv.numCVs()-1)):
#         #get CV position
#         cvLoc = crv.getCV(cv)
#         cvLocs.append(cvLoc)
#         #get param at position
#         cvParam = crv.getParamAtPoint(cvLoc)
#         #get tangent at param
#         cvTan = crv.tangent(cvParam)
#         cvTans.append(cvTan)
#         #maya sees the tangent handles as CV's, we skip those
#         cv = cv+3
#     return cvLocs,cvTans
#
# cvLocations,cvTangents = getBezierCVPosAndTan()
#
#
# filePath = "ttt.csv"
# print filePath
# with open(filePath,"wb") as csvFile:
#     writer = csv.writer(csvFile,quoting=csv.QUOTE_MINIMAL)
#     for cv in range(0,len(cvLocations),1):
#         #extract the position and tangent data and save in correct format
#         #we swap some axis around to make the maya coordinates fit with the unreal ones.
#         writer.writerow((str(cvLocations[cv][0]),str(cvLocations[cv][2]),str(cvLocations[cv][1])))
#
# csvFile.close()
