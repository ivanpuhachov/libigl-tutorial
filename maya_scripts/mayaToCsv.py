# when curve in edit mode -> prints coordinates

import pymel.core as pm
import maya.cmds as mc
import csv
#select the curve and call
sel = pm.selected()
for i in sel[0]:
    print(i.getCV)
