# when curve in edit mode

import pymel.core as pm
import maya.cmds as mc
import csv
#select the curve and call
sel = pm.selected()
for i in sel[0]:
    print(i.getCV)


