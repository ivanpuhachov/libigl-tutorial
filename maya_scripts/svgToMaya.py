# note: does not work properly

import os
from maya import cmds
import sys
import re
import random
from xml.dom.minidom import parse, parseString

filepath = cmds.fileDialog2(fm=1, startingDirectory ="$HOME", fileFilter="SVG (*.svg)")
dom = parse(filepath[0])

scale = 1
invscale = scale*-1
curvetemp = []

def duplicates(seq,item):
    start_at = -1
    locs = []
    while True:
        try:
            loc = seq.index(item,start_at+1)
        except ValueError:
            break
        else:
            locs.append(loc)
            start_at = loc
    return locs

def splitSVG():
	global total
	svgpoints = re.sub("[mclSQhuC]", "", d)
	svgpoints = svgpoints.split("M")
	svgpoints = filter(None, svgpoints)
	for curve in svgpoints:
		z = ""
		curve = curve.split()
		curve = filter(None, curve)
		curvetemp = []
		for lpos, litem in enumerate(curve, start=0):
			lprev = lpos - 1
			if litem == "L":
				curvetemp.append(curve[lprev])
				curvetemp.append(curve[lprev])
			elif litem == "C":
				curvetemp.append(curve[lprev])
				curvetemp.append(curve[lprev])
			elif litem == "z":
				curvetemp.append(curve[lprev])
				curvetemp.append(curve[lprev])
				curvetemp.append(curve[0])
				curvetemp.append(curve[0])
				curvetemp.append(curve[0])
			else:
				curvetemp.append(curve[lpos])

		curve = curvetemp
		curve = [i for i, next_i in zip(curve, curve[1:] + [None]) if (i, next_i) != ('M', 'M')]
		total = len(curve)
		n1 = total-3
		n2 = total+2
		table = [i for i in range(800) for _ in range(3)][0:n2]
		table = re.sub("[],[]", "", str(table))
		table = re.sub("[ ]", ",", str(table))
		curvecoords = ""
		for coords2d in curve:
			coords3d = re.sub("[,]", " ", coords2d) + " 0"
			coord = coords3d.split(" ")
			coords2d = '(' + coord[0] + ',' + coord[1] +  ',0), '
			curvecoords += coords2d
			print (coords2d)
		comm1 = "cmds.curve(d=1,  p=[" + curvecoords + "] )" # : k=[" + table + "] )"
		comm2 = "cmds.scale( 1, -1, 1 )"
		print(comm1)
		exec comm1
		exec (comm2)
	return curve

def givecoords2d():
	splitSVG()


pathlist= [elt.getAttribute("d") for elt in dom.getElementsByTagName('path')]
for d in pathlist:
	givecoords2d()
	curvetemp = []
