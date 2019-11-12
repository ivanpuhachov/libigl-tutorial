from maya import cmds
import csv

filepath = cmds.fileDialog2(fm=1, startingDirectory ="/home/ivan/projects/libigl-tutorial/cpp_input", fileFilter="CSV (*.csv)")
with open(filepath[0], 'rb') as csvfile:
    spamreader = csv.reader(csvfile)
    coords = ''
    for row in spamreader:
        new_coords = '(' + row[0] +',' + row[1] +',' + row[2] + '),'
        coords+=new_coords
    comm1 = "cmds.curve(d=1,  p=[" + coords + "] )"
    print comm1
    exec comm1
