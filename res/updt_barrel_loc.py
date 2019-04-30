#!/usr/bin/env python

#This script is designed to be run within blender from within ncnr.blend:
# filename = "/full/path/to/upd_barrel_loc.py"
# exec(compile(open(filename).read(), filename, 'exec'))

import bpy
import csv

with open('ncnr_locations.csv', 'r') as csvfile:
    
    ncnrreader = csv.reader(csvfile, delimiter = ',', quotechar ='|')
    for row in ncnrreader:
        
        if row[0] == 'obj_name':
            pass
        else:
            bpy.data.objects[row[0]].location.x = float(row[1])
            bpy.data.objects[row[0]].location.y = float(row[2])
            bpy.data.objects[row[0]].location.z = float(row[3])
