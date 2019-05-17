#!/usr/bin/env python

import sys
import os

from time import sleep
from operator import itemgetter 

import underworlds
import underworlds.server
from underworlds.tools.loader import ModelLoader
from underworlds.tools.spatial_relations import *
from underworlds.tools.edit import *
from underworlds.helpers.geometry import get_world_transform
from underworlds.helpers.transformations import *

from placement_description import *

if __name__ == "__main__":
    
    ctx = underworlds.Context("Map Testing")
    
    target = numpy.empty((2,6), dtype="a36")
    
    world = []
    
    ModelLoader().load("map1.blend", world="map1")
    ModelLoader().load("map2.blend", world="map2")
    
    time.sleep(5) # leave some time for the loader to finish
    
    world.append(ctx.worlds["map1"])
    world.append(ctx.worlds["map2"])
    
    target[0][0] = world[0].scene.nodebyname("chrome_barrel")[0].id
    target[0][1] = world[0].scene.nodebyname("green_barrel")[0].id
    target[0][2] = world[0].scene.nodebyname("green_barrel.001")[0].id
    target[0][3] = world[0].scene.nodebyname("grey_barrel")[0].id
    target[0][4] = world[0].scene.nodebyname("silver_barrel")[0].id
    target[0][5] = world[0].scene.nodebyname("yellow_barrel")[0].id
    target[1][5] = world[1].scene.nodebyname("chrome_barrel.001")[0].id
    target[1][4] = world[1].scene.nodebyname("green_barrel.002")[0].id
    target[1][3] = world[1].scene.nodebyname("grey_barrel.001")[0].id
    target[1][2] = world[1].scene.nodebyname("silver_barrel.001")[0].id
    target[1][1] = world[1].scene.nodebyname("yellow_barrel.001")[0].id
    target[1][0] = world[1].scene.nodebyname("yellow_barrel.002")[0].id

    for node in world[0].scene.nodes:
        if node.name[0] != '_':
            while len(world[0].scene.nodes[node.id].name.split('.')) > 1 or len(world[0].scene.nodes[node.id].name.split('_')) > 1:
                print "Attempting Format"
                format_name("map1", node.id)
                time.sleep(0.2)
            print world[0].scene.nodes[node.id].name
    
    for node in world[1].scene.nodes:
        if node.name[0] != '_':
            while len(world[1].scene.nodes[node.id].name.split('.')) > 1 or len(world[1].scene.nodes[node.id].name.split('_')) > 1:
                print "Attempting Format"
                format_name("map2", node.id)
                time.sleep(0.2)
            print world[1].scene.nodes[node.id].name
    
    i = 0
    j = 0
    
    na_desc = [[]]
    
    while i < 2:
        if i == 1:
            na_desc.append([])

        target_chk = target[i]
        
        node_chk = []
        
        for node_id in target_chk:
            node_chk.append(world[i].scene.nodes[node_id])
        
        while j < 6:
            node_chk.pop(0)
            desc = gen_spatial_desc(ctx, world[i].name, target[i][j], "default", node_chk,"en_GB","NonAmbig", "locate", False, True, 0)
            na_desc[i].append(str(desc))
            j = j+1
            
        j = 0
        i = i + 1
    
    i = 0
    j = 0
    
    while i < 2:
        while j < 6:
            print na_desc[i][j]
            
            j = j+1
            
        j = 0
        i = i + 1
