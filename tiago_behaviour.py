#!/usr/bin/env python

from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_msgs.msg import Empty
from time import sleep
import rospy
import tf
import motion
import numpy
import math
import random
import time
import sys
import os
import pyttsx

import underworlds
import underworlds.server
from underworlds.tools.loader import ModelLoader
from underworlds.tools.spatial_relations import *
from underworlds.tools.edit import *

import csv
from sklearn.svm import SVC
from sklearn.metrics import confusion_matrix
from sklearn.neural_network import MLPClassifier
import pickle

from placement_description import *

state = "idle"
prev_state = "idle"

ctx = underworlds.Context("Tiago Description")

target = numpy.empty((2,7), dtype="a36")
na_desc = [[]]
obj_frame_id = [[]]

order = 0
cond1 = "N"
cond2 = "D"
cur_targ = 0
cur_map = 0
new_desc = True
cur_targ_frame = "1_1_target"
cur_obj_id = ""
d_rel_list = []
iteration = 0
full_desc = ""

tts = pyttsx.init()
#tts.say('Good morning.')
#tts.runAndWait()
#tts.stop() - Might allow an interrupt

target_change = False

def build_classifier():
    
    train = []
    trainresult = []
    test = []
    testresult = []
    
    with open('res/train.csv', 'rb') as csvfile:
    
        trainreader = csv.reader(csvfile, delimiter = ',', quotechar ='|')
        
        for row in trainreader:
            
            if train == []:
                train = numpy.append(train, [row[0], row[1], row[2], row[3]])
            else:
                train = numpy.vstack((train, [row[0], row[1], row[2], row[3]]))
            
            trainresult = numpy.append(trainresult, [row[4]])

        clf = SVC(gamma='auto', kernel='rbf')
        clf.fit(train, trainresult) 
    
    with open('res/test.csv', 'rb') as csvtestfile:
        
        testreader = csv.reader(csvtestfile, delimiter = ',', quotechar ='|')
        
        for row in testreader:
            
            if test == []:
                test = numpy.append(test, [row[0], row[1], row[2], row[3]])
            else:
                test = numpy.vstack((test, [row[0], row[1], row[2], row[3]]))
            
            testresult = numpy.append(testresult, [row[4]])

        print(clf.score(test, testresult))
        predresult = clf.predict(test)
        
        print confusion_matrix(testresult, predresult)
        
    return clf
    
def load_mlp_classifier(filename):
    
    clf = pickle.load(open(filename, 'rb'))
    
    test = []
    testresult = []
    
    with open('res/test.csv', 'rb') as csvtestfile:
        
        testreader = csv.reader(csvtestfile, delimiter = ',', quotechar ='|')
        
        for row in testreader:
            
            if test == []:
                test = numpy.append(test, [float(row[0]), float(row[1]), float(row[2]), float(row[3])])
            else:
                test = numpy.vstack((test, [float(row[0]), float(row[1]), float(row[2]), float(row[3])]))
            
            testresult = numpy.append(testresult, [row[4]])

        print(clf.score(test, testresult))
        predresult = clf.predict(test)
        
        print confusion_matrix(testresult, predresult)
        
    return clf
    
    

def set_condition(message):
    global order
    global cond1
    global cond2
    
    conditions = message.data
    
    order, cond1, cond2 = conditions.split('-')

def na_description(cur_map, targ):
    
    global new_desc
    new_desc = False
    
    msg = na_desc[cur_map][targ]
    
    targetpose = PoseStamped()        
    targetpose.header.frame_id = str(cur_map + 1) + "_" + str(targ + 1) + "_target"
    targetpose.header.stamp = rospy.Time(0)
    
    vct_val = 90 + random.randint(0,20)
    msg = "\VCT=" + str(vct_val) + "\ " + msg 
    
    look_at(targetpose)
    
    pub_rob_start.publish()
    pub_rob_desc.publish(str(msg) + " - " + str(rospy.Time.now()))
    say_id = textSpchProxy.post.say(msg)
    textSpchProxy.wait(say_id, 0)
    
    pub_rob_end.publish()
    
    trackerProxy.lookAt([1,0,0.6], 0, .1,False)
    
    return rospy.Time.now()
   
def d_description(node_id, decision, location):

    global new_desc
    global d_rel_list
    global cur_map
    global ctx
    global iteration
    global full_desc
    global cur_targ
    global state
    
    #state = "wait"
    
    if cur_map == 0:
        worldName = "map1"
    else:
        worldName = "map2"
    
    if new_desc == True:
        
        new_desc = False
        desc, d_rel_list = dynamic_desc(ctx, worldName, [], node_id, 0, "initial", location, "default", "en_GB", True)
        full_desc = desc
        iteration = 1
        
    #elif decision == "elaborate":
        #if iteration < 4:
            #desc, d_rel_list = dynamic_desc(ctx, worldName, d_rel_list, node_id, iteration, decision, "default", "en_GB", True)
            #iteration = iteration + 1
            #full_desc = full_desc + ', ' + desc
        #else:
            #desc = full_desc
            
    else:
        desc, d_rel_list = dynamic_desc(ctx, worldName, d_rel_list, node_id, iteration, decision, location, "default", "en_GB", True)
    
    #print desc
    
    #state = "d_describe"
    
    return desc
    
def command(message):
    
    global order
    global cond1
    global cond2
    global cur_targ
    global state
    global cur_map
    global new_desc
    global cur_targ_frame
    global prev_state
    
    try:
        textSpchProxy.stopAll()
    except Exception as e:
        print e
    
    if message.data == "tutorial":
        
        trackerProxy.lookAt([1,0,0.6], 0, .1,False)
        trackerProxy.registerTarget("Face",.2)
        trackerProxy.track("Face")
    
        msg = "Hello. "
        msg = msg + "I am Pico, today we are going to play a game together. \pau=500\ "
        msg = msg + "In a moment we are going to see the map of a city. \pau=500\ "
        msg = msg + "However some of the buildings from the city are missing. \pau=500\ "
        msg = msg + "You will see some buildings pop up with a red border, you can move them around by touching and dragging the object on the screen. \pau=500\ "
        msg = msg + "I will be describing where the empty spaces are that these buildings should go in. \pau=500\ "
        msg = msg + "In front of you right now you can see the names I will be using to describe the buildings that need to be moved, and that are already in position. \pau=500\ "
        msg = msg + "A church, a commercial district, a factory, a fire department, a hospital, a manor, a police department, a power plant, and a residence. \pau=500\ "
        msg = msg + "Let's get started!"

        msg = "\RSPD=90\ \VCT=100\ " + msg

        say_id = textSpchProxy.post.say(msg)
        textSpchProxy.wait(say_id, 0)
        
        pub_start_placement1.publish()
        
    elif message.data == "placement1":
        
        cur_map = int(order)
        cur_targ_frame = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
        new_desc = True
    
        if cond1 == "D":
            state = "d_describe"
        else:
            state = "na_describe"
    
    elif message.data == "placement2":
        
        trackerProxy.lookAt([1,0,0.6], 0, .1,False)
        trackerProxy.registerTarget("Face",.2)
        trackerProxy.track("Face")
        
        cur_map = 1 - int(order)
        cur_targ_frame = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
        new_desc = True
            
        if cond2 == "D":
            state = "d_describe"
        else:
            state = "na_describe"
            
    elif message.data == "success":
        
        state = "success"
        
        cur_targ = cur_targ + 1
        cur_targ_frame = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
        
        congrat = ["Well done! ", "Good job! ", "Excellent! "]
        msg = random.choice(congrat)
        
        if cur_targ < 7:
            chatter = [" ", "Let's do the next one.", "Onto the next one.", "Let's keep going.", "Ok, now for the next one"]
            msg = msg + random.choice(chatter)
        
        vct_val = 90 + random.randint(0,20)
        msg = "\VCT=" + str(vct_val) + "\ " + msg 
        
        say_id = textSpchProxy.post.say(msg)
        textSpchProxy.wait(say_id, 0)
        
        if cur_targ < 7:
            state = prev_state
            new_desc = True
        else:
            state = "idle"
            cur_targ = 0
        
        #print cur_targ_frame
        
    elif message.data == "wait":
        
        chatter = ["Good let's bring that one back.", "Nice work, now we need to bring it back to the start point.", "Ok we got it, bring the barrel back to the corner!"]
        prev_state = state
        state = "wait"
    
    elif message.data == "resume":
        
        state = prev_state
    
    else:
        order, _, _ = message.data.split('-')
        if int(order) == 0 or int(order) == 1:
            set_condition(message)
        else:
            textSpchProxy.post.say("I am afraid I can't do that Dave")

if __name__ == "__main__":
    global state
    global ctx
    global target
    global na_desc
    global cur_targ
    global cur_map
    global new_desc
    global cur_targ_frame
    global cur_obj_id
    
    na_desc = [[]]
    new_desc = True
    
    prev_x = 0
    prev_y = 0
    prev_z = 0
    previous_vector = numpy.array([0,0,0])
    
    rospy.init_node("tiago_behaviours");

    last_desc = rospy.Time.now()

    #POSES_TOPIC = rospy.get_param('~poses_output_topic','poses')

    sub_placement_start = rospy.Subscriber("tiago/place_desc/command", String, command, queue_size=1)

    pub_speech = rospy.Publisher("/speech", String, queue_size=5)
    pub_start_placement1 = rospy.Publisher("/sandtray/signals/start_placement1", Empty, queue_size=1)
    pub_rob_start = rospy.Publisher("/sandtray/signals/rob_speech_start", Empty, queue_size=1)
    pub_rob_end = rospy.Publisher("/sandtray/signals/rob_speech_end", Empty, queue_size=1)
    pub_rob_desc = rospy.Publisher("sandtray/signals/rob_speech_desc", String, queue_size=5)

    state = "wait"
    
    clf = load_mlp_classifier("mlp_placement_cur.sav")

    world1 = ctx.worlds["map1"]
    world2 = ctx.worlds["map2"]
    
    ModelLoader().load("res/map_wEmpty.blend", world="map1")
    ModelLoader().load("res/map2_wEmpty.blend", world="map2")
    
    time.sleep(5) # leave some time for the loader to finish
    
    target[0][0] = world1.scene.nodebyname("residence-18")[0].id
    target[0][1] = world1.scene.nodebyname("manor")[0].id
    target[0][2] = world1.scene.nodebyname("residence-7")[0].id
    target[0][3] = world1.scene.nodebyname("commercial_district-1")[0].id
    target[0][4] = world1.scene.nodebyname("police_department")[0].id
    target[0][5] = world1.scene.nodebyname("police_department-1")[0].id
    target[0][6] = world1.scene.nodebyname("church-2")[0].id
    target[1][0] = world2.scene.nodebyname("hospital-1")[0].id
    target[1][1] = world2.scene.nodebyname("hospital-2")[0].id
    target[1][2] = world2.scene.nodebyname("residence-6")[0].id
    target[1][3] = world2.scene.nodebyname("hospital")[0].id
    target[1][4] = world2.scene.nodebyname("residence-17")[0].id
    target[1][5] = world2.scene.nodebyname("manor-1")[0].id
    target[1][6] = world2.scene.nodebyname("power_plant")[0].id
    
    obj_frame_id[0].append("1_1_residential")
    obj_frame_id[0].append("1_2_manor")
    obj_frame_id[0].append("1_3_residential")
    obj_frame_id[0].append("1_4_commercial")
    obj_frame_id[0].append("1_5_police")
    obj_frame_id[0].append("1_6_police")
    obj_frame_id[0].append("1_7_church")
    obj_frame_id.append([])
    obj_frame_id[1].append("2_1_hospital")
    obj_frame_id[1].append("2_2_hospital")
    obj_frame_id[1].append("2_3_residential")
    obj_frame_id[1].append("2_4_hospital")
    obj_frame_id[1].append("2_5_residential")
    obj_frame_id[1].append("2_6_manor")
    obj_frame_id[1].append("2_7_power")
    
    for node in world1.scene.nodes:
        format_name("map1", node.id)
    
    for node in world2.scene.nodes:
        format_name("map2", node.id)
        
    time.sleep(10)
    
    i = 0
    j = 0
    
    while i < 2:
        if i == 0:
            worldname = "map1"
        else:
            worldname = "map2"
            na_desc.append([])
           
        world = ctx.worlds[worldname]
        
        target_chk = target[i]
        
        node_chk = []
        
        for node_id in target_chk:
            node_chk.append(world.scene.nodes[node_id])
        
        while j < 7:
            node_chk.pop(0)
            desc = gen_spatial_desc(ctx, worldname, target[i][j], "default", node_chk,"en_GB","NonAmbig", "placement", True)
            na_desc[i].append(str(desc))
            j = j+1
            
        j = 0
        i = i + 1

    tl = tf.TransformListener()

    r = rospy.Rate(10)
    
    decision_list = []
    
    targ_found = False
    new_desc = False
    prev_desc = ""

    while not rospy.is_shutdown():
                
        if state == "na_describe" and (((rospy.Time.now() - last_desc).to_sec() > 5) or (new_desc == True)):
            last_desc = na_description(cur_map, cur_targ)
        
        if state == "d_describe":
            
            if new_desc == True or targ_found == False:
            
                targ_found = False
                
                prev_x = 0
                prev_y = 0
                prev_z = 0
                previous_vector = numpy.array([0,0,0])
                
                cur_obj_id = target[cur_map][cur_targ]
                
                try:
                    (trans, rot) = tl.lookupTransform(cur_targ_frame, '/sandtray', rospy.Time(0))
                    #Convert for classifier
                    target_x = trans[0] * -15
                    target_y = trans[1] * -15
                    target_z = trans[2] * 15
                    
                    targ_found = True
                    
                except Exception as e:
                    print "exception in targ_frame"
                    print e
                    pass
            
            if targ_found == False:
                continue
            
            try:

                (trans, rot) = tl.lookupTransform(obj_frame_id[cur_map][cur_targ], '/sandtray', rospy.Time(0))
                #Convert for classifier
                cur_x = trans[0] * -15
                cur_y = trans[1] * -15
                cur_z = trans[2] * 15
                
                x_vec = cur_x - prev_x
                y_vec = cur_y - prev_y
                z_vec = cur_z - prev_z
                
                vector = numpy.array([x_vec,y_vec,z_vec])
                mag = numpy.linalg.norm(vector)
                
                x_dist = target_x - cur_x
                y_dist = target_y - cur_y
                z_dist = target_z - cur_z
                
                x_prev_dist = target_x - prev_x
                y_prev_dist = target_y - prev_y
                z_prev_dist = target_z - prev_z
                
                dist_vec = numpy.array([x_dist,y_dist,z_dist])
                distance_to_target = numpy.linalg.norm(dist_vec)
                
                prev_dist_vec = numpy.array([x_prev_dist,y_prev_dist,z_prev_dist])
                prev_distance_to_target = numpy.linalg.norm(prev_dist_vec)
                
                change_in_dist = distance_to_target - prev_distance_to_target
                
                prev_mag = numpy.linalg.norm(previous_vector)
                
                if mag == 0 or prev_mag == 0:
                    angle_from_prev = 0
                else:
                    unit_vector_1 = vector/mag
                    unit_vector_2 = previous_vector/prev_mag
                    angle_from_prev = numpy.arccos(numpy.clip(numpy.dot(unit_vector_1, unit_vector_2), -1.0, 1.0))
                
                prev_x = cur_x
                prev_y = cur_y
                prev_z = cur_z
                
                previous_vector = vector
                
                pred_data = numpy.array([distance_to_target, change_in_dist, mag, angle_from_prev])
                
                shaped_data = pred_data.reshape(1,-1)
                
                decision = clf.predict(shaped_data)
                
                decision_list.append(int(decision[0]))
                
                if len(decision_list) > 5:
                    decision_list.pop(0)
                
                #print decision
            
            except Exception as e:
                print "exception in obj_frame"
                print e
                pass
                
            time_since_last = (rospy.Time.now() - last_desc).to_sec()
            
            if time_since_last > 0.5 or new_desc == True:
                if new_desc == True:
                    final_dec = "initial"
                elif len(decision_list) == 0:
                    final_dec = "elaborate"
                else:
                    #print decision_list
                    avg_decision = float(sum(decision_list))/float(len(decision_list))
                    #print avg_decision
                    if avg_decision < 0.8:
                        final_dec = "negate"
                    elif avg_decision > 1.5:
                        final_dec = "positive"
                    else:
                        final_dec = "elaborate"
                        
                if (final_dec == "initial") or (final_dec  == "negate") or (final_dec == "elaborate" and time_since_last >= 1) or (final_dec == "positive" and time_since_last >= 2):
                    d_desc = d_description(cur_obj_id, final_dec, numpy.array([cur_x, cur_y, cur_z]))
                    
                    if final_dec == "elaborate" and d_desc == prev_desc and time_since_last < 5:
                        pass
                    else:
                        
                        targetpose = PoseStamped()        
                        targetpose.header.frame_id = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
                        targetpose.header.stamp = rospy.Time(0)
    
                        look_at(targetpose)
                        
                        vct_val = 90 + random.randint(0,20)
                        msg = "\VCT=" + str(vct_val) + "\ " + d_desc 
                        
                        pub_rob_start.publish()
                        pub_rob_desc.publish(str(msg) + " - " + str(rospy.Time.now()))
                        say_id = textSpchProxy.post.say(str(msg))
                        textSpchProxy.wait(say_id, 0)
                        
                        pub_rob_end.publish()
                        
                        prev_desc = d_desc
                        
                        trackerProxy.lookAt([1,0,0.6], 0, .1,False)
                        
                        last_desc = rospy.Time.now()
                        
                else:
                    pass
        
            
        elif state == "wait":
            pass
        else:
            pass

        r.sleep()
