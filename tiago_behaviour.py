#!/usr/bin/env python

from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_msgs.msg import Empty
from time import sleep
from datetime import datetime
from operator import itemgetter 
import rospy
import tf
#import motion
import numpy
import math
import random
import time
import sys
import os
import pyttsx
import csv

import underworlds
import underworlds.server
from underworlds.tools.loader import ModelLoader
from underworlds.tools.spatial_relations import *
from underworlds.tools.edit import *
from underworlds.helpers.geometry import get_world_transform
from underworlds.helpers.transformations import *

import csv
from sklearn.svm import SVC
from sklearn.metrics import confusion_matrix
from sklearn.neural_network import MLPClassifier
import pickle

from placement_description import *

state = "idle"
prev_state = "idle"

ctx = underworlds.Context("Tiago Description")

target = numpy.empty((2,6), dtype="a36")
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
logpath = ""

tts = pyttsx.init()

#tts.say('Good morning.')
#tts.runAndWait()
#tts.stop() - Might allow an interrupt

target_change = False

def say(msg):
    
    global tts
    
    tts = pyttsx.init()
    tts.setProperty('rate', 125)
    tts.say(msg)
    a = tts.runAndWait()
    
def load_mlp_classifier(filename):
    
    clf = pickle.load(open(filename, 'rb'))
    
    test = []
    testresult = []
    
    with open('res/test.csv', 'rb') as csvtestfile:
        
        testreader = csv.reader(csvtestfile, delimiter = ',', quotechar ='|')
        
        for row in testreader:
            
            if test == []:
                test = numpy.append(test, [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])])
            else:
                test = numpy.vstack((test, [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])]))
            
            testresult = numpy.append(testresult, [row[6]])

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
    
    #targetpose = PoseStamped()        
    #targetpose.header.frame_id = str(cur_map + 1) + "_" + str(targ + 1) + "_target"
    #targetpose.header.stamp = rospy.Time(0)
    
    #vct_val = 90 + random.randint(0,20)
    #msg = "\VCT=" + str(vct_val) + "\ " + msg 
    
    #look_at(targetpose)
    
    #pub_rob_start.publish()
    #pub_rob_desc.publish(str(msg) + " - " + str(rospy.Time.now()))
    #say_id = textSpchProxy.post.say(msg)
    write_log("Tiago Starts Speaking")
    write_log(msg)
    say(msg)
    write_log("Tiago Stops Speaking")
    
    #pub_rob_end.publish()
    
    #trackerProxy.lookAt([1,0,0.6], 0, .1,False)
    
    return rospy.Time.now()
   
def d_description(node_id, decision):

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
        desc, d_rel_list = dynamic_desc(ctx, worldName, [], node_id, 0, "initial", "default", "en_GB", False, True)
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
        desc, d_rel_list = dynamic_desc(ctx, worldName, d_rel_list, node_id, iteration, decision, "default", "en_GB", False, True, 0, "nearby")
    
    #print desc
    
    #state = "d_describe"
    
    return desc

def create_log():
    
    global logpath
    global start_log_time
    
    if not os.path.exists('log'):
        os.makedirs('log')
        
    logpath = os.path.join('log',str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S')))
    os.makedirs(logpath)
    
    start_log_time = rospy.Time.now()
    
    write_log('Start')

def write_log(log):

    global logpath
    global start_log_time

    csvpath = os.path.join(logpath, 'log.csv')
    with open(csvpath, 'a') as csvfile:
        logwriter = csv.writer(csvfile)
        logwriter.writerow([rospy.Time.now() - start_log_time, log])
        
def write_position_log(x, y, yaw, prev_x, prev_y, prev_yaw, target_x, target_y):

    global logpath
    global start_log_time
    
    csvpath = os.path.join(logpath, 'position_log.csv')
    with open(csvpath, 'a') as csvfile:
        logwriter = csv.writer(csvfile)
        logwriter.writerow([rospy.Time.now() - start_log_time, x, y, yaw, prev_x, prev_y, prev_yaw, target_x, target_y])
    
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
    global tts
    
    if message.data == "tutorial":
        
        state = "wait"
        if cond1 != "R":
        
            msg = "Hello. "
            msg = msg + "I am Tiago, today we have an important job. "
            msg = msg + "You should be able to see me in a room with some barrels through the security cameras. "
            msg = msg + "Some of these barrels are emmitting different types of radiation. "
            msg = msg + "We need to sort them for safe disposal. I am able to identify the radioactive barrels. "
            msg = msg + "But I will need you to guide me to the locations I describe using the arrow keys on your keyboard. "
            msg = msg + "Once I am in position you can command me to move my arm into the grab position, and then activate the electromagnet to pick up the barrel. "
            msg = msg + "We will then need to bring it back to the starting point for later disposal. "
            msg = msg + "Let's get started!"

            #msg = "\RSPD=90\ \VCT=100\ " + msg

            say(msg)
        
        create_log()
        
        #pub_start_placement1.publish()
        
    #elif message.data == "placement1":
        
        write_log("Condition: %s-%s" % (cond1, cond2))
        write_log("Start First Map")
        
        cur_map = int(order)
        cur_targ = 0
        cur_targ_frame = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
        new_desc = True
    
        if cond1 == "D":
            state = "d_describe"
        elif cond1 == "N":
            state = "na_describe"
        else:
            state = "record"
    
    elif message.data == "placement2":
        
        #trackerProxy.lookAt([1,0,0.6], 0, .1,False)
        #trackerProxy.registerTarget("Face",.2)
        #trackerProxy.track("Face")
        
        write_log("Start Second Map")
        
        cur_map = 1 - int(order)
        cur_targ = 0
        cur_targ_frame = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
        new_desc = True
            
        if cond2 == "D":
            state = "d_describe"
        elif cond2 == "N":
            state = "na_describe"
        else:
            state = "record"
            
    elif message.data == "success":
        
        try:
            tts.stop()
        except Exception as e:
            print e
        
        write_log("Item placed " + str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target")
        state = "wait"
        
        cur_targ = cur_targ + 1
        cur_targ_frame = str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target"
        
        congrat = ["Well done! ", "Good job! ", "Excellent! "]
        msg = random.choice(congrat)
        
        if cur_targ < 6:
            chatter = [" ", "Let's do the next one.", "Onto the next one.", "Let's keep going.", "Ok, now for the next one"]
            msg = msg + random.choice(chatter)
        
        #vct_val = 90 + random.randint(0,20)
        #msg = "\VCT=" + str(vct_val) + "\ " + msg 
        
        if cond1 != "R":
            say(msg)
        
        if cur_targ < 6:
            state = prev_state
            new_desc = True
        else:
            state = "wait"
            cur_targ = 0
        
        #print cur_targ_frame
        
    elif message.data == "wait":
        
        try:
            tts.stop()
        except Exception as e:
            print e
        
        write_log("Item picked up " + str(cur_map + 1) + "_" + str(cur_targ + 1) + "_target")
        chatter = ["Good let's bring that one back.", "Nice work, now we need to bring it back to the start point.", "Ok we got it, bring the barrel back to the corner!"]
        prev_state = state
        state = "wait"
    
    elif message.data == "resume":
        
        write_log("Returnin to description")
        state = prev_state
        
    elif message.data == "grab":
        
        write_log("Grab pose initiated")
        
    elif message.data == "leave":
        
        write_log("Leaving grab pose")
        
    
    else:
        order, _, _ = message.data.split('-')
        if int(order) == 0 or int(order) == 1:
            set_condition(message)
        else:
            say("I am afraid I can't do that Dave")

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
    prev_yaw = 0
    previous_vector = numpy.array([0,0,0])
    
    req_yaw = 0
    
    rospy.init_node("tiago_behaviours");

    last_desc = rospy.Time.now()

    #POSES_TOPIC = rospy.get_param('~poses_output_topic','poses')

    sub_placement_start = rospy.Subscriber("tiago/place_desc/command", String, command, queue_size=1)

    #pub_speech = rospy.Publisher("/speech", String, queue_size=5)
    #pub_start_placement1 = rospy.Publisher("/sandtray/signals/start_placement1", Empty, queue_size=1)
    #pub_rob_start = rospy.Publisher("/sandtray/signals/rob_speech_start", Empty, queue_size=1)
    #pub_rob_end = rospy.Publisher("/sandtray/signals/rob_speech_end", Empty, queue_size=1)
    #pub_rob_desc = rospy.Publisher("sandtray/signals/rob_speech_desc", String, queue_size=5)

    state = "wait"
    
    clf = load_mlp_classifier("mlp_placement_cur.sav")

    world = []
    
    ModelLoader().load("res/map1.blend", world="map1")
    ModelLoader().load("res/map2.blend", world="map2")
    
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
    
    grab_pos_id = []
    grab_pos_id.append(world[0].scene.nodebyname("_grab_pos")[0].id)
    grab_pos_id.append(world[1].scene.nodebyname("_grab_pos")[0].id)
    
    base_pos_id = []
    base_pos_id.append(world[0].scene.nodebyname("_base_pos")[0].id)
    base_pos_id.append(world[1].scene.nodebyname("_base_pos")[0].id)
    
    rad_marker_id = []
    rad_marker_id.append(world[0].scene.nodebyname("_rad_marker")[0].id)
    rad_marker_id.append(world[1].scene.nodebyname("_rad_marker")[0].id)
    
    #obj_frame_id[0].append("1_1_residential")
    #obj_frame_id[0].append("1_2_manor")
    #obj_frame_id[0].append("1_3_residential")
    #obj_frame_id[0].append("1_4_commercial")
    #obj_frame_id[0].append("1_5_police")
    #obj_frame_id[0].append("1_6_police")
    #obj_frame_id[0].append("1_7_church")
    #obj_frame_id.append([])
    #obj_frame_id[1].append("2_1_hospital")
    #obj_frame_id[1].append("2_2_hospital")
    #obj_frame_id[1].append("2_3_residential")
    #obj_frame_id[1].append("2_4_hospital")
    #obj_frame_id[1].append("2_5_residential")
    #obj_frame_id[1].append("2_6_manor")
    #obj_frame_id[1].append("2_7_power")
    
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
        
    time.sleep(10)
    
    i = 0
    j = 0
    
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

    tl = tf.TransformListener()

    r = rospy.Rate(20)
    
    decision_list = []
    
    targ_found = False
    new_desc = False
    prev_desc = ""

    rad_scale, rad_shear, rad_angles, _, rad_persp = decompose_matrix(get_world_transform(world[cur_map].scene, world[cur_map].scene.nodes[rad_marker_id[cur_map]]))
    base_scale, base_shear, _, _, base_persp = decompose_matrix(get_world_transform(world[cur_map].scene, world[cur_map].scene.nodes[base_pos_id[cur_map]]))
    grab_scale, grab_shear, _, _, grab_persp = decompose_matrix(get_world_transform(world[cur_map].scene, world[cur_map].scene.nodes[grab_pos_id[cur_map]]))

    print("Boot complete, awaiting commands")

    while not rospy.is_shutdown():
        
        msg_add = ''        
        
        if state != "wait":
            
            if new_desc == True or targ_found == False:
            
                targ_found = False
                
                _, _, _, translate, _ = tf.transformations.decompose_matrix(get_world_transform(world[cur_map].scene, world[cur_map].scene.nodes[target[cur_map][cur_targ]]))
                
                target_x, target_y, target_z = translate
                
                target_z = 0 #We don't actually care about z in this case and it will throw off the numbers.
                
                if cond1 == 'R':
                    marker_z = 0.205
                else:
                    marker_z = -100
                
                world[cur_map].scene.nodes[rad_marker_id[cur_map]].transformation = compose_matrix(scale = rad_scale, shear = rad_shear, angles = rad_angles, translate = [target_x, target_y, marker_z], perspective = rad_persp)
                
                
                world[cur_map].scene.nodes.update(world[cur_map].scene.nodes[rad_marker_id[cur_map]])
                
                targ_found = True
                
                #prev_x = 0
                #prev_y = 0
                #prev_z = 0
                #previous_vector = numpy.array([0,0,0])
                
                #cur_obj_id = target[cur_map][cur_targ]
                
                #try:
                    #(trans, rot) = tl.lookupTransform(cur_targ_frame, '/sandtray', rospy.Time(0))
                    
                    #target_x = trans[0]
                    #target_y = trans[1]
                    #target_z = trans[2]
                    
                    #targ_found = True
                    
                #except Exception as e:
                    #print "exception in targ_frame"
                    #print e
                    #pass
            
            #if targ_found == False:
                #continue
            
            try:

                (trans, rot) = tl.lookupTransform('map', 'grab_pos', rospy.Time(0))

                cur_roll, cur_pitch, cur_yaw = euler_from_quaternion(rot)

                cur_x = trans[0]
                cur_y = trans[1]
                cur_z = trans[2]
                
                cur_z = 0 #We don't actually care about z in this case and it will throw off the numbers.
                
                world[cur_map].scene.nodes[grab_pos_id[cur_map]].transformation = compose_matrix(scale = grab_scale, shear = grab_shear, angles = [cur_roll, cur_pitch, cur_yaw], translate = [cur_x, cur_y, 0.25], perspective = grab_persp)
                world[cur_map].scene.nodes.update(world[cur_map].scene.nodes[grab_pos_id[cur_map]])
                
                (trans, rot) = tl.lookupTransform('map', 'base_footprint', rospy.Time(0))

                base_roll, base_pitch, base_yaw = euler_from_quaternion(rot)

                base_x = trans[0]
                base_y = trans[1]
                base_z = trans[2]
                
                world[cur_map].scene.nodes[base_pos_id[cur_map]].transformation = compose_matrix(scale = base_scale, shear = base_shear, angles = [base_roll, base_pitch, base_yaw], translate = [base_x, base_y, base_z], perspective = base_persp)
                world[cur_map].scene.nodes.update(world[cur_map].scene.nodes[base_pos_id[cur_map]])
                
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
                
                #if mag == 0 or prev_mag == 0:
                #    angle_from_prev = 0
                #else:
                #    unit_vector_1 = vector/mag
                #    unit_vector_2 = previous_vector/prev_mag
                #    angle_from_prev = numpy.arccos(numpy.clip(numpy.dot(unit_vector_1, unit_vector_2), -1.0, 1.0))
                
                req_yaw =  math.atan2(target_y - base_y, target_x - base_x)
                change_in_yaw = base_yaw - prev_yaw
                
                if req_yaw - base_yaw < -(math.pi):
                    req_change_yaw = req_yaw - base_yaw + (2*math.pi)
                elif req_yaw - base_yaw > math.pi:
                    req_change_yaw = req_yaw - base_yaw - (2*math.pi)
                else:
                    req_change_yaw = req_yaw - base_yaw
                
                write_position_log(cur_x, cur_y, cur_yaw, prev_x, prev_y, prev_yaw, target_x, target_y)
                
                prev_x = cur_x
                prev_y = cur_y
                prev_z = cur_z
                
                prev_yaw = base_yaw
                
                previous_vector = vector
                
                pred_data = numpy.array([distance_to_target, change_in_dist, mag, change_in_yaw, req_yaw, req_change_yaw])
                
                shaped_data = pred_data.reshape(1,-1)
                
                decision = clf.predict(shaped_data)
                
                decision_list.append(int(decision[0]))
                
                if len(decision_list) > 20:
                    decision_list.pop(0)
                
                #print decision
            
            except Exception as e:
                print "exception in obj_frame"
                print e
                pass

            
            if state == "na_describe" and (((rospy.Time.now() - last_desc).to_sec() > 5) or (new_desc == True)):
                last_desc = na_description(cur_map, cur_targ)
            
            if state == "d_describe":
                
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
                        elif avg_decision > 1.6 and avg_decision < 2.6:
                            final_dec = "positive"
                        elif avg_decision >= 2.6:
                            final_dec = "elaborate"
                        else:
                            final_dec = "navigate"
                            
                    if (final_dec == "initial") or (final_dec  == "negate") or ((final_dec == "elaborate" or final_dec == "navigate") and time_since_last >= 1) or (final_dec == "positive" and time_since_last >= 2):
                        
                        if final_dec == "navigate":
                            if req_change_yaw > 0.2:
                                d_desc = "turn left"
                            elif req_change_yaw < -0.2:
                                d_desc = "turn right"
                            else:
                                #check we haven't got the target between the grab position and the base.
                                if abs(math.atan2(target_y - cur_y, target_x - cur_x)) > math.pi/2:
                                    d_desc = "go backward"
                                else:
                                    d_desc = "go forward"
                            
                            if abs(req_change_yaw) > 2.749:
                                msg_add = " about 180 degrees"
                            elif abs(req_change_yaw) > 2.051:
                                msg_add = " about 135 degrees"
                            elif abs(req_change_yaw) > 1.178
                                msg_add = " about 90 degrees"
                                
                        else:
                            d_desc = d_description(target[cur_map][cur_targ], final_dec)
                        
                        if (final_dec == "elaborate" or final_dec == "navigate") and d_desc == prev_desc and time_since_last < 5:
                            pass
                        else:
                            
                            msg = d_desc + msg_add
                            
                            #pub_rob_desc.publish(str(msg) + " - " + str(rospy.Time.now()))
                            write_log("Tiago Starts Speaking")
                            write_log(msg)
                            say(msg)
                            write_log("Tiago Stops Speaking")
                            
                            #pub_rob_end.publish()
                            
                            prev_desc = d_desc
                            
                            #trackerProxy.lookAt([1,0,0.6], 0, .1,False)
                            
                            last_desc = rospy.Time.now()
                            
                    else:
                        pass
            
        else:
            pass

        r.sleep()
