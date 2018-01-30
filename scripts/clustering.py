#!/usr/bin/env python
# Copyright 2017 AtsushiSaito

import rospy, rosbag, rosparam, roslib;
import math, sys, random, datetime, csv, shutil, os, string, time, os.path, collections
import numpy as np

from sklearn.preprocessing import StandardScaler
from sklearn import cluster, preprocessing, mixture

from geometry_msgs.msg import Twist
from raspimouse_ros_2.msg import LightSensorValues
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

class outlier_detection():
    def __init__(self,offset=3,repeat=2):
        self.fitdata = []
        self.offset = int(offset)
        self.repeat = repeat

    def getparam(self):
        return {"offset":self.offset,"repeat":self.repeat}

    def negation(self,data):
        label = []
        max_list = []
        maxlabel = 0

        for i in range(len(data)-self.offset*2):
            #Collect data
            #(offset)--(median)--(offset)
            for k in range(1+(self.offset*2)): self.fitdata.append(int(data[k+i]))

            #Copy frequent value
            maxlabel = collections.Counter(self.fitdata).most_common()[0][0]

            #Make up for offset.
            if i == 0:
                for j in range(self.offset+1):label.append(maxlabel)
            else: label.append(maxlabel)

            self.fitdata = []
        #Make up for offset(finished).
        for i in range(self.offset):label.append(maxlabel)
        return label

    def fit(self,data):
        for i in range(self.repeat):data = self.negation(data)
        return data

class Clustering():
    def __init__(self):
        self.s = rospy.Service('clustering_request', Empty, self.handle_service)
        self.on = False
        self.bag_open = False

    def handle_service(self,req):
        self.fit()
        return EmptyResponse()

    def checkBagopen(self):
        if self.bag_open:
            self.bag.close()
            self.bag_open = False
        return True

    def openBagfile(self):
        self.bagfile_path = rospy.get_param("/bag_filelink", rospy.get_param("/current_bag_file",""))
        self.bag = rosbag.Bag(self.bagfile_path)
        self.bagname, self.bagpath = os.path.splitext(self.bagfile_path)
        print ("----------------------")
        print ("BagFile: [%s]" % self.bagname)
        self.bag_open = True

    def readBagfile(self):
        self.right_forward = []
        self.right_side = []
        self.left_side = []
        self.left_forward = []
        self.linear_x = []
        self.angular_z = []
        for topic, msg, t in self.bag.read_messages(topics=['/event']):
            self.right_forward.append(float(msg.right_forward))
            self.right_side.append(float(msg.right_side))
            self.left_side.append(float(msg.left_side))
            self.left_forward.append(float(msg.left_forward))
            self.linear_x.append(msg.linear_x)
            self.angular_z.append(msg.angular_z)

    def preprocessingValue(self, data):
        SS = StandardScaler().fit(data)
        SSData = SS.transform(data)
        return SSData

    def resultWrite(self):
        predict_path = "/home/ubuntu/.ros" + '/' + string.replace(self.bagname, '/', '_slash_') + '_Predict' + '.txt'
        predict_proba_path = "/home/ubuntu/.ros" + '/' + string.replace(self.bagname, '/', '_slash_') + '_Predict_Proba' + '.txt'
        np.savetxt(predict_path, self.predict, delimiter=',')
        np.savetxt(predict_proba_path, self.predict_proba, delimiter=',', fmt="%.8f", header=predict_proba_path)

    def fit(self):
        if self.checkBagopen():
            self.openBagfile()
            self.readBagfile()

            self.action_values = np.c_[self.linear_x, self.angular_z]
            self.sensor_values = np.c_[self.left_forward, self.left_side, self.right_side, self.right_forward]

            self.data = self.preprocessingValue(self.sensor_values)

            rospy.loginfo("Clustering Start")
            VBGMM = mixture.BayesianGaussianMixture(n_components=10,max_iter=200,random_state=10).fit(self.data)
            self.predict = VBGMM.predict(self.data)
            self.predict_proba = VBGMM.predict_proba(self.data)

            OD = outlier_detection(offset = 5,repeat=2)
            self.od_predict = OD.fit(self.predict)

            self.resultWrite()

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Clustering_Server')
    Clustering().run()
