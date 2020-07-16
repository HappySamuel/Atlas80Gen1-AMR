#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the encoders phidget. Publishes EncoderCounts
messages on the "encoder_counts" topic.

Parameters:
- period: publish only after this period has elapsed (default value: 0.02).
  This limits the publishing rate. It seems the phidget publishes a count
  change every 8ms.
- min_pub_period: if set, we want to publish messages with this period, even if
  encoders are not changing. Otherwise, publish only when the encoders value is
  changing.
'''

import roslib; roslib.load_manifest('phidget_encoders')
import rospy
import threading

from Phidgets.PhidgetException import *
from Phidgets.Events.Events import *
from Phidgets.Devices.Encoder import Encoder

from phidget_encoders.msg import EncoderCounts
import diagnostic_updater as DIAG


def err(e):
    '''A helper function to report Phidget errors'''
    rospy.logerr("Phidget error %i: %s. Exiting..." % (e.code, e.details))
    exit(1)


class CountBuffer:
#resolution*4
    def __init__(self, counts=800):
        self.counts = counts
        self.total = 0
        self.reset()

    def reset(self):
        self.dt = 0.0 #time (sec)
        self.d_count = 0
        self.n = 0

    def add(self, e):
        self.n += 1
        self.d_count += e.positionChange
        self.total += e.positionChange
        self.dt += e.time * 1e-6 #usec to sec

    def get_delta_rev(self):
        return float(self.d_count)/self.counts
        
    def get_total_count(self):
        return self.total

class PhidgetEncoder:
    '''Monitor the pose of each encoders'''

    def __init__(self):
        self.period = rospy.get_param('~period', 0.02)

        self.minPubPeriod = rospy.get_param('~min_pub_period', None)
        if self.minPubPeriod < self.period * 2:
            rospy.loginfo("Warning: you attempted to set the min_pub_period" +
                " to a smaller value (%.3f)" % (self.minPubPeriod) +
                " than twice the period (2 * %.3f)." % (self.period) +
                " This is not allowed, hence I am setting the min_pub_period" +
                " to %.3f." % (self.period * 2) )
            self.minPubPeriod = self.period * 2

        # encoders are identified by their index (i.e. on which port they are
        # plugged on the phidget)
        self.left = 1
        self.right = 0
        self.countBufs = {self.left: CountBuffer(), self.right: CountBuffer()}
        self.lastPub = None

        self._mutex = threading.RLock()

        self.initPhidget()
        self.encoder.setEnabled(self.left, True)
        self.encoder.setEnabled(self.right, True)

        self.pub = rospy.Publisher('encoder_counts', EncoderCounts)

        # diagnostics
        self.diag_updater = DIAG.Updater()
        self.diag_updater.setHardwareID('none')
        f = {'min': 1.0/self.minPubPeriod}
        fs_params = DIAG.FrequencyStatusParam(f, 0.25, 1)
        self.pub_diag = DIAG.HeaderlessTopicDiagnostic('encoder_counts',
                            self.diag_updater, fs_params)

        #TODO: add a diagnostic task to monitor the connection with the phidget

        self.forcePub(None)


    def initPhidget(self):
        '''Connects to the phidget and init communication.'''

        #Create an encoder object (from the Phidget library)
        try:
            self.encoder = Encoder()
        except RuntimeError as e:
            rospy.logerr("Runtime exception: %s. Exiting..." % e.details)
            exit(1)

        #Set the callback
        try:
            self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)
        except PhidgetException as e:
            err(e)


        rospy.loginfo("Opening phidget object....")
        try:
            self.encoder.openPhidget()
        except PhidgetException as e:
            err(e)

        rospy.loginfo("Waiting for attach....")
        try:
            self.encoder.waitForAttach(10000)
        except PhidgetException as e:
            rospy.logerr("Phidget error %i: %s" % (e.code, e.details))
            try:
                self.encoder.closePhidget()
            except PhidgetException as e:
                err(e)
            exit(1)


    def closePhidget(self):
        rospy.loginfo("Closing...")
        try:
            self.encoder.closePhidget()
        except PhidgetException as e:
            err(e)


    def encoderPositionChange(self, e):
        '''A callback function called whenever the position changed'''

        #rospy.loginfo("Encoder %i: Encoder %i -- Change: %i -- Time: %i -- Position: %i"
              #% ( e.device.getSerialNum(), e.index, e.positionChange, e.time,
                    #self.encoder.getPosition(e.index)) )

        with self._mutex:
            if e.index in self.countBufs.keys():
                self.countBufs[e.index].add(e)
                dts = [b.dt for b in self.countBufs.values()]
                if min(dts) >= self.period:
                    #if self.countBufs[self.left].n != self.countBufs[self.right].n:
                        #rospy.loginfo("encoders: time criteria met, but not count criteria")
                    dt = sum(dts)/len(dts)
                    #rospy.loginfo("Publishing")
                    self.publish(dt)


    def publish(self, dt):
        encodersMsg = EncoderCounts()
        self.lastPub = rospy.Time.now()
        encodersMsg.stamp = self.lastPub # i.e. now()
        encodersMsg.dt = dt

        with self._mutex:
            encodersMsg.d_left = self.countBufs[self.left].get_delta_rev()
            encodersMsg.d_right = self.countBufs[self.right].get_delta_rev()
            encodersMsg.left_count = self.countBufs[self.left].get_total_count()
            encodersMsg.right_count = self.countBufs[self.right].get_total_count()
            self.countBufs[self.left].reset()
            self.countBufs[self.right].reset()

        self.pub.publish(encodersMsg)
        self.pub_diag.tick()
        self.diag_updater.update()


    def forcePub(self, dummy):
        ''' We want to publish messages even if the vehicle is not moving. '''
        if self.minPubPeriod:
            if self.lastPub is None:
                self.lastPub = rospy.Time.now()
            else:
                dt = (rospy.Time.now()-self.lastPub).to_sec()
                if dt > self.minPubPeriod:
                    self.publish(dt)
            self.timer = rospy.Timer(rospy.Duration(self.minPubPeriod), self.forcePub, True)





if __name__=='__main__':
    rospy.init_node('encoders_node', log_level=rospy.DEBUG)
    enc = PhidgetEncoder()
    rospy.loginfo('encoders: Spinning...')
    rospy.spin()
    enc.closePhidget()
    rospy.loginfo("Done.")
    exit(0)
