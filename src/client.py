import zmq
import zmq.utils.jsonapi as zmqJson

#zmq context need to be global
context = zmq.Context()

class LeapMotionPoller():

    sub_channel="0 "
    def __init__(self):
        self.zmq = context
        self.socket = self.zmq.socket(zmq.SUB)
        self.socket.connect("tcp://192.168.8.103:5556")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "0")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

    def Poll(self):
        polledSockets=dict(self.poller.poll(timeout=500))
        #print(polledSockets)
        try:
            if self.socket in polledSockets and polledSockets[self.socket]==zmq.POLLIN:
                jsTxt = self.socket.recv_string(zmq.DONTWAIT)
                #print(jsTxt)
                jsObj=zmqJson.loads(jsTxt.lstrip(LeapMotionPoller.sub_channel))
                print(jsObj["handType"])
                return jsObj
        except:
            return None
            pass
        return None

import sys, os
sys.path.append(os.path.join('..'))
from uarm.wrapper import SwiftAPI
from uarm.swift.protocol import SERVO_BOTTOM, SERVO_LEFT, SERVO_RIGHT, SERVO_HAND
class UarmActuator():

    X_MIN, X_MAX=150,220
    Y_MIN, Y_MAX=-155,155
    Z_MIN, Z_MAX=22,150

    def __init__(self):
        try:
            self.uArm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
            self.uArm.waiting_ready()
            self.uArm.reset()

            self.jsonTrack=[]
        except:
            pass

    def connect(self):
        try:
            self.uArm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
            self.uArm.waiting_ready()
            self.uArm.reset()
        except:
            pass

    def disconnect(self):

        try:
            self.uArm.set_position(x=150,y=0,z=22, speed=1000000)
            self.uArm.set_servo_detach()
            self.uArm.disconnect()
        except:
            pass
        finally:
            pass

    def register_callback(self,period):
        self.uArm.register_report_position_callback(callback=self.pos_callback)
        self.uArm.set_report_position(interval=period)

    def deregister_callback(self):
        self.uArm.set_report_position(0)

    def pos_callback(self,ret):
        print('report pos: {}, time: {}'.format(ret, time.time()))
        #TODO check json

    def onLeapMotionUpdate(self,aJson):
        if aJson is not None:
            print("[onLeapMotionUpdate] json: %s" % aJson)
            self.jsonTrack.append(resultJson)
            if len(self.jsonTrack) > 100
                self.jsonTrack.remove(self.actuator.jsonTrack[0])
        return

    def mapper(self):
        #clipping the input
        #using relative unit span for comparison
        #if distance is large than 0.05, the drive the motor to the target point
        #leapmotion: (position_x,position_y,position_z) ~ uarm: (^y,z,^x)
        #leapmotion: clip([-150,150],[80,250],[-50,150])
        return

shouldRun=True

def PollSocket(poller,actuator):
    while shouldRun:
        resultJson=poller.Poll()
        if resultJson is not None:
            actuator.onLeapMotionUpdate(resultJson)

import threading

def main():

    poller = LeapMotionPoller()
    actuator = UarmActuator()
    actuator.connect()
    runner = threading.Thread(target = PollSocket, args=(poller,actuator))
    runner.start()

    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        global shouldRun
        sys.stdin.readline()
        shouldRun = False
    except KeyboardInterrupt:
        pass
    finally:
        runner.join()
        print("Client disconnecting...")
        actuator.disconnect()
        poller.socket.disconnect("tcp://192.168.8.103:5556")
        context.destroy()

    return

if __name__ == "__main__":
    main()
