import zmq
import zmq.utils.jsonapi as zmqJson

#zmq context need to be global
context = zmq.Context()

class LeapMotionPoller():

    sub_channel="0 "
    def __init__(self):
        self.zmq = context
        self.socket = self.zmq.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5556")
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
                #print(jsObj["handType"])
                return jsObj
        except:
            return None
            pass
        return None

import sys, os
import threading
sys.path.append(os.path.join('..'))
from uarm.wrapper import SwiftAPI
from uarm.swift.protocol import SERVO_BOTTOM, SERVO_LEFT, SERVO_RIGHT, SERVO_HAND
class UarmActuator():

    uArm_MIN_XYZ=[150,-155,22]
    uArm_MAX_XYZ=[220,155,150]
    lp_MIN_XYZ=[-150,80,-50]
    lp_MAX_XYZ=[150,250,150]

    def __init__(self):
        try:
            self.jsonTrack=[]
            self.posTrack=[]
            self.cur_hand_type=None
            self.cur_hand_id=None

            self.worker=None
            #working(bool): tell the worker thread alive
            self.working=False
            self.actPosTrack =[]
            #isMoving(bool): read by client, set by robot arm
            self.isMoving=False
            self.isSuction=False
            self.shouldSuction=False
        except:
            print("[Error] UarmActuator failed to init, check Power and connection")
            raise

    def connect(self):
        try:
            self.uArm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
            self.uArm.waiting_ready()
            self.uArm.reset(wait=True,speed=1000000)
            self.uArm.waiting_ready()
            #if setting the period=0.02 would cost the client slow
            self.register_callback(period=0.1)
        except:
            pass

    def disconnect(self):

        try:
            self.working=False
            self.worker.join()

        except Exception as e:
            print("[Error] disconnect error: {}".format(e))
            pass
        finally:
            self.uArm.set_pump(on=False)
            self.deregister_callback()
            self.uArm.set_position(x=150,y=0,z=22, speed=1000000)
            #kludge to fix the async cmd, may consider to use the set_position callback
            import time
            time.sleep(2)
            self.uArm.set_servo_detach()
            self.uArm.disconnect()
            pass

    def register_callback(self,period):
        self.uArm.register_report_position_callback(callback=self.pos_callback)
        self.uArm.set_report_position(interval=period)

    def deregister_callback(self):
        self.uArm.set_report_position(interval=0)
        self.uArm.release_report_position_callback()

    def pos_callback(self,ret):
        uarm_unclamped_coor = ret[:3]
        uarm_clampped_coor = UarmActuator.XYZclamping(uarm_unclamped_coor,UarmActuator.uArm_MIN_XYZ,UarmActuator.uArm_MAX_XYZ)
        #print('uarm_unclamped_coor: {}, uarm_clampped_coor: {}'.format(uarm_unclamped_coor,uarm_clampped_coor))
        uarm_cur_pos=UarmActuator.toRelativePos(uarm_clampped_coor,UarmActuator.uArm_MIN_XYZ,UarmActuator.uArm_MAX_XYZ)
        #print ("uarm_cur_pos: "+str(["{0:0.2f}".format(i) for i in uarm_cur_pos]))

        posTrackLength=len(self.posTrack)
        if posTrackLength > 0 and not self.isMoving:
            #if not self.uArm.get_is_moving():
            try:
                lastPosFromLP = self.posTrack[-1]
                #print ("uarm_cur_pos: "+str(["{0:0.2f}".format(i) for i in uarm_cur_pos]))
                #print ("lastPosFromLP: "+str(["{0:0.2f}".format(i) for i in lastPosFromLP]))
                dist = UarmActuator.dist(uarm_cur_pos,lastPosFromLP)
                #print("dist 1: {}".format(dist))
                if dist >= 0.1:
                    #tmp
                    self.timeoutSet= int(dist*10)
                    #print("timeout: {}".format(self.timeoutSet))
                    #TODO here can have better algo for smoothier track
                    self.actPosTrack.append(lastPosFromLP)
                    self.posTrack.clear()

            except Exception as e:
                print("[Error] e: {}".format(e))

    #def pos_callback(ret):
    #    print('report pos: {}'.format(ret))

    def onLeapMotionUpdate(self,aJson):
        #TODO if a new hand get in, ignore it
        #TODO if no LP input a while, detach motor and wait for in to attach again
        if aJson is not None:
            #print("[onLeapMotionUpdate] json: %s" % aJson)

            #TODO something to do for hand keep tracking
            #self.cur_hand_type=aJson["handType"]
            #self.cur_hand_id=int(aJson["id"])

            #dealing with suctioning
            suction_val = aJson["grab_strength"]
            if suction_val > 0.8:
                self.shouldSuction = True
            else:
                self.shouldSuction = False
            #print("[onLeapMotionUpdate] self.isSuction: {}, self.shouldSuction:{}".format(self.isSuction,self.shouldSuction))

            if self.isSuction != self.shouldSuction:
                self.uArm.set_pump(on=self.shouldSuction)
                self.isSuction = self.shouldSuction
                #print("suction set, self.shouldSuction: {}".format(self.shouldSuction))
            
            if len(self.jsonTrack)>0:
                #do distance checking, filter out dist<0.05 in relative scale: (0-1,0-1,0-1)
                last_json=self.jsonTrack[-1]

                last_rel_pos=UarmActuator.extractLPposFromJson(last_json)
                new_rel_pos=UarmActuator.extractLPposFromJson(aJson)

                dist = UarmActuator.dist(last_rel_pos,new_rel_pos)
                #print("dist 2: {}".format(dist))

                if dist>0.05:
                    self.jsonTrack.append(aJson)
                    if len(self.jsonTrack) > 100:
                        self.jsonTrack.pop(0)
                else:
                    return
            else:
                self.jsonTrack.append(aJson)

        #start from here the hand tracking done
        try:

            if len(self.jsonTrack) is 0:
                return

            cur_json = self.jsonTrack[-1]
            
            cur_target_pos=UarmActuator.extractLPposFromJson(cur_json)
            #remap the directions for the uArm
            buffer_vec = cur_target_pos
            cur_target_pos=[1.0-buffer_vec[2],1.0-buffer_vec[0],buffer_vec[1]]
            #print (["{0:0.2f}".format(i) for i in cur_target_pos])
            self.posTrack.append(cur_target_pos)


            #print("uarm_pos for uarm: " + str(["{0:0.2f}".format(i) for i in uarm_pos]))

            if self.working is False:
                self.working=True
                self.worker = threading.Thread(target=self.workingFunc)
                self.worker.start()

        except Exception as e:
            print("[Error] some exception for jsonTrack?: {}".format(e))

        return

    def workingFunc(self):
        while self.working:
            try:
                #if len(self.posTrack)>0:
                    #xyz=self.posTrack.pop(0)
                    #self.uArm.set_position(x=xyz[0],y=xyz[1],z=xyz[2])
                if len(self.actPosTrack) > 0:
                    self.isMoving=True

                    for pos in self.actPosTrack:
                        uarm_pos=[ a+(b-a)*c for a,b,c in zip(UarmActuator.uArm_MIN_XYZ,
                            UarmActuator.uArm_MAX_XYZ,
                            pos)]
                        #self.uArm.set_position(x=uarm_pos[0],y=uarm_pos[1],z=uarm_pos[2],timeout=(0.1*self.timeoutSet),speed=1000000)
                        self.uArm.set_position(x=uarm_pos[0],y=uarm_pos[1],z=uarm_pos[2],timeout=0.1,speed=1000000)
                    self.uArm.flush_cmd()
                    self.actPosTrack.clear()

                    #kludge for falied suction:
                    self.uArm.set_pump(on=self.isSuction,timeout=0.1)
                    self.isMoving=False
                pass
            except Exception as e:
                print("[Working error] e: {}".format(e))


    @staticmethod
    def clamping(num,aMin,aMax):
        return min(aMax,max(num,aMin))

    @staticmethod
    def XYZclamping(V,mins,maxes):
        return [UarmActuator.clamping(V[i],mins[i],maxes[i]) for i in [0,1,2]]

    @staticmethod
    def toRelativePos(cur,mins,maxes):
        toPercent = lambda cur, min, max: (cur-min)/(max-min)
        return list(map(toPercent,cur,mins,maxes))

    @staticmethod
    def dist(relativePos1,relativePos2):
        from math import sqrt
        diffSqr = lambda a, b: pow((a-b),2)
        sqrs = list(map(diffSqr,relativePos1,relativePos2))
        return sqrt(sum(sqrs))

    @staticmethod
    def extractLPposFromJson(aJson):
        json_coor=[float(aJson["position_x"]),
                    float(aJson["position_y"]),
                    float(aJson["position_z"])]
        clampped_coor=UarmActuator.XYZclamping(json_coor,UarmActuator.lp_MIN_XYZ,UarmActuator.lp_MAX_XYZ)
        return UarmActuator.toRelativePos(clampped_coor,UarmActuator.lp_MIN_XYZ,UarmActuator.lp_MAX_XYZ)


shouldRun=True

def PollSocket(poller,actuator):
    while shouldRun:
        resultJson=poller.Poll()
        if resultJson is not None:
            actuator.onLeapMotionUpdate(resultJson)

def main():

    poller = LeapMotionPoller()
    actuator = None
    try:
        actuator = UarmActuator()
        actuator.connect()
    except:
        print("[Error] actuator some error, stopped")
        raise
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
        poller.socket.disconnect("tcp://localhost:5556")
        context.destroy()

    return

if __name__ == "__main__":
    main()
