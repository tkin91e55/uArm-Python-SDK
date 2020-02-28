import sys
import zmq
import zmq.utils.jsonapi as zmqJson

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.8.103:5556")
socket.setsockopt_string(zmq.SUBSCRIBE, "0")

poller = zmq.Poller()
poller.register(socket, zmq.POLLIN)

shouldRun=True

import json
def PollSocket():
    while shouldRun:

        try:
            polledSockets = dict(poller.poll(timeout=500))
            if socket in polledSockets and polledSockets[socket] == zmq.POLLIN:
                jsTxt = socket.recv_string(zmq.DONTWAIT)
                #print(jsTxt)
                jsObj=zmqJson.loads(jsTxt.lstrip("0 "))
                print(jsObj["handType"])
        except:
            pass
         
import threading

def main():
    runner = threading.Thread(target = PollSocket)
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
        socket.disconnect("tcp://192.168.8.103:5556")
        context.destroy()
    return

if __name__ == "__main__":
    main()
