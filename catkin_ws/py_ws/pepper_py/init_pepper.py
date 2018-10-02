# -*- encoding: UTF-8 -*-
import qi
import argparse
import sys
from naoqi import ALProxy
import numpy as np

#host = '163.221.124.228'
host = '169.254.246.15'


def Degree(rad):
    degree = rad/np.pi*180
    return degree

def Connect_pepper():
    speech = ALProxy("ALTextToSpeech", host, 9559)
    speech.setLanguage("English")
    speech.say("start")

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=host,
                        help="Robot IP address. On robot or Local Naoqi: use host.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    return session,speech

def Init_pepper(session,tts):
    #tts.say("initialize")
    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    # Wake up robot
    motion_service.wakeUp()
    # Send robot to Stand Init
    posture_service.goToPosture("StandInit", 0.5)
    #motion_service.rest()
    bodyNames = motion_service.getBodyNames("Body")
    print "Body:"
    print str(bodyNames)
    print ""

    #tts.say("check angle")
    names         = "Joints"
    useSensors  = True
    sensorAngles = motion_service.getAngles(names, useSensors)
    print "Sensor angles:"

    angles = []
    for angle in sensorAngles:
        angles.append(Degree(angle))
    print str(angles)
    print ""

    return motion_service,posture_service

def Hund_test(session,tts,motion_service):
    #tts.say("Close hund")
    print "Close hund"
    motion_service  = session.service("ALMotion")

    handName  = 'LHand'
    motion_service.closeHand(handName)

    handName  = 'RHand'
    motion_service.closeHand(handName)


if __name__ == "__main__":
    session,tts = Connect_pepper()
    motion_service,posture_service = Init_pepper(session,tts)
    Hund_test(session,tts,motion_service)


