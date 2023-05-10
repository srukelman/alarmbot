import numpy as np
import matplotlib as plt

class fourwheelomni:
    #constructor for a 4 wheel omni-drive/x-drive given its initial pose (x, y, rotation), 
    def __init__(self, initPose, wheelDistance, wheelRadius):
        self.pose = initPose
        self.wheelDistance = wheelDistance
        self.wheelRadius = wheelRadius
    
    def getPose(self):
        return self.pose
    def setPose(self, pose):
        self.pose = pose
    
    def getWheelDistance(self):
        return self.wheelDistance
    def setWheelDistance(self, wheelDistance):
        self.wheelDistance = wheelDistance

    def getWheelRadius(self):
        return self.wheelRadius
    def setWheelRadius(self, wheelRadius):
        self.wheelRadius = wheelRadius

    def drive(self, xVel, yVel, tVel, fieldRel, dt):
        #convert field relative value to robot relative values
        if fieldRel:
            pass
        #scaling up x and y velocities to make up for angles wheels (multiplying by sqrt 2 rounded)
        xVel *= 1.414
        yVel *= 1.414

        #setting wheel velocities
        wheel0V = yVel - xVel + (tVel * self.wheelDistance)
        wheel1V = -yVel -xVel + (tVel * self.wheelDistance)
        wheel2V = -yVel + xVel + (tVel * self.wheelDistance)
        wheel3V = yVel +xVel + (tVel * self.wheelDistance)

        #calculating robot relative x, y, and t robot speeds
        xSpeed = (-wheel0V - wheel1V + wheel2V + wheel3V)/4/1.414
        ySpeed = (wheel0V - wheel1V - wheel2V + wheel3V)/4/1.414
        tSpeed = (wheel0V + wheel1V + wheel2V + wheel3V)/4/self.wheelDistance

        #updating pose based on current speeds and dt (change in time since last update)
        currPose = self.getPose()
        currPose[0] += (xSpeed * dt)
        currPose[1] += (ySpeed * dt)
        currPose[2] += (tSpeed * dt)
        self.setPose(currPose)

