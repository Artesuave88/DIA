import tkinter as tk
import random
import math
import numpy as np

class Bot():
    def checkBattery(self, chargerL, chargerR):
        if self.batteryLevel<20:
            chargerL
            chargerR            
            
            
            
        

    def __init__(self,namep):
        self.x = random.randint(100,300)
        self.y = random.randint(100,200)
        self.theta = random.uniform(0.0,2.0*math.pi)
        #self.theta = 0
        self.name = namep
        self.ll = 60 #axle width
        self.vl = 0.0
        self.vr = 0.0
        self.battery = 2000

    def draw(self,canvas):
        points = [ (self.x + 30*math.sin(self.theta)) - 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - 30*math.cos(self.theta)) - 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - 30*math.sin(self.theta)) - 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + 30*math.cos(self.theta)) - 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - 30*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + 30*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x + 30*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - 30*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta)  \
                ]
        canvas.create_polygon(points, fill="blue", tags=self.name)

        self.sensorPositions = [ (self.x + 20*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y - 20*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta), \
                                 (self.x - 20*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y + 20*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta)  \
                            ]
    
        centre1PosX = self.x 
        centre1PosY = self.y
        canvas.create_oval(centre1PosX-8,centre1PosY-8,\
                           centre1PosX+8,centre1PosY+8,\
                           fill="gold",tags=self.name)
        batteryText = canvas.create_text(self.x,self.y,text=str(self.battery),tags=self.name)        

        wheel1PosX = self.x - 30*math.sin(self.theta)
        wheel1PosY = self.y + 30*math.cos(self.theta)
        canvas.create_oval(wheel1PosX-3,wheel1PosY-3,\
                                         wheel1PosX+3,wheel1PosY+3,\
                                         fill="red",tags=self.name)

        wheel2PosX = self.x + 30*math.sin(self.theta)
        wheel2PosY = self.y - 30*math.cos(self.theta)
        canvas.create_oval(wheel2PosX-3,wheel2PosY-3,\
                                         wheel2PosX+3,wheel2PosY+3,\
                                         fill="green",tags=self.name)

        sensor1PosX = self.sensorPositions[0]
        sensor1PosY = self.sensorPositions[1]
        sensor2PosX = self.sensorPositions[2]
        sensor2PosY = self.sensorPositions[3]
        canvas.create_oval(sensor1PosX-3,sensor1PosY-3, \
                           sensor1PosX+3,sensor1PosY+3, \
                           fill="yellow",tags=self.name)
        canvas.create_oval(sensor2PosX-3,sensor2PosY-3, \
                           sensor2PosX+3,sensor2PosY+3, \
                           fill="yellow",tags=self.name)
        
    # cf. Dudek and Jenkin, Computational Principles of Mobile Robotics
    def move(self,canvas,dt,registryPassives):
                #battery
        self.battery -= 1
        if self.battery<0:
            self.battery = 0
        if self.battery==0:
            self.vl = 0.0
            self.vr = 0.0
        for pp in registryPassives:
            if isinstance(pp,Charger):
                chX, chY = pp.getLocation()
                if math.sqrt(  (chX-self.x)*(chX-self.x) + (chY-self.y)*(chY-self.y) )<100: #if near battery, recharge
                    self.battery += 5
        
        
        
        
        if self.vl==self.vr:
            R = 0
        else:
            R = (self.ll/2.0)*((self.vr+self.vl)/(self.vl-self.vr))
        omega = (self.vl-self.vr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta        
        if self.vl==self.vr: # straight line movement
            self.x += self.vr*math.cos(self.theta) #vr wlog
            self.y += self.vr*math.sin(self.theta)
        canvas.delete(self.name)
        self.draw(canvas)
    def senseCharger(self, registryPassives):
        lightL = 0.0
        lightR = 0.0
        for pp in registryPassives:
            if isinstance(pp,Charger):
                lx,ly = pp.getLocation()
                distanceL = math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                                       (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )
                distanceR = math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                                       (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )
                lightL += 200000/(distanceL*distanceL)
                lightR += 200000/(distanceR*distanceR)
        return lightL, lightR

    def senseLight(self, registryPassives):
        lightL = 0.0
        lightR = 0.0
        for pp in registryPassives:
            if isinstance(pp,Lamp):
                lx,ly = pp.getLocation()
                distanceL = math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                                       (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )
                distanceR = math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                                       (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )
                lightL += 200000/(distanceL*distanceL)
                lightR += 200000/(distanceR*distanceR)
        return lightL, lightR
    
    
    def senseHeat(self, registryPassives):
        heatL = 0.0
        heatR = 0.0
        for pp in registryPassives:
            if isinstance(pp,Heater):
                hx,hy = pp.getLocation()
                distanceL = math.sqrt( (hx-self.sensorPositions[0])*(hx-self.sensorPositions[0]) + \
                                       (hy-self.sensorPositions[1])*(hy-self.sensorPositions[1]) )
                distanceR = math.sqrt( (hx-self.sensorPositions[2])*(hx-self.sensorPositions[2]) + \
                                       (hy-self.sensorPositions[3])*(hy-self.sensorPositions[3]) )
                heatL += 200000/(distanceL*distanceL)
                heatR += 200000/(distanceR*distanceR)
        return heatL, heatR
    
    def toCharger(self,registryPassives):
        chargerL = 0.0
        chargerR = 0.0
        for pp in registryPassives:
            if isinstance(pp,Heater):
                cx,cy = pp.getLocation()
                distanceL = math.sqrt( (cx-self.sensorPositions[0])*(cx-self.sensorPositions[0]) + \
                                       (cy-self.sensorPositions[1])*(cy-self.sensorPositions[1]) )
                distanceR = math.sqrt( (cx-self.sensorPositions[2])*(cx-self.sensorPositions[2]) + \
                                       (cy-self.sensorPositions[3])*(cy-self.sensorPositions[3]) )
                chargerL += 200000/(distanceL*distanceL)
                chargerR += 200000/(distanceR*distanceR)
        return chargerL, chargerR        

    def transferFunction(self, lightL, lightR,chargeL, chargeR):
        
        #One of the must be commented out
            #Towards the light
             if  self.battery>=1000:

                if lightR>100:
                    lightR=math.sqrt((lightR))
                if lightL>100:
                    lightL=math.sqrt((lightL))
                if lightR <3:
                    lightR=lightR**2
                self.vl = lightR
                self.vr = lightL
             if self.battery<1000:
                    #move towards charger
                    self.vl = math.sqrt(chargeR)*5
                    self.vr = math.sqrt(chargeL)*5
             if chargeR+chargeL>400 and self.battery<1500:
                    #near charger: stop and wait to charge
                    self.vl = 0
                    self.vr = 0            
#Towards the heat 

#afraid of the light
"""
                self.vl=lightL
                self.vr=lightR
"""
class Lamp():
    def __init__(self,namep):
        self.centreX=400
        self.centreX = random.randint(100,500)
        self.centreY = random.randint(100,500)
        self.name = namep
        
    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-10,self.centreY-10, \
                                  self.centreX+10,self.centreY+10, \
                                  fill="yellow",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY
class Charger():
    def __init__(self,namep):
        self.centreX = random.randint(100,900)
        self.centreY = random.randint(100,900)
        self.name = namep
        
    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-10,self.centreY-10, \
                                  self.centreX+10,self.centreY+10, \
                                  fill="red",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY



class Heater():
    def __init__(self,namep):
        self.centreX = random.randint(100,900)
        self.centreY = random.randint(100,900)
        self.name = namep
        
    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-10,self.centreY-10, \
                                  self.centreX+10,self.centreY+10, \
                                  fill="red",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY



def initialise(window):
    window.resizable(False,False)
    canvas = tk.Canvas(window,width=1000,height=1000)
    canvas.pack()
    return canvas

def register(canvas):
    registryActives = []
    registryPassives = []
    noOfBots    = input('How many Bots?: ')
    noOfBots=    int(noOfBots)

    noOfLights = input('How many Lights?: ')
    noOfLights=int(noOfLights)
    noOfHeaters   = input ('How many heaters?: ')
    noOfHeaters=int(noOfHeaters)
    for i in range(0,noOfBots):
        bot = Bot("Bot"+str(i))
        registryActives.append(bot)
        bot.draw(canvas)
    for i in range(0,noOfLights):
        lamp = Lamp("Lamp"+str(i))
        registryPassives.append(lamp)
        lamp.draw(canvas)
    for i in range(0,noOfHeaters):
        heater = Heater("Heater"+str(i))
        registryPassives.append(heater)
        heater.draw(canvas)
    charger = Charger("Charger0")
    registryPassives.append(charger)
    charger.draw(canvas)                
    return registryActives, registryPassives

def moveIt(canvas,registryActives,registryPassives):
    for rr in registryActives:
        lightIntensityL, lightIntensityR = rr.senseLight(registryPassives)
        chargeL, chargeR = rr.senseCharger(registryPassives)
        rr.transferFunction(lightIntensityL, lightIntensityR, chargeL, chargeR)
        rr.move(canvas,0.5,registryPassives)
    canvas.after(50,moveIt,canvas,registryActives,registryPassives)

def main():

    window = tk.Tk()
    canvas = initialise(window)
    registryActives, registryPassives = register(canvas)
    moveIt(canvas,registryActives,registryPassives)
    window.mainloop()
main()
