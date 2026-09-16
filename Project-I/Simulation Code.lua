function sysCall_init() 
    
    mobileRob=sim.getObject('.') -- the handle of the mobilerob "Pioneer_p3dx"
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor") -- Handle of the left motor
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor") -- Handle of the right motor
    -- Testing and things of that nature
    leftSensor=sim.getObjectHandle("LeftSensor") -- Handle of the left IR sensor
    middleSensor=sim.getObjectHandle("MiddleSensor") -- Handle of the middle IR sensor
    rightSensor=sim.getObjectHandle("RightSensor") -- Handle of the right IR sensor
    
    setSpeed=100*math.pi/180 -- the max running speed; will slow down when making turns (how about 150, 200, 300)
    
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0}) -- the mobileRob moving trace
        
    usensors={-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,8,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end

    noDetectionDist=1 -- You have to change this value [0 1]...
    proxDist={noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist}

    braitenbergLeftFrontSens={1,2,-2,-1} -- Braitenberg weights for the 4 front prox sensors (avoidance)
    braitenbergLeftSideSens={-1,0} -- Braitenberg weights for the 2 side prox sensors (following)

end


function sysCall_sensing()
    local p=sim.getObjectPosition(mobileRob,-1)
    sim.addDrawingObjectItem(robotTrace,p)
end 


function sysCall_actuation()

    local res=0
    local dist=0    
    -- These lines read sensors from robot.
    sensorReading={false,false,false}
    sensorReading[1]=(sim.readVisionSensor(leftSensor)) -- Left IR sensor 
    sensorReading[2]=(sim.readVisionSensor(middleSensor)) -- Middle IR sensor
    sensorReading[3]=(sim.readVisionSensor(rightSensor)) -- -- Right IR sensor

    -- Since there are 16 sensors in robot, so we want to read values from frontal 8 sensors.
    for i=1,8,1 do
        -- To read, using sim.readProximitySensor for each of usensors from initilization.
        -- "res" stores detect status of a sensor. 0 - undetect, 1 - detect
        -- "dist" stores the distance from a sensor to the detected object. See the example values from slide.
        -- After the following line, we can get status and value from all sensors in the robot.
        res,dist=sim.readProximitySensor(usensors[i])
        -- According to the "dist", we check whether an obstacle is detected. 
        proxDist[i]=noDetectionDist
        if (res>0) and (dist<noDetectionDist) then
            proxDist[i]=dist
        end
    end
    

    -- Line tracking according to IR sensor readings
    vLeft=setSpeed -- Default to move forward
    vRight=setSpeed

    -- Velocity control according to opMode
    if ((sensorReading[1]==0 or sensorReading[2]==0 or sensorReading[3]==0) and (proxDist[2]+proxDist[3]+proxDist[4]+proxDist[5])==(noDetectionDist*4)) then -- line tracking mode
        if (sensorReading[3]~=0) then -- Right sensor is out of the line; Turn left
            vLeft=vLeft*0.05
        end
        if (sensorReading[1]~=0) then -- left sensor is out of the line;Turn right
            vRight=vRight*0.05
        end
    else -- obstacle avoidance mode
        if (proxDist[3]+proxDist[4]+proxDist[5]+proxDist[6]==noDetectionDist*4) then
            -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
            --- Add your code here


        else
            -- Obstacle in front. Use Braitenberg to avoid it
            --- Add your code here



        end
    end

    -- Rolling!
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

end 


function sysCall_cleanup() 

end 
