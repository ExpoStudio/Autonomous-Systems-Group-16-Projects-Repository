function sysCall_init() 
    
    mobileRob=sim.getObject('.') -- the handle of the mobilerob "Pioneer_p3dx"
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor") -- Handle of the left motor
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor") -- Handle of the right motor
    
    leftSensor=sim.getObjectHandle("LeftSensor") -- Handle of the left IR sensor
    middleSensor=sim.getObjectHandle("MiddleSensor") -- Handle of the middle IR sensor
    rightSensor=sim.getObjectHandle("RightSensor") -- Handle of the right IR sensor
    
    setSpeed=100*math.pi/180 -- the max running speed; will slow down when making turns (how about 150, 200, 300)
    
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0}) -- the mobileRob moving trace
        
    usensors={-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,8,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end

    -- Distance where no detection is assumed. If the distance is greater than this value, it is considered that there is no obstacle in front of the robot.
    noDetectionDist=0.5 -- You have to change this value [0 1]...
    -- Experimental Distance changed to .5

    -- Array to store the distance from each sensor to the detected object. If no object is detected, the value is set to "noDetectionDist".
    proxDist={noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist,noDetectionDist}

    -- Braitenberg weights from the left wheel's perspective

    -- for each entry in the array, it represents the weight of the corresponding proximity sensor. The sign indicates the effect it has on the wheel speed. If the weight is positive, it will influence the wheel to speed up, if its negative it will influence the wheel to slow down. The more the weight, the more influence it has on the wheel's speed.
    braitenbergLeftWheelFrontSensorWeights={1,2,-2,-1} -- Braitenberg weights for the 4 front prox sensors (avoidance). These are proximity sensors 3,4,5,6 in the usensors array.
    -- Braitenberg weights for the 2 side prox sensors (sensors 1 and 6 in the usensors array). These are used for following an object on the side.
    braitenbergLeftWheelSideSensorWeights={-1,0} -- Braitenberg weights for the 2 side prox sensors (following)
    -- The outer sensors in the array (sensors 0 and 7) are not necessary to include because objects detected on the side should not affect the speed of the robot (it already passed the object and/or is in the clear path).
    -- unused   -1      1       2      -2      -1       0     unused
    --   |      |       |       |       |       |       |        |
    --   0      1       2       3       4       5       6        7
    -- MODEL FOR VISUALIZATION PURPOSES

    -- Braitenberg weights from the right wheel's perspective
    braitenbergRightWheelFrontSensorWeights={-1,-2,2,1} -- Braitenberg weights for the 4 front prox sensors (avoidance)
    braitenbergRightWheelSideSensorWeights={0,1} -- Braitenberg

    -- unused   0      -1       -2      2       1       1     unused
    --   |      |       |       |       |       |       |        |
    --   0      1       2       3       4       5       6        7
    -- MODEL FOR VISUALIZATION PURPOSES

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
    

    -- if the sensors detect somethin infront, slow the robot down. The more the sensors detect, the slower the robot moves.
    local speedFactor=1
    for i=1,8,1 do
        if proxDist[i] < noDetectionDist then
            speedFactor = speedFactor - 0.1
        end
    end
    -- Speed factor is probably unnecessary

    -- Line tracking according to IR sensor readings
    vLeft=setSpeed*speedFactor -- Default to move forward
    vRight=setSpeed*speedFactor

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

            -- We use the side sensors to detect if there is an obstacle on the side. If there is, we want to keep a constant distance with it. We can use the Braitenberg weights for the side sensors to determine how much to turn based on the distance of the object. If the object is closer to the left sensors, we turn left to trail around the object.
        else    
            -- Obstacle in front. Use Braitenberg to avoid it
            --- Add your code here

            -- if the object is closer to the left sensors, turn right; if the object is closer to the right sensors, turn left
            -- beacuse each of the proximity sensors has a weight and value depending on the distance of the object, we can write an if statement that determines which direction it turns by comparing left front sensors 1-3 to the right front sensors 4-6. If the left sensors are greater than the right sensors, turn right, and vice versa.
            -- So we sum the braitenberg weights for the left wheel and the right wheel and compare them. If the left wheel is greater than the right wheel, we turn right, and vice versa.
            
            local leftSum = proxDist[1]+proxDist[2]+proxDist[3]
            local rightSum = proxDist[4]+proxDist[5]+proxDist[6]

            if (turningDirection == 0) then
                if (leftSum < rightSum) then
                    turningDirection = -1 -- turn left
                else
                    vRight=vRight*0.05
                    turningDirection = 1 -- turn right
                end
            end

            if (turningDirection == -1) then
                vLeft=vLeft*0.05
            elseif (turningDirection == 1) then
                vRight=vRight*0.05
            end

            -- CHANGE LATER. TURNING CODE IS NOT NEEDED BECAUSE BRAITENBERG WILL HANDLE IT. JUST USE BRAITENBERG WEIGHTS TO DETERMINE TURNING DIRECTION AND SPEED.

        end
    end

    -- Rolling!
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

end 


function sysCall_cleanup() 

end 

