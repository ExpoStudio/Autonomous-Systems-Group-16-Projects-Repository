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

    -- Note that arrays in Lua start at 1, not 0. So the first entry in the array corresponds to sensor 1, the second entry corresponds to sensor 2, and so on. The last entry in the array corresponds to sensor 8.
    -- For each entry in the array, it represents the weight of the corresponding proximity sensor. The sign indicates the effect it has on the wheel speed. If the weight is positive, it will influence the wheel to speed up, if its negative it will influence the wheel to slow down. The more the weight, the more influence it has on the wheel's speed.
    braitenbergLeftWheelFrontSensorWeights={1,2,-2,-1} -- Braitenberg weights for the 4 front prox sensors (avoidance). These are proximity sensors 3,4,5,6 in the usensors array.
    -- Braitenberg weights for the 2 side prox sensors (sensors 2 and 7 in the usensors array). These are used for following an object on the side.
    braitenbergLeftWheelSideSensorWeights={-1.5,0} -- Braitenberg weights for the 2 side prox sensors (following)
    -- The outer sensors in the array (sensors 1 and 8) are not necessary to include because objects detected on the side should not affect the speed of the robot (it already passed the object and/or is in the clear path).
    -- unused   -1      1       2      -2      -1       0     unused
    --   |      |       |       |       |       |       |        |
    --   1      2       3       4       5       6       7        8
    -- MODEL FOR VISUALIZATION PURPOSES

    -- Braitenberg weights from the right wheel's perspective
    braitenbergRightWheelFrontSensorWeights={-1,-2,2,1} -- Braitenberg weights for the 4 front prox sensors (avoidance)
    braitenbergRightWheelSideSensorWeights={0,1.5} -- Braitenberg

    -- unused   0      -1       -2      2       1       1     unused
    --   |      |       |       |       |       |       |        |
    --   1      2       3       4       5       6       7        8
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
    

    --if the sensors detect somethin infront, slow the robot down. The more the sensors detect, the slower the robot moves.
    --local speedFactor=1
    --for i=3,6,1 do
    --    if proxDist[i] < noDetectionDist then
    --        speedFactor = speedFactor - 0.25
    --    end
    --end

    -- Line tracking according to IR sensor readings
    vLeft=setSpeed--*speedFactor -- Default to move forward
    vRight=setSpeed--*speedFactor

    local lastTrackedDirection = "left"

    -- Velocity control according to opMode
    if ((sensorReading[1]==0 or sensorReading[2]==0 or sensorReading[3]==0) and (proxDist[3]+proxDist[4]+proxDist[5]+proxDist[6])==(noDetectionDist*4)) then -- line tracking mode
        if (sensorReading[3]~=0) then -- Right sensor is out of the line; Turn left
            vLeft=vLeft*0.05
        end
        if (sensorReading[1]~=0) then -- left sensor is out of the line;Turn right
            vRight=vRight*0.05
        end
        
    else -- obstacle avoidance mode ONLY IF there is nothing in front of the front sensors AND something is detected by the side sensors in a close enough range
        if (proxDist[3]+proxDist[4]+proxDist[5]+proxDist[6]==noDetectionDist*4) then
            -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
            --- Add your code here
            local leftSum = 0
            local rightSum = 0

            -- if the proximities of the left sensors are greater than half the no detection distance, then we disable braigtenburg weights
            if (proxDist[7]+proxDist[8] >= noDetectionDist*1.2 or proxDist[2]+proxDist[1] >= noDetectionDist*1.2) then
            leftSum = distanceFactor(noDetectionDist, proxDist[2])*braitenbergLeftWheelFrontSensorWeights[1] + distanceFactor(noDetectionDist, proxDist[7])*braitenbergLeftWheelFrontSensorWeights[2]
            rightSum = distanceFactor(noDetectionDist, proxDist[2])*braitenbergRightWheelFrontSensorWeights[1] + distanceFactor(noDetectionDist, proxDist[7])*braitenbergRightWheelFrontSensorWeights[2]
            
            vLeft = vLeft + leftSum
            vRight = vRight + rightSum
            end



            -- say we turn right and then wrap around the obstacle. this implies that since we turned right, now we have to increase the left wheel velocity and decrease the right wheel.
            -- so we can maintain a certain distance around the object using the right sensor. If the right sensor leaves .5m way from the obstacle, but is also less than the nodetectionDistance, increase wheel left speed factor until its within that range. Then 
            -- if the right sensor is detecting an object within a certain distance, we want to turn left to trail around it.
            local didRightSensorMeetCondition = (proxDist[7]+proxDist[8] < noDetectionDist*1.2)
            local didLeftSensorMeetCondition = (proxDist[2]+proxDist[1] < noDetectionDist*1.2)

            print("Right sensor condition: " .. tostring(didRightSensorMeetCondition))
            print("Left sensor condition: " .. tostring(didLeftSensorMeetCondition))

            if (didRightSensorMeetCondition and not didLeftSensorMeetCondition) then
                vRight = setSpeed
                vLeft = vLeft * .5
                print("Turning left")
                lastTrackedDirection = "left"
            elseif (didLeftSensorMeetCondition and not didRightSensorMeetCondition) then
                vLeft = setSpeed
                vRight = vRight * .5
                print("Turning right")
                lastTrackedDirection = "right"
            end

            if (lastTrackedDirection == "left") then
                print("Continuing to turn left")
                vRight = setSpeed
                vLeft = vLeft * .5
                print("Turning left")
            elseif (lastTrackedDirection == "right") then
                print("Continuing to turn right")
                vLeft = setSpeed
                vRight = vRight * .5
                print("Turning right")
            end

            -- We use the side sensors to detect if there is an obstacle on the side. If there is, we want to keep a constant distance with it. We can use the Braitenberg weights for the side sensors to determine how much to turn based on the distance of the object. If the object is closer to the left sensors, we turn left to trail around the object.
        else    
            -- Obstacle in front. Use Braitenberg to avoid it
            --- Add your code here

            -- if the object is closer to the left sensors, turn right; if the object is closer to the right sensors, turn left
            -- beacuse each of the proximity sensors has a weight and value depending on the distance of the object, we can write an if statement that determines which direction it turns by comparing left front sensors 1-3 to the right front sensors 4-6. If the left sensors are greater than the right sensors, turn right, and vice versa.
            -- So we sum the braitenberg weights for the left wheel and the right wheel and compare them. If the left wheel is greater than the right wheel, we turn right, and vice versa.
            local leftSum = 0
            local rightSum = 0

            for i=1,4,1 do
                local sensorInd = i + 2
                leftSum = leftSum + distanceFactor(noDetectionDist, proxDist[sensorInd])*braitenbergLeftWheelFrontSensorWeights[i]
                rightSum = rightSum + distanceFactor(noDetectionDist, proxDist[sensorInd])*braitenbergRightWheelFrontSensorWeights[i]
            end
            
            leftSum = leftSum + distanceFactor(noDetectionDist, proxDist[2])*braitenbergLeftWheelSideSensorWeights[1] + distanceFactor(noDetectionDist, proxDist[7])*braitenbergLeftWheelSideSensorWeights[2]
            rightSum = rightSum + distanceFactor(noDetectionDist, proxDist[2])*braitenbergRightWheelSideSensorWeights[1] + distanceFactor(noDetectionDist, proxDist[7])*braitenbergRightWheelSideSensorWeights[2]


            vLeft = vLeft + leftSum
            vRight = vRight + rightSum
        end
    end

    -- Rolling!
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

end 

-- because the proximity to the object is a raw float, it means that if its closer to the object, it will be a smaller number. This inherently results in a smaller influence on the velocities when multiplying by the braigtenburg weights, resulting in a weaker turn effect as the object gets closer. This function returns the proper inverse of the distance.
function distanceFactor(noDetectionDist, proximalDistance)
    local strength = 0
        strength = 1 - (proximalDistance / noDetectionDist) 
    return math.max(0, strength)
end

function sysCall_cleanup() 

    
end 

