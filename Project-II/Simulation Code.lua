function sysCall_init() 
    
    mobileRob=sim.getObject('.') -- the handle of the mobilerob "Pioneer_p3dx"
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor") -- Handle of the left motor
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor") -- Handle of the right motor
    
    leftSensor=sim.getObjectHandle("LeftSensor") -- Handle of the left IR sensor
    middleSensor=sim.getObjectHandle("MiddleSensor") -- Handle of the middle IR sensor
    rightSensor=sim.getObjectHandle("RightSensor") -- Handle of the right IR sensor
    
    setSpeed=100*math.pi/180 -- the max running speed; will slow down when making turns (how about 100, 300, 500)
    
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0}) -- the mobileRob moving trace
    
    -- You may declare other global variables here
    
    
    
    

end


function sysCall_sensing()
    local p=sim.getObjectPosition(mobileRob,-1)
    sim.addDrawingObjectItem(robotTrace,p)
end 


function sysCall_actuation()
    
    -- These lines read sensors from robot.
    sensorReading={false,false,false}
    sensorReading[1]=(sim.readVisionSensor(leftSensor)) -- Left IR sensor 
    sensorReading[2]=(sim.readVisionSensor(middleSensor)) -- Middle IR sensor
    sensorReading[3]=(sim.readVisionSensor(rightSensor)) -- -- Right IR sensor
   
    --- Line tracking according to IR sensor readings
    vLeft=setSpeed -- Default to move forward
    vRight=setSpeed

    if (sensorReading[1]==0 or sensorReading[2]==0 or sensorReading[3]==0) then -- line tracking while on the line
    
        if (sensorReading[3]~=0) then -- Right sensor is out of the line; Turn left
            vLeft=vLeft*0.05
        end
        if (sensorReading[1]~=0) then -- left sensor is out of the line;Turn right
            vRight=vRight*0.05
        end
    
    else
    
        -- Write your own codes for broken line processing here
    




    
    end
    
    -- Rolling!
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

end 


function sysCall_cleanup() 
 
end 



