function sysCall_init()
    -- Initialize various components and set up the robot for operation
    bubbleRobBase = sim.getObject('.')
    leftMotor = sim.getObject("./leftMotor")
    rightMotor = sim.getObject("./rightMotor")
    noseSensor = sim.getObject("./sensingNose")
    minMaxSpeed = {50 * math.pi / 180, 300 * math.pi / 180} -- Specifies the minimum and maximum speeds for motors
    backUntilTime = -1 -- Tells whether the robot is in forward or backward mode
    floorSensorHandles = {-1, -1, -1} -- Handles to the sensors used for line detection
    floorSensorHandles[1] = sim.getObject("./leftSensor")
    floorSensorHandles[2] = sim.getObject("./middleSensor")
    floorSensorHandles[3] = sim.getObject("./rightSensor")
    robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, {1, 1, 0}, nil, nil, {1, 1, 0})
    
    -- Create the custom UI:
    xml = '<ui title="'..sim.getObjectAlias(bubbleRobBase,1)..' speed" closeable="false" resizeable="false" activate="false">'..[[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
                <label text="" style="* {margin-left: 300px;}"/>
            </ui>
        ]]
    ui = simUI.create(xml)
    speed = (minMaxSpeed[1] + minMaxSpeed[2]) * 0.5 -- Initializes the robot's speed to the midpoint between the minimum and maximum values
    simUI.setSliderValue(ui, 1, 100 * (speed - minMaxSpeed[1]) / (minMaxSpeed[2] - minMaxSpeed[1])) -- Initial value of slider
end

function sysCall_sensing()
    local p = sim.getObjectPosition(bubbleRobBase, -1) -- Gets the current position of the robot
    sim.addDrawingObjectItem(robotTrace, p) -- Adds the current position to the robot trace
end

function speedChange_callback(ui, id, newVal)
    -- Updates the robot speed based on the slider's position
    speed = minMaxSpeed[1] + (minMaxSpeed[2] - minMaxSpeed[1]) * newVal / 100
end

function sysCall_actuation()
    -- Obstacle Detection: Checks if the proximity sensor detects an obstacle
    local result = sim.readProximitySensor(noseSensor)
    if result > 0 then
        backUntilTime = sim.getSimulationTime() + 4
    end

    -- Read the line detection sensors
    sensorReading = {false, false, false}
    for i = 1, 3, 1 do
        local result, data = sim.readVisionSensor(floorSensorHandles[i])
        if result >= 0 then
            sensorReading[i] = (data[11] < 0.5) -- data[11] is the average intensity of the image
        end
    end

    -- Compute left and right velocities to follow the detected line
    rightV = speed
    leftV = speed
    if sensorReading[1] then
        leftV = 0.03 * speed
    end
    if sensorReading[3] then
        rightV = 0.03 * speed
    end
    if sensorReading[1] and sensorReading[3] then
        backUntilTime = sim.getSimulationTime() + 2
    end

    if backUntilTime < sim.getSimulationTime() then
        -- When in forward mode, move forward at the desired speed
        sim.setJointTargetVelocity(leftMotor, leftV)
        sim.setJointTargetVelocity(rightMotor, rightV)
    else
        -- When in backward mode, back up in a curve at reduced speed
        sim.setJointTargetVelocity(leftMotor, -speed / 2)
        sim.setJointTargetVelocity(rightMotor, -speed / 8)
    end
end

function sysCall_cleanup()
    simUI.destroy(ui)
    -- Clean up resources
end
