-- Threaded Child Script
-- ICECUBE Communication Protocal v2.0
-- Attach these scripts to UR5 in V-REP scene.

enableIk=function(enable)
    if enable then
        sim.setObjectMatrix(ikTarget,-1,sim.getObjectMatrix(ikTip,-1))
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_ik,1)
        end

        sim.setExplicitHandling(ikGroupHandle,0)
    else
        sim.setExplicitHandling(ikGroupHandle,1)
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_force,0)
        end
    end
end

ICECUBErmlMoveToJointPositions = function(inFloats)   -- rad
    local targetPos = {0,0,0,0,0,0}
    if #inFloats>=6 then
        for i = 1,6,1 do
            targetPos[i] = inFloats[i]
        end
    else
        for i = 1,#inFloats,1 do
            targetPos[i] = inFloats[i]
        end
    end
    if sim.getIntegerSignal('IKEnable') ~= 0 then
        enableIk(false)
        sim.setIntegerSignal('IKEnable', 0)
    end
    if sim.getSimulationState() ~= sim.simulation_advancing_abouttostop then
        local res = sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos,targetVel)
    end
    return res 
end


ICECUBErmlMoveToPosition = function(inFloats)
    local targetPos = sim.getObjectPosition(ikTip, -1)
    local targetQua = sim.getObjectQuaternion(ikTip, -1)
    if #inFloats>=7 then
        for i = 1,3,1 do
            targetPos[i] = inFloats[i]
        end
        for i = 1,4,1 do
            targetQua[i] = inFloats[i+3]
        end
    else
        print('There should be 7 elements in the desired configuration!')
    end
    if sim.getIntegerSignal('IKEnable') ~= 1 then
        enableIk(true)
        sim.setIntegerSignal('IKEnable', 1)
    end
    if sim.getSimulationState() ~= sim.simulation_advancing_abouttostop then
        local res = sim.rmlMoveToPosition(ikTarget, -1, -1, nil, nil, ikMaxVel, ikMaxAccel, ikMaxJerk, targetPos, targetQua, nil)
    end
    return res
end

function sysCall_threadmain(  )
    sim.setThreadSwitchTiming(100)
    sim.setIntegerSignal('UR5READY', 0)
    -- Initialize some values:
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    ikGroupHandle=sim.getIkGroupHandle('UR5')
    ikTip=sim.getObjectHandle('UR5_ikTip')
    ikTarget=sim.getObjectHandle('UR5_ikTarget')

    -- Set-up some of the RML vectors:
    vel = 60
    accel = 40
    jerk = 80
    currentVel={0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0}

    ikMaxVel={0.4,0.4,0.4,1.8}
    ikMaxAccel={0.8,0.8,0.8,0.9}
    ikMaxJerk={0.6,0.6,0.6,0.8}

    initialConfig={0,0,0,0,0,0}
    

    sim.setIntegerSignal('IKEnable', 0)         -- The sign for ik mechanism
    
    -- The ICECUBE Communication Protocol v2.0
    sim.setIntegerSignal('ICECUBE_0', 0)
    for i = 1,7,1 do
        sim.setFloatSignal('ICECUBE_'..i, 0.000000)
    end
    local rmlJoints = {0, 0, 0, 0, 0, 0}
    local rmlPosQua = {0, 0, 0, 0, 0, 0, 0} 

    sim.setIntegerSignal('UR5READY',1)     -- the sign for client applications
    sim.addStatusbarMessage('The UR5 is ready to move!')

    while true do
        -- The ICECUBE Communication Protocol v2.0
        local icecubeCMD = sim.getIntegerSignal('ICECUBE_0')
        local coarseCMD = math.floor( icecubeCMD/10 )
        local fineCMD = icecubeCMD % 10
        if coarseCMD == 0 then
            -- Default
            if fineCMD == 1 then
                -- Stop the simulation
                break
            else
                sim.wait(0.2)
            end
        elseif coarseCMD == 1 then
            -- Joint Motion Plan
            for i = 1,6,1 do
                rmlJoints[i] = sim.getFloatSignal('ICECUBE_'..i)
            end
            if fineCMD == 1 then 
                -- Simple point-to-point joint motion plan
                ICECUBErmlMoveToJointPositions(rmlJoints)
                sim.setIntegerSignal('ICECUBE_0', 0)
            end
        elseif coarseCMD == 2 then
            -- Cartesian Motion Plan
            for i = 1,7,1 do
                rmlPosQua[i] = sim.getFloatSignal('ICECUBE_'..i)
            end
            if fineCMD == 1 then
                -- Simple point-to-point cartesian motion plan
                ICECUBErmlMoveToPosition(rmlPosQua)
                sim.setIntegerSignal('ICECUBE_0',0)
            end
        end
    end
    sim.stopSimulation()
end