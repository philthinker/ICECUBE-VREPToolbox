-- Threaded Child Script

-- Attach these scripts to UR5 in V-REP scene "UR5plusRG2_PickAndPlace.ttt"
-- This script is used for remote API applications
-- It is not recommanded to use v1 but v2

-- Haopeng Hu
-- 2018.04.13

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

rem_rmlMoveToJointPositions = function(inInts,inFloats,inStrings,inBuffer)
    local targetPos = {0, 0, 0, 0, 0, 0}    -- rad
    if #inFloats>=6 then
        --sim.addStatusbarMessage('Compatible Message Received!')
        for i = 1,6,1 do
            targetPos[i] = inFloats[i]
        end
    end
    if sim.getIntegerSignal('IKEnable') ~= 0 then
        enableIk(false)
        sim.setIntegerSignal('IKEnable', 0)
    end
    if sim.getSimulationState() ~= sim.simulation_advancing_abouttostop then
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos,targetVel)
        --print('maxVel:'..maxVel[1])
    end
    return {},{},{},''
end

rem_rmlMoveToPosition = function(inInts,inFloats,inStrings,inBuffer)
    local targetPos = sim.getObjectPosition(ikTip, -1)
    local targetQua = sim.getObjectQuaternion(ikTip, -1)
    if #inFloats>=7 then
        for i = 1,3,1 do
            targetPos[i] = inFloats[i]
        end
        for i = 1,4,1 do
            targetQua[i] = inFloats[i+3]
        end
    end
    if sim.getIntegerSignal('IKEnable') ~= 1 then
        enableIk(true)
        sim.setIntegerSignal('IKEnable', 1)
    end
    if sim.getSimulationState() ~= sim.simulation_advancing_abouttostop then
        sim.rmlMoveToPosition(ikTarget, -1, -1, nil, nil, ikMaxVel, ikMaxAccel, ikMaxJerk, targetPos, targetQua, nil)
    end
    return {},{},{},''
end

function sysCall_threadmain(  )
    sim.setThreadSwitchTiming(100)
    -- Initialize some values:
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    ikGroupHandle=sim.getIkGroupHandle('UR5')
    ikTip=sim.getObjectHandle('UR5_ikTip')
    ikTarget=sim.getObjectHandle('UR5_ikTarget')

    -- Set-up some of the RML vectors:
    vel = 100
    accel = 20
    jerk = 40
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
    sim.addStatusbarMessage('The UR5 is ready to move!')
    
    while true do
        -- Just hold the thread
    end

    sim.stopSimulation()
end
