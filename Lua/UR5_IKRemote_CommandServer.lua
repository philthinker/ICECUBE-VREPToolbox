-- Non-threaded Child Script

-- Attach these scripts to RemoteAPICommandServer in V-REP scene "UR5plusRG2_PickAndPlace.ttt"

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
    end
    return {},{},{},''
end

rem_rmlMoveToPosition = function(inInts,inFloats,inStrings,inBuffer)
    if sim.getIntegerSignal('IKEnable') ~= 1 then
        enableIk(true)
        sim.setIntegerSignal('IKEnable', 1)
    end
    if sim.getSimulationState ~= sim.simulation_advancing_abouttostop then
        --
    end
    return {},{},{},''
end

function sysCall_init()
    -- do some initialization here:
    jointHandles = {-1,-1,-1,-1,-1,-1}
    for i = 1,6,1 do
        jointHandles[i] = sim.getObjectHandle('UR5_joint'..i)
    end
    ikGroupHandle = sim.getIkGroupHandle('UR5')
    ikTip = sim.getObjectHandle('UR5_ikTip')
    ikTarget = sim.getObjectHandle('UR5_ikTarget')

    --vel = 180
    --accel = 40
    --jerk = 80
    vel = 100
    accel = 20
    jerk = 40
    currentVel={0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0}

    ikMaxVel={0.4,0.4,0.4,1.8}
    ikMaxAccel={0.8,0.8,0.8,0.9}
    ikMaxJerk={0.6,0.6,0.6,0.8}

    initialConfig={0,0,0,0,0,0}
    startConfig={-70.1*math.pi/180,18.85*math.pi/180,93.18*math.pi/180,68.02*math.pi/180,109.9*math.pi/180,90*math.pi/180}

    sim.setIntegerSignal('IKEnable', 0)         -- The sign for ik mechanism
    sim.setIntegerSignal('ClientRunning', 1)    -- The sign for remote client state
    sim.addStatusbarMessage('The UR5 is ready to move!')

end

--function sysCall_actuation()
    -- put your actuation code here
--end

function sysCall_sensing()
    -- put your sensing code here
    if sim.getIntegerSignal('ClientRunning') == 0 then
        sim.stopSimulation()
    end
end

--function sysCall_cleanup()
    -- do some clean-up here
--end

--[[
    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        enableIk(true)

        pos=sim.getObjectPosition(ikTip,-1)
        quat=sim.getObjectQuaternion(ikTip,-1)
        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2]-0.2,pos[3]},quat,nil)
    end

    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.wait(1)
    end

    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2]-0.2,pos[3]+0.2},quat,nil)
    end

    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.wait(1)
    end
]]

    --sim.stopSimulation()
