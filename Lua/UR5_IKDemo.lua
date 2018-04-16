-- This is a threaded script.

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

function sysCall_threadmain()
    -- Initialize some values:
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    ikGroupHandle=sim.getIkGroupHandle('UR5')
    ikTip=sim.getObjectHandle('UR5_tip')
    ikTarget=sim.getObjectHandle('UR5_target')

    -- Set-up some of the RML vectors:
    vel=180
    accel=40
    jerk=80
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

    enableIk(false)
    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,startConfig,targetVel)
    end

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

    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2],pos[3]+0.2},quat,nil)
    end

    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.wait(1)
    end

    enableIk(false)
    sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,initialConfig,targetVel)
    sim.stopSimulation()
end