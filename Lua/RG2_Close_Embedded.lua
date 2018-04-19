-- Non-Threaded Child Script

-- Attach these scripts to RG2 in V-REP scene "UR5plusRG2_PickAndPlace.ttt"
-- Use these scripts for the Pick-and-Place demo

rg2Close = function()
    local v = -motorVelocity
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
    return {},{v},{},''
end

rg2Open = function()
    local v = motorVelocity
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
    return {},{v},{},''
end

function sysCall_init( )
    motorHandle=sim.getObjectHandle('RG2_openCloseJoint')
    motorVelocity=0.05 -- m/s
    motorForce=20 -- N
    sim.setIntegerSignal('RG2Close', 0)
end

function sysCall_actuation( )
    -- Hear from UR5_PickAndPlace_Demo
    local rg2Sign = sim.getIntegerSignal('RG2Close')
    if rg2Sign ~= 0 then
        rg2Close()
    else
        rg2Open()
    end
end