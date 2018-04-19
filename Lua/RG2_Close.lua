-- Non-Threaded Child Script

-- Attach these scripts to RG2 in V-REP scene "UR5plusRG2.ttt"

rg2Close = function(inInts,inFloats,inStrings,inBuffer)
    local v = -motorVelocity
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
    return {},{v},{},''
end

rg2Open = function(inInts,inFloats,inStrings,inBuffer)
    local v = motorVelocity
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
    return {},{v},{},''
end

function sysCall_init( )
    motorHandle=sim.getObjectHandle('RG2_openCloseJoint')
    motorVelocity=0.05 -- m/s
    motorForce=20 -- N
end