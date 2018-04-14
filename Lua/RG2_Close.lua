--No Threaded Child Script

function sysCall_init( )
    motorHandle=sim.getObjectHandle('RG2_openCloseJoint')
    motorVelocity=0.05 -- m/s
    motorForce=20 -- N
end

rg2Close = function(close)
    v = motorVelocity
    if close then
        v = -motorVelocity
    end
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
    return {},{},{},''
end