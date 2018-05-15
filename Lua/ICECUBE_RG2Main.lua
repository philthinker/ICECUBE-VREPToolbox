-- Non-Threaded Child Script
-- ICECUBE Communication Protocal v2.0
-- Attach these scripts to RG2

rg2Close = function()
    local v = -motorVelocity
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
end

rg2Grasp = function()
    index=0
    while true do
        -- Search for non-static and respondable objects
        shape = sim.getObjects(index,sim.object_shape_type)
        if shape == -1 then
            break
        end
        if (sim.getObjectInt32Parameter(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Parameter(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
            -- Ok, we find a non-static, respondable shape that was detected by the proximity sensor.
            attachedShape = shape
            -- Connection
            sim.setObjectParent(attachedShape,connector,false)
            sim.setIntegerSignal('RG2Grasp', 1)
            break
        end
        index = index + 1
    end
end

rg2Open = function()
    local v = motorVelocity
    sim.setJointForce(motorHandle,motorForce)
    sim.setJointTargetVelocity(motorHandle,v)
end

rg2Release = function()
    sim.setObjectParent(attachedShape,-1,true)
    sim.setIntegerSignal('RG2Grasp', 0)
end

function sysCall_init( )
    motorHandle=sim.getObjectHandle('RG2_openCloseJoint')
    motorVelocity=0.05 -- m/s
    motorForce=20 -- N

    connector=sim.getObjectHandle('RG2_attachPoint')
    objectSensor=sim.getObjectHandle('RG2_attachProxSensor')

    sim.setIntegerSignal('RG2Close', 0)
    sim.setIntegerSignal('RG2Grasp', 0)
end

function sysCall_actuation( )
    local rg2Sign = sim.getIntegerSignal('RG2Close')
    if rg2Sign ~= 0 then
        rg2Close()
        rg2Grasp()
    else
        if sim.getIntegerSignal('RG2Grasp') ~= 0 then
            rg2Release()
        end
        rg2Open()
    end
end
