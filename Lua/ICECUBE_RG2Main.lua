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
            sim.setIntegerSignal('RG2GRASP', 1)
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
end

function sysCall_init( )
    motorHandle=sim.getObjectHandle('RG2_openCloseJoint')
    motorVelocity=0.05 -- m/s
    motorForce=20 -- N

    connector=sim.getObjectHandle('RG2_attachPoint')
    objectSensor=sim.getObjectHandle('RG2_attachProxSensor')

    sim.setIntegerSignal('RG2CLOSED', 1)
    sim.setIntegerSignal('RG2GRASP', 0)
    sim.setIntegerSignal('RG2CMD', 0)
end

function sysCall_actuation( )
    local rg2cmdSign = sim.getIntegerSignal('RG2CMD')
    local rg2closeSign = sim.getIntegerSignal('RG2CLOSED')
    local rg2graspSign = sim.getIntegerSignal('RG2GRASP')
    if rg2cmdSign == 0 then
        -- Open
        if rg2closeSign == 0 then
            -- Now it is open, pass
        else
            -- Now it is closed, open it
            sim.setIntegerSignal('RG2CLOSED', 0)
            rg2Open()
            if rg2graspSign == 0 then
                -- Nothing grasped, pass
            else
                -- Something grasped, release it
                sim.setIntegerSignal('RG2GRASP', 0)
                rg2Release()
            end
        end
    elseif rg2cmdSign == 1 then
        -- Close
        if rg2closeSign == 0 then
            -- Now it is open, close it
            sim.setIntegerSignal('RG2CLOSED', 1)
            rg2Close()
            rg2Grasp()
        else
            -- Now it is closed, pass
        end
    end
end

