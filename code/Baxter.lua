------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and earlier: 
colorCorrectionFunction=function(_aShapeHandle_) 
    local version=sim.getInt32Parameter(sim.intparam_program_version) 
    local revision=sim.getInt32Parameter(sim.intparam_program_revision) 
    if (version<30104)and(revision<3) then 
        return _aShapeHandle_ 
    end 
    return '@backCompatibility1:'.._aShapeHandle_ 
end 
------------------------------------------------------------------------------ 
 
 
-- This is a simple non-threaded example script that enables/disables some of Baxter's devices:

if (sim_call_type==sim.syscb_init) then 
    local headSensorOn=true
    local headCameraOn=false
    local rightHandSensorsOn=false
    local leftHandSensorsOn=false
    local headLightColor={1,0,0}

    for i=1,12,1 do
        local h=sim.getObjectHandle('Baxter_ultrasonic_sensor'..i)
        if headSensorOn then
            sim.setExplicitHandling(h,0)
        else
            sim.setExplicitHandling(h,1)
            sim.resetProximitySensor(h)
        end
    end

    local headCam=sim.getObjectHandle('Baxter_camera')
    if headCameraOn then
        sim.setExplicitHandling(headCam,0)
    else
        sim.setExplicitHandling(headCam,1)
        sim.resetVisionSensor(headCam)
    end

    local rightHandProx=sim.getObjectHandle('Baxter_rightArm_proxSensor')
    local rightHandCam=sim.getObjectHandle('Baxter_rightArm_camera')
    if rightHandSensorsOn then
        sim.setExplicitHandling(rightHandProx,0)
        sim.setExplicitHandling(rightHandCam,0)
        local camView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
        sim.adjustView(camView,rightHandCam,64)
    else
        sim.setExplicitHandling(rightHandProx,1)
        sim.resetProximitySensor(rightHandProx)
        sim.setExplicitHandling(rightHandCam,1)
        sim.resetVisionSensor(rightHandCam)
    end
    
    local leftHandProx=sim.getObjectHandle('Baxter_leftArm_proxSensor')
    local leftHandCam=sim.getObjectHandle('Baxter_leftArm_camera')
    if leftHandSensorsOn then
        sim.setExplicitHandling(leftHandProx,0)
        sim.setExplicitHandling(leftHandCam,0)
        local camView=sim.floatingViewAdd(0.1,0.9,0.2,0.2,0)
        sim.adjustView(camView,leftHandCam,64)
    else
        sim.setExplicitHandling(leftHandProx,1)
        sim.resetProximitySensor(leftHandProx)
        sim.setExplicitHandling(leftHandCam,1)
        sim.resetVisionSensor(leftHandCam)
    end

    headLight=sim.getObjectHandle('Baxter_headRing')
    sim.setShapeColor(colorCorrectionFunction(headLight),'',3,headLightColor)
end 

if (sim_call_type==sim.syscb_cleanup) then 
    sim.setShapeColor(colorCorrectionFunction(headLight),'',3,{0,1,0})
end 

