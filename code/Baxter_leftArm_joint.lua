getShiftedMatrix=function(matrix,shift,dir,absoluteShift)

    local m={}
    for i=1,12,1 do
        m[i]=matrix[i]
    end
    if absoluteShift then
        m[4]=m[4]+dir*shift[1]
        m[8]=m[8]+dir*shift[2]
        m[12]=m[12]+dir*shift[3]
    else
        m[4]=m[4]+dir*(m[1]*shift[1]+m[2]*shift[2]+m[3]*shift[3])
        m[8]=m[8]+dir*(m[5]*shift[1]+m[6]*shift[2]+m[7]*shift[3])
        m[12]=m[12]+dir*(m[9]*shift[1]+m[10]*shift[2]+m[11]*shift[3])
    end
    return m
end

_getJointPosDifference=function(startValue,goalValue,isRevolute)
    local dx=goalValue-startValue
    if (isRevolute) then
        if (dx>=0) then
            dx=math.mod(dx+math.pi,2*math.pi)-math.pi
        else
            dx=math.mod(dx-math.pi,2*math.pi)+math.pi
        end
    end
    return(dx)
end

_applyJoints=function(jointHandles,joints)
    for i=1,#jointHandles,1 do
        sim.setJointTargetPosition(jointHandles[i],joints[i])
    end
end

getConfig=function()
   
    local config={}
    for i=1,#jh,1 do
        config[i]=sim.getJointPosition(jh[i])
    end
    return config
end

getConfigConfigDistance=function(config1,config2)
    
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

generatePathLengths=function(path)
   
    local d=0
    local l=#jh
    local pc=#path/l
    local retLengths={0}
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
        retLengths[i+1]=d
    end
    return retLengths
end

generateDirectPath=function(goalConfig,steps)
    local startConfig=getConfig()
    local dx={}
    for i=1,#jh,1 do
        dx[i]=(goalConfig[i]-startConfig[i])/(steps-1)
    end
    local path={}
    for i=1,steps,1 do
        for j=1,#jh,1 do
            path[#path+1]=startConfig[j]+dx[j]*(i-1)
        end
    end
    return path, generatePathLengths(path)
end

generateIkPath=function(goalPose,steps)
    
    sim.setObjectMatrix(ikTarget,-1,goalPose)
    local c=sim.generateIkPath(ikGroup,jh,steps)
    if c then
        return c, generatePathLengths(c)
    end
end

executeMotion=function(path,lengths,maxVel,maxAccel,maxJerk)
    dt=sim.getSimulationTimeStep()

    
    jointsUpperVelocityLimits={}
    for j=1,7,1 do
        res,jointsUpperVelocityLimits[j]=sim.getObjectFloatParameter(jh[j],sim.jointfloatparam_upper_limit)
    end
    velCorrection=1

    sim.setThreadAutomaticSwitch(false)
    while true do
        posVelAccel={0,0,0}
        targetPosVel={lengths[#lengths],0}
        pos=0
        res=0
        previousQ={path[1],path[2],path[3],path[4],path[5],path[6],path[7]}
        local rMax=0
        rmlHandle=sim.rmlPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
        while res==0 do
            res,posVelAccel,sync=sim.rmlStep(rmlHandle,dt)
            if (res>=0) then
                l=posVelAccel[1]
                for i=1,#lengths-1,1 do
                    l1=lengths[i]
                    l2=lengths[i+1]
                    if (l>=l1)and(l<=l2) then
                        t=(l-l1)/(l2-l1)
                        for j=1,7,1 do
                            q=path[7*(i-1)+j]+_getJointPosDifference(path[7*(i-1)+j],path[7*i+j],jt[j]==sim.joint_revolute_subtype)*t
                            dq=_getJointPosDifference(previousQ[j],q,jt[j]==sim.joint_revolute_subtype)
                            previousQ[j]=q
                            r=math.abs(dq/dt)/jointsUpperVelocityLimits[j]
                            if (r>rMax) then
                                rMax=r
                            end
                        end
                        break
                    end
                end
            end
        end
        sim.rmlRemove(rmlHandle)
        if rMax>1.001 then
            velCorrection=velCorrection/rMax
        else
            break
        end
    end
    sim.setThreadAutomaticSwitch(true)

    posVelAccel={0,0,0}
    targetPosVel={lengths[#lengths],0}
    pos=0
    res=0
    jointPos={}
    rmlHandle=sim.rmlPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
    while res==0 do
        dt=sim.getSimulationTimeStep()
        res,posVelAccel,sync=sim.rmlStep(rmlHandle,dt)
        if (res>=0) then
            l=posVelAccel[1]
            for i=1,#lengths-1,1 do
                l1=lengths[i]
                l2=lengths[i+1]
                if (l>=l1)and(l<=l2) then
                    t=(l-l1)/(l2-l1)
                    for j=1,7,1 do
                        jointPos[j]=path[7*(i-1)+j]+_getJointPosDifference(path[7*(i-1)+j],path[7*i+j],jt[j]==sim.joint_revolute_subtype)*t
                    end
                    _applyJoints(jh,jointPos)
                    break
                end
            end
        end
        sim.switchThread()
    end
    sim.rmlRemove(rmlHandle)
end

setStageAndWaitForNext=function(stageNumber)
    sim.setIntegerSignal(signalName,stageNumber)
    while true do
        stage=sim.getIntegerSignal(signalName)
        if (stage==stageNumber+1) then
            break
        end
        sim.switchThread() -- don't waste CPU time
    end
end

jh={-1,-1,-1,-1,-1,-1,-1}
jt={-1,-1,-1,-1,-1,-1,-1}
for i=1,7,1 do
    jh[i]=sim.getObjectHandle('Baxter_leftArm_joint'..i)
    jt[i]=sim.getJointType(jh[i])
end
baseHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
baxterBaseHandle=sim.getObjectHandle('Baxter')
ikGroup=sim.getIkGroupHandle('Baxter_leftArm')
ikTarget=sim.getObjectHandle('Baxter_leftArm_target')
target1=sim.getObjectHandle('Baxter_leftArm_mpTarget1')
name=sim.getObjectName(baseHandle)
suffix=sim.getNameSuffix(name)

signalName='BaxterLeftArmSignal'
if (suffix>=0) then
    signalName=signalName..'#'..suffix
end
    
maxVel=3    
maxAccel=1
maxJerk=8000
ikSteps=20
fkSteps=160
metric={1,1,1,1,1,1,1}


setStageAndWaitForNext(0)

-- Open Gripper
gripperName = 'BaxterGripper'
sim.setIntegerSignal(gripperName..'_close',0)

-- step 1
config={-80*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 2
config={-80*math.pi/180, 9*math.pi/180, -3.5*math.pi/180, 8*math.pi/180, 8*math.pi/180, 80*math.pi/180, -20*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 3
gripperName = 'BaxterGripper'
sim.setIntegerSignal(gripperName..'_close',1)
sim.wait(2)

-- step 4
config={-60*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 5
config={48*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 6
config={48*math.pi/180, 9*math.pi/180, -3.5*math.pi/180, 10*math.pi/180, 10*math.pi/180, 80*math.pi/180, -25*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Open Gripper
gripperName = 'BaxterGripper'
sim.setIntegerSignal(gripperName..'_close',0)
sim.wait(3)

-- step 7
config={48*math.pi/180, 9*math.pi/180, -3.5*math.pi/180, 10*math.pi/180, 10*math.pi/180, 95*math.pi/180, -5*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Close Gripper
gripperName = 'BaxterGripper'
sim.setIntegerSignal(gripperName..'_close',1)
sim.wait(2)

-- step 8
config={48*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 9
config={48*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 9
config={-60*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- step 10
config={-80*math.pi/180, 9*math.pi/180, -3.5*math.pi/180, 8*math.pi/180, 8*math.pi/180, 75*math.pi/180, -20*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Open Gripper
gripperName = 'BaxterGripper'
sim.setIntegerSignal(gripperName..'_close',0)

-- step 11
config={0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

