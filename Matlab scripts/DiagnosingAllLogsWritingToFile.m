clear; clc;
% Store releveant variables

% Labels
% Att: attitude info, desired vs actual attitude information
% Camera: time and position when camera shutte was activated
% CMD: Commands executed as part of mission. Lat/lon
% Compass: compass readings
% CURR: current info (throttle out, voltage, current, board voltage, total
% current)
% CTUN: control tuning (throttle in/out, barometer altitude, desired climb
% rate, climb rate)
% EV: Event record
% ERR: Errors
% GPS: gps info (num sats, hdop, lat/lon, rel alt (to launch), speed, vert speed, time)
% Mode: flight mode when changed, throttle needed to maintain altitude
% IMU: accelerometer and gyro info
% INAV: inertial navigation info (inertial position from home)
% NTUN: navigation (desired vs actual position, velocities, and
% accelerations)
% PM: performance monitoring (slow loops and such)
% Parm: parameters which were stored. If changed, not sure how this is
% handeled
% Radio: quantifying radio performance
% Cust: custom packet (time, control mode, distance to waypoint, region of
% interest location (absolute or relative?), barometer altitude, relative
% position to home (change to gps), inertial altitude

% workingDir='/Users/gcharalambides/Desktop/Cape Flight Logs/All utah flight logs';
workingDir='/Users/gcharalambides/Desktop/sansa utah 2 matlab files';
% workingDir='/Users/gcharalambides/Desktop/test';
fileList=dir(workingDir);

for i=1:numel(fileList)
    clearvars -except workingDir fileList names i
    names{i}=fileList(i).name;
    if i>2 % first two are just dots
        fileName=names{i};
        fileLoc=strcat(workingDir,'/',fileName);
        openFile=load(fileLoc);
        
        % Attitude: Desired vs actual inputs, mechanical failures (esc, props
        % coming off, etc)
        DesRoll=openFile.ATT(:,3); Roll=openFile.ATT(:,4); DesPitch=openFile.ATT(:,5); Pitch=openFile.ATT(:,6);
        DesYaw=openFile.ATT(:,7); Yaw=openFile.ATT(:,8);
        
        % Barometer
        airPressure=openFile.BARO(:,4); airTemp=openFile.BARO(:,5);
        
        % CMD
        cmdLat=openFile.CMD(:,10); cmdLon=openFile.CMD(:,11); cmdAlt=openFile.CMD(:,end);
        
        % MODE
        controlMode=openFile.MODE(:,2);
        ThrCrs=openFile.MODE(:,3); % autopilot's best guess as to what throttle is required to maintain a stable hover
        
        % Curr
        ThrOut=openFile.CURR(:,3); Volt=openFile.CURR(:,5)/100; Curr=openFile.CURR(:,6); Vcc=openFile.CURR(:,7);
        CurrTot=openFile.CURR(:,8); currTime=openFile.CURR(:,2);
        
        % GPS
        gpsStatus=openFile.GPS(:,2); gpsTime=openFile.GPS(:,3); NSats=openFile.GPS(:,5); Hdop=openFile.GPS(:,6); latitude=openFile.GPS(:,7); longitude=openFile.GPS(:,8);
        relAlt=openFile.GPS(:,9); horizSpeed=openFile.GPS(:,11); vel_z=openFile.GPS(:,13);
        
        % CTUN
        ctunTime=openFile.CTUN(:,2); Dalt=openFile.CTUN(:,6); inavAlt=openFile.CTUN(:,7);
        BarAlt=openFile.CTUN(:,8); CRt=openFile.CTUN(:,12); ThrIn=openFile.CTUN(:,3);
        
        % NTUN
        ntunExist=sum(strcmp(fieldnames(openFile),'NTUN'));
        if ntunExist
            VelX=openFile.NTUN(:,9)/100; VelY=openFile.NTUN(:,10)/100; DAccX=openFile.NTUN(:,11); DAccY=openFile.NTUN(:,12);
        else
            VelX=0; VelY=0; DAccX=0; DAccY=0;
        end
        
        
        % IMU
        AccX=openFile.IMU(:,6); AccY=openFile.IMU(:,7); AccZ=openFile.IMU(:,8);
        
        % PM
        NLon=openFile.PM(:,2); NLoop=openFile.PM(:,3);
        
        % PARM (parameters); storing using logical indexing
        Parameters=openFile.PARM;
        AHRS_EKF_USE=Parameters{strcmp(Parameters(:,1),'AHRS_EKF_USE'),2}; % note curly braces
        ATC_SLEW_YAW=Parameters{strcmp(Parameters(:,1),'ATC_SLEW_YAW'),2};
        BAROGLTCH_ENABLE=Parameters{strcmp(Parameters(:,1),'BAROGLTCH_ENABLE'),2};
        BAROGLTCH_ACCEL=Parameters{strcmp(Parameters(:,1),'BAROGLTCH_ACCEL'),2};
        BAROGLTCH_DIST=Parameters{strcmp(Parameters(:,1),'BAROGLTCH_DIST'),2};
        INAV_TC_Z=Parameters{strcmp(Parameters(:,1),'INAV_TC_Z'),2};
        WP_YAW_BEHAVIOR=Parameters{strcmp(Parameters(:,1),'WP_YAW_BEHAVIOR'),2};
        WPNAV_ACCEL=Parameters{strcmp(Parameters(:,1),'WPNAV_ACCEL'),2};
        WPNAV_ACCEL_Z=Parameters{strcmp(Parameters(:,1),'WPNAV_ACCEL_Z'),2};
        WPNAV_SPEED=Parameters{strcmp(Parameters(:,1),'WPNAV_SPEED'),2};
        WPNAV_SPEED_DN=Parameters{strcmp(Parameters(:,1),'WPNAV_SPEED_DN'),2};
        WPNAV_SPEED_UP=Parameters{strcmp(Parameters(:,1),'WPNAV_SPEED_UP'),2};
        
        % Radio
        RSSI=openFile.RAD(:,3); RemRSSI=openFile.RAD(:,4); RxErrors=openFile.RAD(:,8);
        
        % CUST
        custTime=openFile.CUST(:,2); CM=openFile.CUST(:,3); WpDist=openFile.CUST(:,5)/100; roiX=openFile.CUST(:,7); roiY=openFile.CUST(:,8); roiZ=openFile.CUST(:,9);
        custBarAlt=openFile.CUST(:,10); custLat=openFile.CUST(:,11); custLon=openFile.CUST(:,12);
        custInAlt=openFile.CUST(:,13); rtl_state=openFile.CUST(:,14); custCurTot=openFile.CUST(:,15);
    
        %% Intermediate calculations
        AltDif=inavAlt-Dalt; % Positive if above desired
        netSpeed=sqrt(horizSpeed.^2+vel_z.^2);
        % droneVel=sqrt(VelX.^2+VelY.^2);
        netAccel=sqrt(AccX.^2+AccY.^2+AccZ.^2);
        % To find average speed based on wp distance
        tempMax=WpDist(1); j=1;
        for i=2:length(WpDist)
            if (WpDist(i)>(WpDist(i-1)+10))
                WpCust(j,:)=openFile.CUST(i,:); % storing just the points when sent to a new WP
                j=j+1;
            end
        end
        if exist('WpCust')
            truncWpDist=WpCust(:,5)/100;
        else
            truncWpDist=0;
        end
        % timeBetweenWaypoints=CMD(2:end,2)-CMD(1:end-1,2);
        % speedBetweenWaypoints
        
        % Counting how often region of interest seems to not be updated
%         numXzero=sum((roiX(2:end)-roiX(1:end-1))==0);
%         numYzero=sum((roiY(2:end)-roiY(1:end-1))==0);
        t1=roiX(2:end)-roiX(1:end-1)==0;
        t2=roiY(2:end)-roiY(1:end-1)==0;    
        t3=[t1,t2]; t4=sum(t3')';
        percentNotUpdated=sum(t4==2)/size(t4,1)*100;
%         percentNotUpdated2=numYzero/size(roiY,1)*100;
        
        
%         % battery stuff
%         minAltInd=find(BarAlt==min(BarAlt));
%         openFile.CTUN(minAltInd,1); % gives line number of minimum altitude, find closest in CURR to find % drained
        
        % Radio stuff
        avgRSSI=mean(RSSI); timesLostRSSI=sum(RSSI==0); percentRssiLost=timesLostRSSI/size(RSSI,1)*100;
        avgRemRSSI=mean(RemRSSI); timesLostRemRSSI=sum(RemRSSI==0); percentRemRssiLost=timesLostRemRSSI/size(RemRSSI,1)*100;
              
        minAltIndCust=find(openFile.CUST(:,10)==min(openFile.CUST(:,10)));
        timeTillBottom=openFile.CUST(minAltIndCust,2)-openFile.CUST(1,2);
        totalTime=openFile.CUST(end,2)-openFile.CUST(1,2);
        percentDownhill=timeTillBottom/totalTime;
        totalCurrentDraw=CurrTot(end);
        
        %% Logging params
        
        % when applicable, also add...
        % - boolean if person is in frame, perhaps even which quadrant of frame
        % - temperature
        % - Total dist traveled (perhaps change calc method when new CMD logging)
        % (downhill?)
        % - separate into down vs uphill? at least for battery stuff?
        % - 1-off params: temperature, wind speed, payload, intended height above
        % ground, testing date, which drone it is
        % - fused altitude + barometer reading?
        
        % Current/battery stuff
        battCapacity=6000; % 6000 mAh for the x8
        EndingVoltage=median(Volt(end-round(.05*size(Volt,1)))); % essentially applies a low pass filter to
        % get the ending voltage
        currentDrained=CurrTot(end);
        percentCurrentUsed=currentDrained/(battCapacity*0.8); % assuming we don't want to drain more than 80%
        
        % GPS stuff
        timesNo3dFix=sum(gpsStatus~=3);
        avgNumSats=median(NSats);
        minNumSats=min(NSats);
        avgHdop=median(Hdop);
        maxHdop=max(Hdop);
        
        % Speed stuff
        maxSpeed=max(netSpeed); % includes horizontal and vertical speed
        avgSpeed=mean(netSpeed);
%         ntunSpeed=sqrt(VelX.^2+VelY.^2); % doesn't incorporate z
%         ntunMaxSpeed=max(ntunSpeed); % doesn't incorporate z
        newSpeedVec=netSpeed(netSpeed>2); % ignore entries on ground essentially
        gpsAvgSpeed=median(newSpeedVec);
        
        % Time in air
        inAir=length(find(ThrOut>0)); % number of entries in air measured by having non-zero throttle output
        onGround=length(find(ThrOut==0)); % number of entries on ground
        percentInAir=inAir/(inAir+onGround);
        timeElapsed=currTime(end)-currTime(1);
        timeInAir=timeElapsed*percentInAir;
        timeInAir=(timeInAir*10^-3)/60; % covert to minutes
        estimatedPossibleTimeInAir=timeInAir/percentCurrentUsed;
        
        % Distance from skier to drone
        skierLoc=[roiX,roiY,roiZ];
        droneLoc=[custLat,custLon,custInAlt];
        for t=1:length(skierLoc)
            hDistance(t)=GPScalculateDistance(skierLoc(t,1:2),droneLoc(t,1:2)); % horizontal distance from skier to drone
            vDistance(t)=abs(skierLoc(t,3)-droneLoc(t,3));
        end
        distSkierToDrone=sqrt(hDistance.^2+vDistance.^2);
        avgDistSkierToDrone=median(distSkierToDrone);
        
        % Misc
        avgThrustToHover=median(ThrCrs);
        distanceTraveled=sum(truncWpDist); % better way would be adding WPdist to the CMD packet
        distStartToEnd=GPScalculateDistance([cmdLat(1),cmdLon(1)],[cmdLat(end),cmdLon(end)]);
        maxAboveAltDif=max(AltDif);
        maxBelowAltDif=min(AltDif);
        avgAirPressure=floor(median(airPressure));
        avgAirTemp=round(median(airTemp)*(9/5)+32); % convert to F
        
        
        % Error messages
        errExist=sum(strcmp(fieldnames(openFile),'ERR')); % 1 if there is an error message
        if errExist
            errSubsys=openFile.ERR(:,2);
            errECode=openFile.ERR(:,3);
        end
        
        
        %% write log to a file  
        fid=fopen('tempLogWrite.txt','at');        
        fprintf(fid,'%s, %.1f, %.2f, %.2f, %i, %.1f, %.2f, %.1f, %.2f, %.2f, %i, %i, %i, %i, %i, %i, %i, %i, %.2f, %.2f, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i',...
             fileName, avgDistSkierToDrone, timeInAir, estimatedPossibleTimeInAir, round(avgThrustToHover), distanceTraveled,...
             EndingVoltage, currentDrained, maxSpeed, gpsAvgSpeed, round(percentNotUpdated), round(avgRSSI),...
             round(percentRssiLost), round(avgRemRSSI), round(percentRemRssiLost), timesNo3dFix, avgNumSats, minNumSats, avgHdop, maxHdop, AHRS_EKF_USE, ATC_SLEW_YAW,...
             INAV_TC_Z, WP_YAW_BEHAVIOR, WPNAV_ACCEL, WPNAV_ACCEL_Z, WPNAV_SPEED, WPNAV_SPEED_DN, WPNAV_SPEED_UP, avgAirPressure, avgAirTemp);
        
        if errExist
            fprintf(fid, '%i, ', errExist); % formatting was tricky with printing vectors
            for i2=1:length(errSubsys)
                if i2<length(errSubsys)
                    fprintf(fid, '%i:%i ',errSubsys(i2),errECode(i2));
                else
                   fprintf(fid, '%i:%i',errSubsys(i2),errECode(i2)); 
                end
                    
            end
        end
        
        fprintf(fid,'\n');
        fclose(fid);
    end
end