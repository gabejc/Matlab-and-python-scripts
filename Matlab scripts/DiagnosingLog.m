% Store releveant variables
close all;

% Altered custom packet a bit

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
% Cust: custom packet (time, control mode, distance to waypoint, region of
% interest location (absolute or relative?), barometer altitude, relative
% position to home (change to gps), inertial altitude


% Attitude: Desired vs actual inputs, mechanical failures (esc, props
% coming off, etc)
attTime=ATT(:,2); DesRoll=ATT(:,3); Roll=ATT(:,4); DesPitch=ATT(:,5); Pitch=ATT(:,6);
DesYaw=ATT(:,7); Yaw=ATT(:,8);

% CMD
if exist('CMD')
    cmdLat=CMD(:,10); cmdLon=CMD(:,11); cmdAlt=CMD(:,end);
else
    disp('No CMDs found')
    cmdLat=0; cmdLon=0; cmdAlt=0;
end

% MODE
controlMode=MODE(:,2);
ThrCrs=MODE(:,3); % autopilot's best guess as to what throttle is required to maintain a stable hover

% Curr
ThrOut=CURR(:,3); Volt=CURR(:,5)/100; Curr=CURR(:,6); Vcc=CURR(:,7);
CurrTot=CURR(:,8); currTime=CURR(:,2);

% ERR
if exist('ERR')
    Subsys=ERR(:,2); ECode=ERR(:,3);
end

% GPS
gpsStatus=GPS(:,2); gpsTime=GPS(:,3); NSats=GPS(:,5); Hdop=GPS(:,6); latitude=GPS(:,7); longitude=GPS(:,8);
relAlt=GPS(:,9); horizSpeed=GPS(:,11); vel_z=GPS(:,13);

% CTUN
ctunTime=CTUN(:,2); Dalt=CTUN(:,6); inavAlt=CTUN(:,7); BarAlt=CTUN(:,8); CRt=CTUN(:,12); ThrIn=CTUN(:,3);
DSAlt=CTUN(:,9); SAlt=CTUN(:,10);

% NTUN
if exist('NTUN');
    ntunTime=NTUN(:,2); VelX=NTUN(:,9)/100; VelY=NTUN(:,10)/100; DAccX=NTUN(:,11); DAccY=NTUN(:,12);
else
    VelX=0; VelY=0; DAccX=0; DAccY=0;
end

% IMU
imuTime=IMU(:,2); AccX=IMU(:,6); AccY=IMU(:,7); AccZ=IMU(:,8);

% PM
NLon=PM(:,2); NLoop=PM(:,3);

% PARM (parameters); storing using logical indexing
Parameters=PARM;
AHRS_EKF_USE=Parameters{strcmp(Parameters(:,1),'AHRS_EKF_USE'),2}; % note curly braces
ATC_SLEW_YAW=Parameters{strcmp(Parameters(:,1),'ATC_SLEW_YAW'),2};
BAROGLTCH_ENABLE=Parameters{strcmp(Parameters(:,1),'BAROGLTCH_ENABLE'),2};
BAROGLTCH_ACCEL=Parameters{strcmp(Parameters(:,1),'BAROGLTCH_ACCEL'),2};
BAROGLTCH_DIST=Parameters{strcmp(Parameters(:,1),'BAROGLTCH_DIST'),2};
BATT_CAPACITY=Parameters{strcmp(Parameters(:,1),'BATT_CAPACITY'),2};
INAV_TC_Z=Parameters{strcmp(Parameters(:,1),'INAV_TC_Z'),2};
WP_YAW_BEHAVIOR=Parameters{strcmp(Parameters(:,1),'WP_YAW_BEHAVIOR'),2};
WPNAV_ACCEL=Parameters{strcmp(Parameters(:,1),'WPNAV_ACCEL'),2};
WPNAV_ACCEL_Z=Parameters{strcmp(Parameters(:,1),'WPNAV_ACCEL_Z'),2};
WPNAV_SPEED=Parameters{strcmp(Parameters(:,1),'WPNAV_SPEED'),2};
WPNAV_SPEED_DN=Parameters{strcmp(Parameters(:,1),'WPNAV_SPEED_DN'),2};
WPNAV_SPEED_UP=Parameters{strcmp(Parameters(:,1),'WPNAV_SPEED_UP'),2};

% Radio
if exist('RAD')
    RSSI=RAD(:,3); RemRSSI=RAD(:,4); RxErrors=RAD(:,8);
else
    RSSI=999; RemRSSI=999; RxErrors=999;
end

% RCIN - RC input
rcInTime=RCIN(:,2);
rollInput=RCIN(:,3);
pitchInput=RCIN(:,4);
throttleInput=RCIN(:,5);
yawInput=RCIN(:,6);

% RCOU - RC output
rcOutTime=RCOU(:,2);
rollOutput=RCOU(:,3);
pitchOutput=RCOU(:,4);
throttleOutput=RCOU(:,5);
yawOutput=RCOU(:,6);


% To test if transmitter/pilot influenced flight / caused crash
% figure; plot(rcInTime,rollInput,rcInTime,rollOutput); ylabel('Roll'); legend('Input','Output')
% figure; plot(rcInTime,pitchInput,rcInTime,pitchOutput); ylabel('Pitch'); legend('Input','Output')
% figure; plot(rcInTime,throttleInput,rcInTime,throttleOutput); ylabel('Throttle'); legend('Input','Output')



% CUST
if exist('CUST')
    custTime=CUST(:,2); CM=CUST(:,3); WpDist=CUST(:,4)/100; roiX=CUST(:,5); roiY=CUST(:,6); roiZ=CUST(:,7);
    custLat=CUST(:,8); custLon=CUST(:,9); custInAlt=CUST(:,10);
    cust_cur_tot=CUST(:,11); DtP=CUST(:,12);
else
    disp('Warning, no custom logs')
    custTime=0; CM=0; WpDist=0; roiX=0; roiY=0; roiZ=0;
    custLat=0; custLon=0; custInAlt=0; cust_cur_tot=0;
end

% Remove 0 points
latitude2=latitude(find(latitude~=0));
longitude2=longitude(find(longitude~=0));
roiX2=roiX(find(roiX~=0));
roiY2=roiY(find(roiY~=0));
roiZ2=roiZ(find(roiZ~=0));
custLat2=custLat(find(custLat~=0));
custLon2=custLat(find(custLat~=0));

% rtl_state: 1=Initial climb, 2=Return home, 3=loiter at home, 4=final
% descent, 5=land

% %% testing rtl_state
% tLat=latitude(rtl_state==2);
% tLon=longitude(rtl_state==2);
% tAlt=relAlt(rtl_state==2);
% figure; plot(tLat,tLon);
% figure; plot(tAlt);


%% Intermediate calculations
AltDif=inavAlt-Dalt; % Positive if above desired
netSpeed=sqrt(horizSpeed.^2+vel_z.^2);
% droneVel=sqrt(VelX.^2+VelY.^2);
netAccel=sqrt(AccX.^2+AccY.^2+AccZ.^2);
% To find average speed based on wp distance
tempMax=WpDist(1); j=1;
for i=2:length(WpDist)
    if (WpDist(i)>(WpDist(i-1)+10))
        WpCust(j,:)=CUST(i,:); % storing just the points when sent to a new WP
        j=j+1;
    end
end
if exist('WpCust')
    truncWpDist=WpCust(:,5)/100;
else
    truncWpDist=0;
end
% truncWpDist=WpCust(:,5)/100;
% timeBetweenWaypoints=CMD(2:end,2)-CMD(1:end-1,2);
% speedBetweenWaypoints

% to test how often region of interest wasn't updated
% sum(sum([roiX(2:end),roiY(2:end)]-[roiX(1:end-1),roiY(1:end-1)]))/2;
t1=roiX(2:end)-roiX(1:end-1)==0;
t2=roiY(2:end)-roiY(1:end-1)==0;
t3=[t1,t2]; t4=sum(t3')';
percentNotUpdated=sum(t4==2)/size(t4,1)*100;
fprintf('Percent roi not updated %f: \n',percentNotUpdated);


% % battery stuff
% minAltInd=find(BarAlt==min(BarAlt));
% CTUN(minAltInd,1); % gives line number of minimum altitude, find closest in CURR to find % drained

minAltIndCust=find(CUST(:,10)==min(CUST(:,10)));
timeTillBottom=CUST(minAltIndCust,2)-CUST(1,2);
totalTime=CUST(end,2)-CUST(1,2);
percentDownhill=timeTillBottom/totalTime;
totalCurrentDraw=CurrTot(end);

% Radio stuff
avgRSSI=mean(RSSI); timesLostRSSI=sum(RSSI==0);
avgRemRSSI=mean(RemRSSI); timesLostRemRSSI=sum(RemRSSI==0);

% Finding voltage draw downhill vs climb vs return
val=cust_cur_tot(custInAlt==min(custInAlt))/cust_cur_tot(end);
% disp(sprintf('Drained at bot: %f ',val*100))
% ind1=find(BarAlt==min(BarAlt)); % index where drone reaches bottom
% percentThroughRun=ind1/size(BarAlt,1);
% ind2=round(percentThroughRun*size(CurrTot,1));
% curDrained=CurrTot(ind2);Ric
% pcurDrained=curDrained/CurrTot(end)

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
% battCapacity=6000; % 6000 mAh for the x8
EndingVoltage=min(Volt);
currentDrained=CurrTot(end);
percentCurrentUsed=currentDrained/(BATT_CAPACITY*0.8); % assuming we don't want to drain more than 80%

% Baro
Pressure=BARO(:,4);
Temperature=BARO(:,5);

% GPS stuff
timesNo3dFix=sum(gpsStatus~=3);
avgNumSats=median(NSats);
minNumSats=min(NSats);
avgHdop=median(Hdop);
maxHdop=max(Hdop);

% Speed stuff
maxSpeed=max(netSpeed); % includes horizontal and vertical speed
avgSpeed=median(netSpeed);
ntunSpeed=sqrt(VelX.^2+VelY.^2);
ntunMaxSpeed=max(ntunSpeed);
ntunAvgSpeed=median(ntunSpeed);

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
% droneLoc=[latitude,longitude,relAlt];
for t=1:length(skierLoc)
    hDistance(t)=GPScalculateDistance(skierLoc(t,1:2),droneLoc(t,1:2)); % horizontal distance from skier to drone
    vDistance(t)=abs(skierLoc(t,3)-droneLoc(t,3));
end
distSkierToDrone=sqrt(hDistance.^2+vDistance.^2);
avgDistSkierToDrone=median(distSkierToDrone);

% %% Calculating distance condition for drone to move on (replicated from
% % arduino rails code)
% % close all
% % Precomputing
% waypoints=[cmdLat(1:end-1),cmdLon(1:end-1),cmdAlt(1:end-1)];
% nWaypoints=size(waypoints,1);
% % for i = 1:nWaypoints-1
% %     waypointVects(i,1) = waypoints(i,1) - waypoints(i+1,1); % vector points from next to prev
% %     waypointVects(i,2) = waypoints(i,2) - waypoints(i+1,2);
% %     waypointVects(i,3) = waypoints(i,3) - waypoints(i+1,3);
% % end
% waypointVects=waypoints(1:end-1,:)-waypoints(2:end,:); % more elegant way
% 
% % calculating
% j=1;
% currWpVect=waypoints(1,1:2)-waypoints(2,1:2);
% railDistanceThreshold=40;
% currentWpIndex=2;
% while currentWpIndex<nWaypoints
% %     currentWpIndex=i; % start at 2?
% %     currentWaypoint=waypoints(currentWpIndex,:);
% %     currentHelmetPosition=[roiX(j),roiY(j),roiZ(j)];
%     currentWaypoint=waypoints(currentWpIndex,1:2); % 2D, since altitude isnt on same scale
%     currentHelmetPosition=[roiX(j),roiY(j)];    
%     
%     % Store vectors (current waypoint to helmet, current waypoint to previous
%     % waypoint)
%     vect_currwp_helmet=currentHelmetPosition-currentWaypoint;
%     currWpVect=waypointVects(currentWpIndex-1,1:2); % wpVects(i) is from wpi to wp(i+1)
%     
%     % Convert to meters and make longitude adjustments
%     vect_currwp_helmet(2) = vect_currwp_helmet(2) * (111111*cos(vect_currwp_helmet(1) * pi / 180));
%     vect_currwp_helmet(1) = vect_currwp_helmet(1) * 111111;
%     currWpVect(2) = currWpVect(2) * (111111*cos(currWpVect(1) * pi / 180));
%     currWpVect(1) = currWpVect(1) * 111111;
%     
%     % Normalize waypoint vector
%     currWpVect=currWpVect/norm(currWpVect);
%      
%     distance_to_plane(j)=dot(vect_currwp_helmet,currWpVect);
% %     fprintf('Distance to plane: %f\n',distance_to_plane);
%     if (distance_to_plane(j) <= railDistanceThreshold && currentWpIndex<nWaypoints)
%         currentWpIndex=currentWpIndex+1;
% %         currWpVect=waypointVects(currentWpIndex - 1,1:2)
% %         currWpVect=waypoints(currentWpIndex-1,1:2)-waypoints(currentWpIndex,1:2); % point toward departing wp
%     end
%     j=j+1;
% end
% 
% figure; plot(distance_to_plane);

%%

% Misc
avgThrustToHover=median(ThrCrs);
distanceTraveled=sum(truncWpDist); % better way would be adding WPdist to the CMD packet
distStartToEnd=GPScalculateDistance([cmdLat(1),cmdLon(1)],[cmdLat(end),cmdLon(end)]);
maxAboveAltDif=max(AltDif);
maxBelowAltDif=min(AltDif);

altitudeDrop=abs(min(cmdAlt));
slopeOfRun=altitudeDrop/distStartToEnd;
disp(sprintf('Slope of run: %f',slopeOfRun))

% % Error messages
% errExist=sum(strcmp(fieldnames(openFile),'ERR')); % 1 if there is an error message
% if errExist
%     errSubsys=openFile.ERR(:,2);
%     errECode=openFile.ERR(:,3);
% end


%% Plots
% % things to plot
% % net speed (note params)
% % tracking (gps path and wp's it was following)
% % altitude (desired vs actual)
% % RSSI values
% % ROI, as '*': can more easily see connection quality
% % distance from wp
% 
% close all;
% 

% Pitch tracking
figure; plot(attTime,DesPitch,attTime,Pitch)
legend('Desired pitch','Actual pitch')

% Accelerations
figure; plot(imuTime,AccX,imuTime,AccY,imuTime,AccZ)
legend('X','Y','Z'); title('Vibrations')

% Compass
magTime=MAG(:,2); MagX=MAG(:,3); MagY=MAG(:,4); MagZ=MAG(:,5);
magNorm=sqrt(MagX.^2+MagY.^2+MagZ.^2);
figure; plot(magNorm);
title('Compass performance')


% Region of interest
if abs(sum(roiX))>0 % if the helmet is on
    followingASkier=1; % boolean
else
    followingASkier=0;
end
if followingASkier % if we're following a skier
    figure; plot(roiX2,roiY2,'*r') % plotting discretized shows how often region of interest is being updated
    title('Region of interest'); ylabel('latitude'); xlabel('longitude');
end



% % % Distance from skier
% if followingASkier
%     figure;
%     plot(custTime,hDistance,custTime,vDistance,custTime,distSkierToDrone)
%     title('Distances of skier from drone')
%     legend('Horizontal distance','Vertical distance','Net distance')
% end

% % Speed
% figure; plot(gpsTime,netSpeed); title ('speed')
% figure; plot(Hdop); title('Hdop')
% 
% % % for first rail path, two rogue points (last one at 0,0). second one rogue
% newCMD=CMD;
% % newCMD=removerows(CMD,2);
% newCMD=removerows(newCMD,length(newCMD));
% % newtimeBetweenWaypoints=(newCMD(2:end,2)-newCMD(1:end-1,2))*10^-3;
% % speedBetweenWaypoints=truncWpDist./newtimeBetweenWaypoints;
% % [truncWpDist,speedBetweenWaypoints]
% % % speedBetweenWaypoints=truncWpDist(2:end)./newtimeBetweenWaypoints;
% % % [truncWpDist(2:end),speedBetweenWaypoints]
% 
% % Track follow with waypoints. 
% latitude2(109)=latitude2(108);
figure; plot(cmdLat,cmdLon,'*'); hold on; plot(latitude2,longitude2)
title('Drone lat lon flight path')
% % figure; plot(cmdLat,cmdLon,'*'); hold on; plot(custLat,custLon)
% figure; plot(cmdLat(1:end-1),cmdLon(1:end-1),'*'); hold on; plot(latitude,longitude) % last one at 0,0
% figure; plot(newCMD(:,10),newCMD(:,11),'*'); hold on; plot(latitude,longitude)
% % plot(WpCust(:,11),WpCust(:,12),'k+') % location of drone when new cmd is
% % issued (rel to home though?)
% 
% % % unique to ned crash file
% % pt1=GPS(148,7:8); % find(GPS(:,1)==16576) % for the ned crash file
% % pt2=GPS(167,7:8); % find(GPS(:,1)==18652)
% % pt3=GPS(178,7:8); % find(GPS(:,1)==19854)
% % pt4=GPS(241,7:8); % find(GPS(:,1)==26737)
% % temp=[pt1;pt2;pt3;pt4];
% % plot(temp(:,1),temp(:,2),'*r') % location of drone when new Wp Cmd is issued
% % % drone is being set to the next wp when its still like 20m from its
% % % target....should be 2m....unit issue somewhere?
% 

% figure; plot(gpsTime,vel_z,gpsTime,horizSpeed)
% title('Horizontal and vertical speeds')
% legend('Vertical','Horizontal')

% % Actual vs commanded altitude (baro and LRF)
figure; plot(ctunTime,Dalt,ctunTime,inavAlt); legend('Desired','Fused');%,'LRF')
title('Drone altitude')
ylabel('Altitude (m)'); xlabel('time (s)')
% hold on; plot(cmdAlt,'*');
% figure; plot(ctunTime,DSAlt,ctunTime,SAlt); legend('Desired','Actual')
% 
% 

% figure; plot(latitude,relAlt,'k'); hold on; 
figure; plot(custLat,custInAlt,'k'); hold on; 
plot(cmdLat(2:end),cmdAlt(2:end),'r'); plot(cmdLat(2:end),cmdAlt(2:end),'b*')
legend('Actual path','Interpolated commanded path'); title('Altitude vs latitude')
ylabel('Altitude (m)')

figure; plot(custTime,DtP/100);
title('Distance to plane')
ylabel('Meters (m)'); xlabel('Time')

% % Battery stuff
% figure; plot(currTime,Curr,currTime,CurrTot)
% legend('Current draw','Total current draw') % to easily compare come much of battery depleted 
% % at particular points
