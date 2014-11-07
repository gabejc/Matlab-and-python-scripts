% Script which will input a .mat log file and output latitude, longitude,
% altitude from the flight

% Issues: Be wary of the altitude readings from helmet 1



%%
clear trueTrack gpsTrack
close all

latitude=GPS(2:end,7); longitude=GPS(2:end,8); altitude=GPS(2:end,9); % note, chopping off the first point
gpsTrack=[latitude,longitude,altitude]; % vary of innacurate barometer (table top testing varied by 8m)
i=1;j=1;k=1;
% i2=1;
% bool=true;
pt1=[gpsTrack(1,1),gpsTrack(1,2)];
distThreshold=20; % waypoints spaced this 3D distance apart
eps=2; % distThreshold +/- eps
trueTrack(1:2,:)=gpsTrack(1:2,:); % initialize first points of the rail
k=k+2;
bool=true;
while bool==true
    pt1=gpsTrack(i,:);
    for j=i:length(gpsTrack)
        pt2=gpsTrack(j,:);
        horizDist=GPScalculateDistance(pt1(1:2),pt2(1:2));
        vertDist=abs(pt1(3)-pt2(3));
        dist=sqrt(horizDist^2+vertDist^2);
        if (dist<(distThreshold+eps) && dist>(distThreshold-eps) && j~=length(gpsTrack))
            i=j;
            trueTrack(k,:)=gpsTrack(j,:);
            k=k+1;
            break
        end
        if j==length(gpsTrack) % if we've reached the end
            bool=false;
        end
    end
end
trueTrack(end+1,:)=gpsTrack(end,:); % set the last waypoint to the end regardless of distance

%%
% size(trueTrack)
figure(1); plot(latitude,longitude); hold on; plot(trueTrack(:,1),trueTrack(:,2),'r*')
xlabel('Latitude'); ylabel('Longitude'); title('Flight path')
% figure(2); plot(latitude,altitude); hold on; plot(trueTrack(:,1),trueTrack(:,3),'r*')

autocontinue=1; % always autocontinue
heightOffset=20; % Meters above the ground we want to fly
trueTrack(2:end,3)=trueTrack(2:end,3)+heightOffset;

delete('RailWpPath.txt'); % so I don't keep writing on top
fid=fopen('RailWpPath.txt','at');
fprintf(fid,'QGC WPL 110\n');
for i=1:size(trueTrack,1) % note: wp's start indexing from 0
    index=i-1;
    if index==0 % home waypoint
        currentWp=1; coordFrame=0; command=16; param1=0; param2=0; param3=0; param4=0;
        trueTrack(i,3)=GPS(2,10); % Set home altitude to be absolute (as MP does it) % note 2
    elseif index==1 % auto takeoff
        currentWp=0; coordFrame=3; command=22; param1=0; param2=0; param3=0; param4=0;
    elseif index==size(trueTrack,1)-1 % land
        currentWp=0; coordFrame=3; command=21; param1=0; param2=0; param3=0; param4=0;
    else
        currentWp=0; coordFrame=3; command=82; param1=3000; param2=0; param3=1; param4=0; % not sure why param3=1, online says empty
    end
    fprintf(fid,'%i\t%i\t%i\t%i\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%i\n',...
        index,currentWp,coordFrame,command,param1,param2,param3,param4,...
        trueTrack(i,1),trueTrack(i,2),trueTrack(i,3), autocontinue);
end
fclose(fid);

% WP format
%QGC WPL <VERSION>
%<INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2>...
%<PARAM3> <PARAM4> <PARAM5/X/LONGITUDE> <PARAM6/Y/LATITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>

% WP 0 is home and has command 16, and current wp 1
% WP 1 is auto-takeoff
% WP last is land

% Relevant commands see (https://pixhawk.ethz.ch/mavlink/)
% 16: waypoint_nav, also used for takeoff
% 17: loiter_unlim
% 20: RTL
% 21: Land
% 22: Takeoff
% 80 or 201?: ROI
% 82: Spline, param1 is amount of time to stay at each wp


% Averaging three gps tracks

% %%
% % file1=load(trackLog1); % can load a .mat file this way?
% file1=load('/Users/gcharalambides/Desktop/helmet bedform park matlab rails files/2014-11-06 16-41-41 - path 1 i think.log-1147.mat');
% file2=load('/Users/gcharalambides/Desktop/helmet bedform park matlab rails files/2014-11-06 16-52-56 - path 2.log-1240.mat');
% file3=load('/Users/gcharalambides/Desktop/helmet bedform park matlab rails files/2014-11-06 16-58-18 - path 3, with laptop.log-1174.mat');
% 
% vars1=file1.GPS;
% vars2=file2.GPS;
% vars3=file3.GPS;
% 
% 
% track1=[vars1(2:end,7),vars1(2:end,8),vars1(2:end,9)]; % store lat/lon/alt
% track2=[vars2(2:end,7),vars2(2:end,8),vars2(2:end,9)]; % store lat/lon/alt
% track3=[vars3(2:end,7),vars3(2:end,8),vars3(2:end,9)]; % store lat/lon/alt
% 
% figure(1);
% plot(track1(:,1),track1(:,2),track2(:,1),track2(:,2),track3(:,1),track3(:,2));
% legend('track 1', 'track 2', 'track 3')
% title('gps tracks of the various tracks')
% 
% % figure(2); % comparing the altitudes of each track
% 
% 
% 
% 
% % for each point in track 1, find the closest point in the other tracks and
% % avg altitudes
% altVec=zeros(length(track1),3);
% for i=1:length(track1)
%     point1=[track1(i,1),track1(i,2)];
% %     dist=10^5; % initialize distance
%     shortestDist=10^5;
%     for j=1:length(track2);
%         point2=[track2(j,1),track2(j,2)];
%         dist=GPScalculateDistance(point1,point2);
% %         [dist, j]
%         if (dist<shortestDist)
%             shortestDist=dist;
%             jShortest=j;
%         end
%     end
%     altVec(i,1:2)=[track1(i,3),track2(jShortest,3)];
% end
%         
% %     dist=10^5; % initialize distance
% for i=1:length(track1)
%     point1=[track1(i,1),track1(i,2)];
%     shortestDist=10^5;
%     for k=1:length(track3);
%         point2=[track3(k,1),track3(k,2)];
%         dist=GPScalculateDistance(point1,point2);
%         if (dist<shortestDist)
%             shortestDist=dist;
%             kShortest=k;
%         end
%     end    
%     altVec(i,:)=[track1(i,3), track2(jShortest,3), track3(kShortest,3)];
% end
% avgAlt=mean(altVec');
% avgAlt=avgAlt'; % convert back to column vector
% 
% avgdTrack=[track1(:,1),track1(:,2),avgAlt];
