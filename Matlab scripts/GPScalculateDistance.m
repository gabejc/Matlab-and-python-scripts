% calculates the distance between two GPS coordinates using the
% haversine formula
function d=GPScalculateDistance(Point1,Point2)
    phi1=Point1(1)*pi/180;
    phi2=Point2(1)*pi/180;
    dphi=phi2-phi1;
    dlambda=(Point2(2)-Point1(2))*pi/180;
    a=sin(dphi/2)^2+cos(phi1)*cos(phi2)*sin(dlambda/2)^2;
    c=2*atan2(sqrt(a),sqrt(1-a));
    d=6378*c; % radius of earth is 6378km
    d=d*10^3;
end

% phi1=DesiredLat(i)*pi/180;
% phi2=DesiredLat(i+1)*pi/180;
% dphi=phi2-phi1;
% dlambda=(DesiredLong(i+1)-DesiredLong(i))*pi/180;
% a=sin(dphi/2)^2+cos(phi1)*cos(phi2)*sin(dlambda/2)^2;
% c=2*atan2(sqrt(a),sqrt(1-a));
% d(i)=R*c;



% (http://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters)
% function d=GPScalculateDistance(lat1, lon1, lat2, lon2)
%     R = 6378.137;
%     dLat = (lat2 - lat1) * pi / 180;
%     dLon = (lon2 - lon1) * pi / 180;
%     a = sin(dLat/2) * sin(dLat/2) + ...
%     cos(lat1 * pi / 180) * cos(lat2 * pi / 180) * ...
%     sin(dLon/2) * sin(dLon/2);
%     c = 2 * atan2(sqrt(a), (1-a));
%     d = R * c;
% end