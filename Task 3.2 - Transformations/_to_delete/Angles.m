function angles = acc_to_angles(a)
% a      : [ax; ay; az] aksellerometer [m/s^2]. z peker ned, saa az = -g i hvile
% angles : [pitch; elevation] in rad, 2x1

persistent last_angles
if isempty(last_angles)
    last_angles = [0; 0];
end

ax = a(1);  ay = a(2);  az = a(3);

% Under oppstart er alle IMU-maalinger 0. Da gir atan2(-0,-0) = -pi, ikke 0,
% saa vi beholder forrige gyldige vinkel til det kommer en ekte maaling.
if ay ~= 0 || az ~= 0 || ax ~= 0
    p = atan2(-ay, -az);                   % (2.19a)
    e = atan2(ax, sqrt(ay^2 + az^2));      % (2.19b)
    last_angles = [p; e];
end

angles = last_angles;
end
