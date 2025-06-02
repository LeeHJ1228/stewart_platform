function b_rot = stewartrot (theta, phi, psi, b_global)
% This function applies three rotation matrices to determine the value
% of b in the rotated reference frame

% rotation_matrix = [cosd(psi)*cosd(phi)
%     cosd(psi)*sind(phi)*sind(theta)-sind(psi)*cosd(theta)
%     cosd(psi)*sind(phi)*cosd(theta)+sind(psi)*sind(theta)
%     0; ...
%     sind(psi)*cosd(phi)
%     sind(psi)*sind(phi)*sind(theta)+cosd(psi)*cosd(theta)
%     sind(psi)*sind(phi)*cosd(theta)-cosd(psi)*sind(theta)
%     0; ...
%     -sind(phi)
%     cosd(phi)*sind(theta)
%     cosd(phi)*cosd(theta)
%     0; ...
%     0  0  0  1];

rotation_matrix = [ ...
    cosd(psi)*cosd(phi),  cosd(psi)*sind(phi)*sind(theta) - sind(psi)*cosd(theta),  cosd(psi)*sind(phi)*cosd(theta) + sind(psi)*sind(theta),  0; ...
    sind(psi)*cosd(phi),  sind(psi)*sind(phi)*sind(theta) + cosd(psi)*cosd(theta),  sind(psi)*sind(phi)*cosd(theta) - cosd(psi)*sind(theta),  0; ...
   -sind(phi),            cosd(phi)*sind(theta),                                    cosd(phi)*cosd(theta),                                    0; ...
    0,                    0,                                                        0,                                                        1];

b_rot = rotation_matrix * b_global;