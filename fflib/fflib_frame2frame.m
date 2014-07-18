% Converts a position or velocity from a frame B to a frame A. 
% The inputs are:
%   - Position or velocity in frame B
%   - Position or velocity of frame B in frame A
%   - Attitude quaternion of frame B in frame A
% Output: 
%   - Position or velocity in frame A
%
% Application example: let frame A be the NED frame, frame B an aircraft's
% body frame; you could use this function to convert the position of the
% pilot in the body frame to the NED frame by calling it function as
% follows:
% pPilot_NED = frame2frame(pPilot_bodyframe, pAircraft_NED, qAttitudeAircraft_NED);
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function pv_A = fflib_frame2frame(pv_B, pvB_A, qAttitudeB_A)
    % Rotate:
    pv_A = quatrotate(quatconj(qAttitudeB_A'), pv_B')';
    % Translate:
    pv_A = pvB_A + pv_A;
end
