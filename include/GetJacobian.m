function [J, bTi] = GetJacobian(biTei, bTe, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
%
% Output:
% - J: end-effector jacobian matrix
numberOfLinks = size(jointType);
J = zeros(6,numberOfLinks);

bTi = zeros(4,4,numberOfLinks);
for i = 1:numberOfLinks
    bTi(:,:,i) = GetTransformationWrtBase(biTei,i);
end

for i = 1:numberOfLinks

%angular jacobian
    
    if jointType(i) == 0        %if link i is revolute

        ki = bTi(1:3,3,i);      %take the rotational axis (z-axis of the joint frame wrt base)
        string=sprintf('btd%d',i);
        disp(string);disp(bTi(:,:,i));
        J(1:3,i) = ki; 
    
    else                        %if link i is prismatic

        J(1:3,i) = zeros(3,1);  %set to zero
        
    end

%linear jacobian
    
    if jointType(i) == 0        %if joint i is revolute

        ki = bTi(1:3,3,i);
        norm(ki)%rotational axis
        bri = bTi(1:3,4,i);     %vector from base to joint i
        bre = bTe (1:3,4);      %vector from base to EE
        ire = bre - bri;        %vector from joint i to EE
        string1=sprintf('%dre',i);
        disp(string1);disp(ire);
        J(4:6,i) = cross(ki,ire);    
    
    else                        %if joint is prismatic

        ki = bTi(1:3,3);        %prismatic axis
        J(4:6,i) = ki;
        
    end
end
