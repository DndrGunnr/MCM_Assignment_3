%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
addpath('include')
% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];

%% Compute direct geometry

%direct geometry given the vector of joint configuration q
iTj_q = GetDirectGeometry(q, geom_model, linkType);

% Compute the transformation of EE w.r.t. the base
bTe = GetTransformationWrtBase(iTj_q,numberOfLinks);

% Computing end effector jacobian 
j = GetJacobian(iTj_q, bTe, linkType, numberOfLinks);

%% 1.1.1
q1=[1.8,1.8,1.8,1.8,1.8,1.8,1.8];

iTj_q1 = GetDirectGeometry(q1, iTj_q, linkType);
bTe = GetTransformationWrtBase(iTj_q1,numberOfLinks);
j1 = GetJacobian(iTj_q1, bTe, linkType, numberOfLinks);
%% 1.1.2
q2 = [0.3, 1.4, 0.1, 2.0, 0, 1.3, 0];

iTj_q2 = GetDirectGeometry(q2, iTj_q, linkType);
bTe = GetTransformationWrtBase(iTj_q2,numberOfLinks);
j2 = GetJacobian(iTj_q2, bTe, linkType, numberOfLinks);
%% 1.1.3
q3 = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0];

iTj_q3 = GetDirectGeometry(q1, iTj_q, linkType);
bTe = GetTransformationWrtBase(iTj_q3,numberOfLinks);
j3 = GetJacobian(iTj_q3, bTe, linkType, numberOfLinks);
%% 1.1.4
q4 = [1, 1, 1, 1, 1, 1, 1];

iTj_q4 = GetDirectGeometry(q4, iTj_q, linkType);
bTe = GetTransformationWrtBase(iTj_q4,numberOfLinks);
j4 = GetJacobian(iTj_q4, bTe, linkType, numberOfLinks);