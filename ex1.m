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
bTe = GetTransformationWrtBase(geom_model,numberOfLinks);

% Computing end effector jacobian 
j = GetJacobian(iTj_q, bTe, linkType, numberOfLinks);


%ki = T(1:3,3);