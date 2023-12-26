%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about eventual warnings!
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;
figure;
% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Function that gives the transformation from <base> to <e-e>, given a
% configuration of the manipulator
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7');%DO NOT EDIT 

% Tool frame definition
%lenght measurements converted in meters
 eOt = [0, 0, 0.2104];
 
 Phi=deg2rad(-44.98);
 eRt=[cos(Phi), -sin(Phi), 0;
      sin(Phi), cos(Phi),  0;
      0,        0,         1];
 eTt = [  eRt, eOt';
        0, 0, 0, 1];
 bTt = bTe*eTt;         

 
% Goal definition 
%as defined by specs
%measurements in meters
bOg = [0.55, -0.3, 0.2]';
%rotation around y-axis of EE initial frame of pi/6
theta = pi/6;
eRg= [cos(pi/6), 0,  sin(pi/6);
       0,        1,      0    ;
     -sin(pi/6), 0, cos(pi/6) ];
bRg=bTe(1:3,1:3)*eRg;


% Switch between the two cases (with and without the tool frame)
tool = true; % change to true for using the tool
if tool == true
    %bTg = ...; % if controlling the tool frame
    tRg = eRg;             %transformation matrix is the same
    
    bRt = bTt(1:3,1:3);
    bRg = bRt * tRg;
    bTg = [bRg,bOg; 0, 0, 0, 1];

else
    %bTg = ...; % if controlling the ee frame
    bTg=[bRg,bOg;
         0,0,0,1];
end   

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
lin_err = zeros(3,1);
ang_err = zeros(3,1); 
% Start the inverse kinematic control  
q = q_init;

%% Simulation Loop
for i = t
    
    if tool == true %compute the error between the tool frame and goal frame
        
        eRt=[cos(Phi), -sin(Phi), 0;
              sin(Phi), cos(Phi),  0;
              0,        0,         1];
        eTt = [  eRt, eOt';
                0, 0, 0, 1];
        bTt = bTe*eTt;   

        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        %bJt = ... 
        lin_err = bOg - bTt(1:3,4);
        bRt = bTt(1:3,1:3);
        [theta, v]=ComputeInverseAngleAxis(bRt'*bRg);
        ang_err = bRt*(theta*v)';
        
    else % compute the error between the e-e frame and goal frame
        eRg= [cos(theta), 0,  sin(theta);
            0,        1,      0    ;
            -sin(theta), 0, cos(theta) ];
        bRg=bTe(1:3,1:3)*eRg;
        bTg=[bRg,bOg;
            0,0,0,1];
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        %the linear error is the position vector from the EE frame to the
        %goal frame
        lin_err = bOg-bTe(1:3,4); 
        %the angular error is the angle between the EE frame and the goal
        %frame around the axix-vector
        bRe=bTe(1:3,1:3);
        [theta, v]=ComputeInverseAngleAxis(bRe'*bRg);
        ang_err= bRe*(theta*v)';
    end
       
    %% Compute the reference velocities
    %no velocities of goal frame
    v_ref= linear_gain*lin_err;
    omega_ref= angular_gain*ang_err;
    
   
    %% Compute desired joint velocities
    x_dot=[omega_ref;v_ref];
    q_dot=pinv(bJe)*x_dot;
    
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    %switch visuals to off for seeing only the frames
    show(model.franka,[q',0,0],'visuals','on');
    hold on
    if tool == true
        %set the window size of the figure to "full-screen" for a better visualization
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(x_dot) < 0.001)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    hold off;
end
