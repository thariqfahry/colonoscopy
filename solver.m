 %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%5
%%%%%%%%%%%%%%555
%%%%%%%%%%%%%55
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%this one is the SCISSOR

colon = csvread('smooth_coordinates.csv',1,0);
%colon = colon/100;

offset_x = 0;
offset_y = 0;

colonsize = size(colon);
z = zeros(colonsize(1,1),1);

colon = [colon(:,1)+(z+offset_x),colon(:,2)+z+offset_y];

hr_colon_small = [colon,z];


% Start with a blank rigid body tree model.
robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',5);

robot2 = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',5);

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%Specify parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stepSize = 1.8/4;
%Specify base offsets%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r0_x = 76;
r0_y = -53.5;

r1_x = 167.5;
r1_y = -50.5;

%Specify arm lengths%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L1 = 142;           % box section
L2 = 194;           % forearm
%L3 = 171;
%L4 = 171;

%Specify if arms are symmetrical%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L3 = L1;
L4 = L2;

%..or a parallelogram%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%L3 = L2;
%L4 = L1;

%Specify joint limits%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
JL00 = [pi/3, 3*pi/2];
JL01 = [-pi , pi];

JL10 = [-pi, pi];
JL11 = [-pi, pi];

%...or turn off joint limits%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%JL00 = [-pi, pi];
%JL01 = [-pi , pi];

%JL10 = [-pi, pi];
%JL11 = [-pi, pi];



%Specify solver weights (unused for now)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
weights = [0, 0, 0, 1, 1, 0];
weights2 = [0, 0, 0, 1, 1, 0];

%Specify solver algorithm (if unspecified, defaults to BFGS)%%%%%%%%%%%%%%%
solvername = 'LevenbergMarquardt';

%Specify maximum solver iterations (default = 1500)
maxits = 1500;

%Specify till when to plot points%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotuntil = length(colon);
%%
% Add |'link1'| body with |'joint1'| joint.
body = robotics.RigidBody('link1');
joint = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([r0_x r0_y 0]));
joint.JointAxis = [0 0 1];
joint.PositionLimits = JL00;
body.Joint = joint;
addBody(robot, body, 'base');
%%
% Add |'link2'| body with |'joint2'| joint.
body = robotics.RigidBody('link2');
joint = robotics.Joint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
joint.PositionLimits = JL01;
body.Joint = joint;
addBody(robot, body, 'link1');

%%
% Add |'tool'| end effector with |'fix1'| fixed joint.
body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               second robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


body = robotics.RigidBody('link3');
joint = robotics.Joint('joint3', 'revolute');
setFixedTransform(joint,trvec2tform([r1_x r1_y 0]));
joint.JointAxis = [0 0 1];
joint.PositionLimits = JL10;
body.Joint = joint;
addBody(robot2, body, 'base');
%%
% Add |'link2'| body with |'joint2'| joint.
body = robotics.RigidBody('link4');
joint = robotics.Joint('joint4','revolute');
setFixedTransform(joint, trvec2tform([L3,0,0]));
joint.JointAxis = [0 0 1];
joint.PositionLimits = JL11;
body.Joint = joint;
addBody(robot2, body, 'link3');

%%
% Add |'tool'| end effector with |'fix1'| fixed joint.
body = robotics.RigidBody('tool2');
joint = robotics.Joint('fix2','fixed');
setFixedTransform(joint, trvec2tform([L4, 0, 0]));
body.Joint = joint;
addBody(robot2, body, 'link4');



%%
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
showdetails(robot)
showdetails(robot2)

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t = (0:0.2:40)'; % Time
count = colonsize(1,1);
points=hr_colon_small;
%% Inverse Kinematics Solution
% Use an |InverseKinematics| object to find a solution of robotic 
% configurations that achieve the given end-effector positions along the 
% trajectory. 

%%
% Pre-allocate configuration solutions as a matrix |qs|.

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

q1 = homeConfiguration(robot2);
ndof = length(q1);
qs1 = zeros(count, ndof);
%%
% Create the inverse kinematics solver. Because the _xy_ Cartesian points are the
% only important factors of the end-effector pose for this workflow, 
% specify a non-zero weight for the fourth and fifth elements of the 
% |weight| vector. All other elements are set to zero.
ik = robotics.InverseKinematics('RigidBodyTree', robot);
ik2 = robotics.InverseKinematics('RigidBodyTree', robot2);


%solver: change if problematic
ik.SolverAlgorithm = solvername;
ik2.SolverAlgorithm = solvername;

ik.SolverParameters.MaxIterations = maxits;
ik2.SolverParameters.MaxIterations = maxits;

%The first three elements correspond to the weights on the 
%error in orientation for the desired pose.
%The last three elements correspond to the
%weights on the error in xyz position for
%the desired pose.

endEffector = 'tool';
endEffector2 = 'tool2';

%%
% Loop through the trajectory of points to trace the circle. Call the |ik|
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.

qInitial = q0; % Use home configuration as the initial guess
qInitial1 = q1;

for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);    %%%%%%%   [configSol,solInfo] = ik(endeffector,pose,weights,initialguess)
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
    
    qSol = ik2(endEffector2,trvec2tform(point),weights2,qInitial1);
    % Store the configuration
    qs1(i,:) = qSol;
    % Start from prior solution
    qInitial1 = qSol;
    
    
end

fprintf("IK complete.\n")

timenow = ['solutions\MS_SCISSOR_Radians' ' ' datestr(datetime,31) ' ' '[' num2str(L1) ',' num2str(L2) ',' num2str(L3) ',' num2str(L4) ']' 'x' num2str(count) '.csv' ];
timenow = strrep(timenow,':','.');

qs = qs+[(deg2rad(0)),0];                               %adjustment by a constant angle if needed (not done)

qf = [qs(:,1),qs1(:,1)];
%csvwrite(timenow,qf);

step_matrix = -round(qf/(deg2rad(stepSize)));           %round and convert to steps

normalizer = [round(0/(stepSize)),round(0/(stepSize))]; %edit normalizer if needed (not done)
normalizer = step_matrix(1,:)-normalizer;               %store first coordinate in normalizer varibale

step_matrix = step_matrix-normalizer;                   %normalize first coordinate to 0,0

qe = -(step_matrix+normalizer)*deg2rad(stepSize);       %convert back to radians to compare error created by rounding  

qs(:,1) = qe(:,1);      %write "erroneous" data back into original solution matrices
qs1(:,1) = qe(:,2);



csvwrite('Angles.csv',step_matrix)
fprintf("Angles.csv written. \n")

ms_time = transpose([0:(7/(count-1)):7]); %#ok<NBRAK> % create time array for motion study

MS0 = [ms_time,rad2deg(qs(:,1))];
MS1 = [ms_time,90-(rad2deg(qs1(:,1)))];

MS0(1,:) = 0;
MS1(1,:) = 0;

csvwrite('solutions\_MS0_MotionStudy.csv',MS0);
csvwrite('solutions\_MS1_MotionStudy.csv',MS1);

fprintf("MS CSV written. \n")

%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot 
% configuration. Also, plot the desired trajectory.

%%
% Show the robot in the first configuration of the trajectory. Adjust the 
% plot to show the 2-D plane that circle is drawn on. Plot the desired 
% trajectory.

if 1==1
figure
show(robot,qs(1,:)');
show(robot2,qs1(1,:)');

view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(1:plotuntil,1),points(1:plotuntil,2),'k')
axis([-70 300 -70 300])

filename = 'output.gif';

%%
% Set up a <docid:robotics_ref.bu31fh7-1 Rate> object to display the robot 
% trajectory at a fixed rate of 15 frames per second. Show the robot in
% each configuration from the inverse kinematic solver. Watch as the arm
% traces the circlular trajectory shown.
framesPerSecond = 180;
r = robotics.Rate(framesPerSecond);

for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    
    show(robot2,qs1(i,:)','PreservePlot',false);
    drawnow

    waitfor(r);
end
end