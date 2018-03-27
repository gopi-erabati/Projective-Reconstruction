%% lab1
clc; close all; clear all;

%read the 3D object
ptCloud = pcread('hulk.ply');
%ptCloud = pcdownsample(ptCloud, 'gridAverage' , 1); % to downsample
showPointCloud(ptCloud);
axis([-5 ptCloud.XLimits(2) -5 ptCloud.YLimits(2) -5 ptCloud.ZLimits(2)]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D scene and projections onto two image planes');
hold on;

% take the 3dPoints
X = ptCloud.Location;

%% simulate sterovision system

%% camera 1
fx1 = 1 ; fy1 = 1; fc1 = 50; u01 = 0; v01 = 0; % pixel density and focal length
kx1 = fc1 * fx1 ; ky1 = fc1 * fy1;
K1 = [kx1 0 u01;
    0 ky1 v01;
    0 0 1]; % intrinsic parameters
normCam1 = [eye(3) zeros(3,1)]; %normalsised camera matrix
R1 = eye(3);
t1 = [ 0*fc1 0 0]'; % to put camera 1 at origin Extrinsic parmeters

%Camera Matric of camera1
P1 = K1 * normCam1 * [R1 t1; zeros(1,3) 1];

% to draw camera1 on plot
Rc1 = [1 0 0 ; 0 1 0; 0 0 1];
tc1 = [0 0 0]';
Draw3DCamera([Rc1 tc1], 'C1');

%% camera 2
fx2 = 1; fy2 = 1; fc2 = 50; u02 = 0; v02 = 0; % pixel density and focal length
kx2 = fc2 * fx2 ; ky2 = fc2 * fy2;
K2 = [kx2 0 u02;
    0 ky2 v02;
    0 0 1]; % intrinsic parameters
normCam2 = [eye(3) zeros(3,1)]; %normalsised camera matrix
R2 = eye(3);
txx = 300; % trsnalatoin along x
t2 = [ -txx 0 0]'; %to trsnalate camera by 400 Extrinsic parameters

% the above '-' is becasue "THAT IS TRANSLATION OF WORLD COOROINATE ORIGIN
% WITHRESPECT TO CAMERA2"

%Camera Matric of camera2
P2 = K2 * normCam2 * [R2 t2; zeros(1,3) 1];

% to draw camera2 on plot
Rc1 = [cos(pi/4) sin(pi/4) 0 ; -sin(pi/4) cos(pi/4) 0; 0 0 1];
tc1 = [txx 0 0]';
Draw3DCamera([Rc1 tc1], 'C2', 'b');


%% Compute resulting images

%% camera1

% projections of scene onto camera planes
x1 = projection(P1, X');

C1 = camstruct('f', 50); % to construct camera structure to be used by imagept2plane function
% convert image pts to a plane
pt1 = imagept2plane(C1, x1(1:2,:), [0 0 fc1], [0 0 1]);

%define a plane at focal length fc1
transparency = 0.3;    %mostly clear
% XL = [10 200];
% YL = [10 200];
min1 = min(pt1(1:2,:),[],2);
max1 = max(pt1(1:2,:),[],2);
XL = [min1(1,1)-20 max1(1,1)+20];
YL = [min1(2,1)-20 max1(2,1)+20];
zpos = 50;
patch(XL([1 2 2 1 1]), YL([1 1 2 2 1]), zpos * ones(1,5), [0 .2 .7], 'FaceAlpha', transparency);


% to plot the points on image plane
plot3(pt1(1,:),pt1(2,:), pt1(3,:),'.','MarkerSize',10, 'LineWidth', 1,'Color','r');

% UNCOMMENT below to show the projection lines
% for i = 1:size(X,1)
%     plot3([X(i,1) 0],  [X(i,2) 0],[X(i,3) 0], 'LineWidth', 0.00002, 'Color', 'g');
% end

%% camera 2

%projections of scene onto camera planes
x2 = projection(P2, X');

C2 = camstruct('f', 50, 'P', [txx;0;0]); % to construct camera structure to be used by imagept2plane function
% convert image points to a  a plane
pt2 = imagept2plane(C2, x2(1:2,:), [0 0 fc1], [0 0 1]);

%define a plane at focal length 1
transparency = 0.3;    %mostly clear
% XL = [280 450];
% YL = [20 200];
min1 = min(pt2(1:2,:),[],2);
max1 = max(pt2(1:2,:),[],2);
XL = [min1(1,1)-20 max1(1,1)+20];
YL = [min1(2,1)-20 max1(2,1)+20];
zpos = 50;
patch(XL([1 2 2 1 1]), YL([1 1 2 2 1]), zpos * ones(1,5), [0 .2 .7], 'FaceAlpha', transparency);

% to plot the points on image plane
plot3(pt2(1,:),pt2(2,:), pt2(3,:),'.','MarkerSize',10, 'LineWidth', 1,'Color','b');

% UNCOMMENT below to show the projection lines
% for i = 1:size(X,1)
%     plot3([X(i,1) txx],  [X(i,2) 0],[X(i,3) 0], 'LineWidth', 0.00002, 'Color' , 'c');
% end

hold off;

%% to create images with the 2D Points

%create images
image1 = createImage(pt1(1:2,:));
image2 = createImage(pt2(1:2,:));

%% Fundamental matrix from known parameters
tx = [-txx 0 0]; % trsnalation of world cordinate with respect to camera2
tx = star(tx); % skew symmetric matrix

R = eye(3);

% Fundamental matrix
F = inv(K1') * tx * R * inv(K2);

disp('***********************************************************');
disp('Computed Fundamental matrix from known camera parameters');
disp(F);
disp('***********************************************************');


%% Estimate the fundamental matrix from the two images

points2d = [x2(1,:)' x2(2,:)' x1(1,:)' x1(2,:)'];

[M,T1,T2]=normalHartley(points2d'); %hartley normalizatiomn of points

% %Using 7 point method
% disp('***********************************************************');
% disp('Method: seven points to estimate fundamnetal matrix')
% tic
% F7p=funmat7p(M(:,1:7));
% time7p=toc;
% F7p=T1'*F7p*T2;
% F7p=F7p./norm(F7p);
% disp('Funamental Matrix')
% disp(F7p)
% disp(sprintf('Rank-2: %d',rank(F7p)==2))
% disp(sprintf('Time: %f',time7p))
% disp('***********************************************************');

% Using RANSAC method
disp('***********************************************************');
disp('Method: RANSAC from Salvi Toolbox')
tic
[FRANSAC,wRANSAC]=funmatRANSAC(M,8,0.99,0.25);
timeRANSAC=toc;
FRANSAC=T1'*FRANSAC*T2;
FRANSAC=FRANSAC./norm(FRANSAC);
disp('Funamental Matrix')
disp(FRANSAC)
disp(sprintf('Rank-2: %d',rank(FRANSAC)==2))
% disp(sprintf('Time: %f',timeRANSAC))
disp('***********************************************************');


% fundamental matrix using MATLAB RANSAC code
disp('Method: RANSAC from MATLAB Toolbox')
tic
fRANSAC = estimateFundamentalMatrix(x2(1:2,:)',...
    x1(1:2,:)','Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);
tRANSAC = toc;
disp('Funamental Matrix')
disp(fRANSAC);
disp(sprintf('Rank-2: %d',rank(fRANSAC)==2))
% disp(sprintf('Time: %f',tRANSAC))
disp('***********************************************************');


%% to compute error by drawing epipolar lines to check Fundamental matrices

% show image1
figure,
subplot(1,2,1)
imshow(image1, []);
title('Image : Camera at origin');

% to draw epipolar lines for image 1 on image2 and check for error
% hold on;
error = computeErrorEpipolar( x1, x2, FRANSAC, 'left', 512, 'drawNo'); % 'drawYes' to draw epipolar lines
disp('Error in corresponding points of image 1 lying on epipolar lines generated by points of image 2')
disp(num2str(error));
disp('***********************************************************');
% hold off;

% show image2
subplot(1,2,2)
imshow(image2, []);
title(['Image : Camera translated by', num2str(txx), ' mm']);

% to draw epipolar lines for image 2 on image1
hold on;
error = computeErrorEpipolar( x1, x2, FRANSAC, 'right', 512 , 'drawYes'); % 'drawYes' to draw epipolar lines
disp('Error in corresponding points of image 2 lying on epipolar lines generated by points of image 1')
disp(num2str(error));
disp('***********************************************************');
hold off;

%% 3D reconstruction from Canonical representation using Trinagulation

% calculate epipole of camera2
[U D V] = svd(FRANSAC);

epipole2 = V(:,end);

% skew symmetric of wpipole
epipole2Skew = star(epipole2);

% calculate camera matrices in  canocial represntation
P1_canonical = [eye(3) zeros(3,1)];
v = [0 0 0]';
lambda = 1;
P2_canonical = [epipole2Skew * FRANSAC + epipole2 * v', lambda.*epipole2];

% triangulation
Xproj = f_intersection( P2_canonical , P1_canonical ,x2(1:2,:), x1(1:2,:) ); % projective

% for pure translational motion the camera matrices are define by
P1_trans = [eye(3) zeros(3,1)];
P2_trans = [eye(3) epipole2];

%triangulation
Xaff = f_intersection( P2_trans , P1_trans ,x2(1:2,:), x1(1:2,:) ); % Affine

figure
plot3(Xaff(1,:)', Xaff(2,:)', Xaff(3,:)','.','MarkerSize',10, 'LineWidth', 1,'Color','r');
title('Affine recosntruction : Translated camera');

% Xmetric = f_intersection( P1 , P2 ,x1(1:2,:), x2(1:2,:) ); %metric reconstruction

figure
plot3(Xproj(1,:)', Xproj(2,:)', Xproj(3,:)','.','MarkerSize',10, 'LineWidth', 1,'Color','r');
title('Projective reconstruction from canonical forms : kindly zoom in to see clearly');


%% compute 2d points from computed 3d points using canonical camera matrices

% camera 1
% projections of scene onto camera planes
x1est = projection(P1_canonical, Xproj);

C1new = projmatrix2camstruct(P1_canonical);

% convert image pts to a plane
pt1est = imagept2plane(C1new, x1est(1:2,:), [0 0 1], [0 0 1]);

% camera 2
% projections of scene onto camera planes
x2est = projection(P2_canonical, Xproj);

C2new = projmatrix2camstruct(P1_canonical);

% convert image pts to a plane
pt2est = imagept2plane(C2new, x2est(1:2,:), [0 0 1], [0 0 1]);

image1 = createImage(pt1est(1:2,:));
image2 = createImage(pt2est(1:2,:));

% show image1
figure,

subplot(1,2,1)
imshow(image1, []);


subplot(1,2,2)
imshow(image2,[]);
mtit('2d projections from 3d points obtained by projective transformation');

%% compute residual error

% for camera 1
xerror1 = x1 - x1est;

error1 = 0;
for i = 1 : size(xerror1,2)
    error1 = error1 + norm(xerror1(:,i));
end

error1 = error1/size(xerror1,2); % residual error

disp('The Residual error (2D error) for image 1 is')
disp(num2str(error1))
disp('***********************************************************');

% for camera 2
xerror2 = x2 - x2est;

error2 = 0;
for i = 1 : size(xerror2,2)
    error2 = error2 + norm(xerror2(:,i));
end

error2 = error2/size(xerror2,2);

disp('The Residual error (2D error) for image 2 is')
disp(num2str(error2))
disp('***********************************************************');