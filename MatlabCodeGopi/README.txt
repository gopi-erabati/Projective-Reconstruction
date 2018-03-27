To run the code:

1. Change your current working directory of MATLAB to 'MatlabCodeGopi' folder.

2. Run the 'lab1_v1.m' file which executes all the tasks of lab1.

3. You can view the results in command window and pop-up figures.

We can get information of all files by 'help <filename>' 

The other m0files are for suporting the execution of tasks in lab_v1.m file.

camstruct - to construct camera structure which is used by imagept2plane function
computeErrorEpipolar - used to draw epipolar lines and calcualte error to check fundamental matrix
createImage - a function to create image from 2d points
Draw3DCamera - Draws the camera position and axis rotation in the 3D scene.
f_intersection - triangulation function
funmat7p - fundamental matrix using 7 points method
funmatRANSAC - fundamnetal matrix using RANSAC method
HomogeneousCoordinates - function to convert to homogenous cordinates
imagept2plane - Project image points to a plane and return their 3D locations
mtit - used to position a main title for subplots
normalHartley - to normalise points
pointToLine - distnace between point and line
projection - this function to computer projections of 3d points onto 2d using camera parameters
projmatrix2camstruct - Projection matrix to camera structure
rq3 - RQ decomposition of 3x3 matrix
star - to form skerw symmetric matrix

Cheers!