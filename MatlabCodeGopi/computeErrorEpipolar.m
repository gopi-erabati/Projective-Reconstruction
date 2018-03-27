function error = computeErrorEpipolar( x1, x2, F, lorR , imageWidth, drawYesNo)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % this function is used to calculate the average error between points in
% % right image and the epipolar lines to check the accuracy of Fundamental
% % matrix F
%
% Inputs
%     x1              2d points on left image
%     x2              2d points on right image
%     F               Fundamental matrix
%     lorR            left or right image error
%     imageWidth      width of image displayed
%       drawYesNo       whether to draw epipolar lines or not
%
% Output
%     error           Average error (distance) between points and epipolar lines
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

error = 0;

% to display that we are drawing epipolar lines on command window
if (strcmp(drawYesNo, 'drawYes'))
            disp('Drawing Epipolar Lines...');
end

if (strcmp(lorR,'right'))
    % convert to homogenous cordinates
    x1 = HomogeneousCoordinates(x1,'2D');
    
    for i=1:size(x1,2)
        
        % find epipolar line
        l = F * x1(:,i);
        
        %ax + by + c = 0 ==> y = (-ax - c) / b
        %x coordinate in the image
        x = [0 imageWidth];
        y = (-l(1)*x - l(3)) / l(2);
        
        % to draw epipolar lines
        if (strcmp(drawYesNo, 'drawYes'))
            
            ci = mod(i,6)+1;
            
            colors = ['r','c','y','g','m','k'];
            
            p1 = plot(x, y, colors(ci));
            p1.Color(4) = 0.05;
        end
        
        error = error + pointToLine([x2(1:2,i)' 0], [x(1) y(1) 0], [x(end) y(end) 0]); % distance from point to line
        
    end
    error = error/size(x1,2);
else
    % convert to homogenous cordinates
    x2 = HomogeneousCoordinates(x2,'2D');
    
    for i=1:size(x2,2)
        
        % find epipolar line
        l = F' * x2(:,i);
        
        %ax + by + c = 0 ==> y = (-ax - c) / b
        %x coordinate in the image
        x = [0 imageWidth];
        y = (-l(1)*x - l(3)) / l(2);
        
        % to draw epipolar lines
        if (strcmp(drawYesNo, 'drawYes'))
            
            ci = mod(i,6)+1;
            
            colors = ['r','c','y','g','m','k'];
            
            p1 = plot(x, y, colors(ci));
            p1.Color(4) = 0.5;
        end
        
        error = error + pointToLine([x1(1:2,i)' 0], [x(1) y(1) 0], [x(end) y(end) 0]); % distance from point to line
        
    end
    error = error/size(x2,2);
end


end

