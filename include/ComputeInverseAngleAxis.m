function [theta,v] = ComputeInverseAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'v' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % eig()
    % find()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.
    
    % Check matrix R to see if its size is 3x3
    if size(R) == [3,3]
        
        % Check matrix R to see if it is orthogonal
        tolerance=1e-4;
        if (abs(R*R.'-eye(3))<(ones(3)*tolerance))
            % Check matrix R to see if it is proper: det(R) = 1
            
            if abs(det(R)-1)<1*tolerance
                % Compute the angle of rotation
                theta=acos((trace(R)-1)/2);
                %if the sin(theta) is equal to zero, I cannot use the vex
                %solution
                
               if  sin(theta)==0
                    % Calculate eigenvalues and eigenvectors of R
                    [V,D]=eig(R);
                    eigenvalues=diag(D);
               
                    % Compute the axis of rotation
                    indx=find(abs(eigenvalues-1)<tolerance);
                    v=V(:,indx);
                    %choose correct axis unit vector
                    if abs(ComputeAngleAxis(theta,-v)-R)<ones(3)*1e-4
                       v=-v;
                    end
                else
                    R1 = (R-transpose(R))/2;
                    vex = [R1(3,2) R1(1,3) R1(2,1)];
                    v = vex / sin(theta);
                end

               

            else
                err('DETERMINANT OF THE INPUT MATRIX IS NOT 1')  
            end
        else
            err('NOT ORTHOGONAL INPUT MATRIX')
        end
    else
       err('WRONG SIZE OF THE INPUT MATRIX')
    end
end

