function [C, b] = computeConstraint(order, m, k_r, k_psi, t, keyframe)

n = 4;                              %State number

%Waypoint constraints
C1 = zeros(2*m*n,n*(order+1)*m);
b1 = zeros(2*m*n,1);
computeMat = eye(order+1);          %Required for computation of polynomials
for i=1:m
    waypoint = keyframe(:,i);       %Waypoint at t(i)
    
    if(i==1)                        %Initial and Final Position
        %Initial
        values = zeros(1,order+1);
        for j=1:order+1
            values(j) = polyval(computeMat(j,:),t(i));
        end
        
        for k=1:n
            c = zeros(1,n*(order+1)*m);
            c( ((i-1)*(order+1)*n+(k-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
            C1(k,:) = c;
        end
        b1(1:n) = waypoint;
        
        %Final
        for j=1:order+1
            values(j) = polyval(computeMat(j,:),t(m+1));
        end
        
        for k=1:n
            c = zeros(1,n*(order+1)*m);
            c( ((m-1)*(order+1)*n+(k-1)*(order+1)+1) : ((m-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
            C1(k+n,:) = c;
        end
        b1(n+1:2*n) = waypoint;
        
    else
        %Elsewhere
        values = zeros(1,order+1);
        for j=1:order+1
            values(j) = polyval(computeMat(j,:),t(i));
        end
        
        for k=1:n
            c = zeros(1,n*(order+1)*m);
            c( ((i-2)*(order+1)*n+(k-1)*(order+1)+1) : ((i-2)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
            C1(k+2*n*(i-1),:) = c;
        end
        b1(2*n*(i-1)+1:2*n*(i-1)+n) = waypoint;
        
        for k=1:n
            c = zeros(1,n*(order+1)*m);
            c( ((i-1)*(order+1)*n+(k-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
            C1(k+2*n*(i-1)+n,:) = c;
        end
        b1(2*n*(i-1)+n+1:2*n*(i-1)+2*n) = waypoint;
        
    end
    
end


% Continuity of derivatives
% b2 = zeros(2*m*(n-1)*k_r,1);
% C2 = zeros(2*m*(n-1)*k_r,n*(order+1)*m);
% constraintData_r = zeros(m,k_r,3);
% constraintData_r(:,1,1:2) = 2;
% for i=1:m
%     for h=1:k_r
%         if(i==1)
%             c = zeros(1,n*(order+1)*m); %Will be a row vector of C
%             values = zeros(1,order+1);
%             for j=1:order+1
%                 tempCoeffs = computeMat(j,:);
%                 for k=1:h
%                     tempCoeffs = polyder(tempCoeffs);
%                 end
%                 values(j) = polyval(tempCoeffs,t(i));
%             end
%             values = repmat(values,1,n-1);%x,y,z  -  (yaw)
%             c( ((i-1)*(order+1)*n+1) : (i*(order+1)*n-(order+1))) = values;
%             C2(2*((i-1)+(h-1))+1,:) = c;
%             b2(2*(n-1)*(i-1)+1) = constraintData_r(i,h,1); %x
%             b2(2*(n-1)*(i-1)+2) = constraintData_r(i,h,2); %y
%             b2(2*(n-1)*(i-1)+3) = constraintData_r(i,h,3); %z
%         
%         else
%             
%         end
%     end
% end
% 
% b3 = zeros(2*m*k_psi,1);
% C3 = zeros(2*m*k_psi,n*(order+1)*m);
% for h=1:k_psi
%    
    
    
end

% C = [C1; C2; C3];
% b = [b1; b2; b3];
C = C1;
b = b1;
end