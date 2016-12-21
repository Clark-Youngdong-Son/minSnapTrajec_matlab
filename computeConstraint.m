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


% Derivative constraints

% Position
C2 = zeros(2*m*(n-1)*k_r,n*(order+1)*m);                    %(n-1) : yaw excluded here
b2 = ones(2*m*(n-1)*k_r,1)*eps;
constraintData;
%constraintData_r = zeros(m,k_r,3);
for i=1:m
    for h=1:k_r
        if(i==1)
            %Initial
            values = zeros(1,order+1);
            for j=1:order+1
                tempCoeffs = computeMat(j,:);
                for k=1:h
                    tempCoeffs = polyder(tempCoeffs);
                end
                values(j) = polyval(tempCoeffs,t(i));
            end
            
            continuity = zeros(1,n-1);
            for k=1:n-1
                if(constraintData_r(i,h,k)==eps)
                    %Continuity
                    continuity(k) = true;
                end
                
                c = zeros(1,n*(order+1)*m);
                if(continuity(k))
                    c( ((i-1)*(order+1)*n+(k-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
                    c( ((m-1)*(order+1)*n+(k-1)*(order+1)+1) : ((m-1)*(order+1)*n+(k-1)*(order+1))+order+1) = -values;
                    C2(k + (h-1)*(n-1),:) = c;
                    b2(k + (h-1)*(n-1)) = 0; 
                else
                    c( ((i-1)*(order+1)*n+(k-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
                    C2(k + (h-1)*(n-1),:) = c;
                    b2(k + (h-1)*(n-1)) = constraintData_r(i,h,k); 
                end
            end
        
            %Final
            values = zeros(1,order+1);
            for j=1:order+1
                tempCoeffs = computeMat(j,:);
                for k=1:h
                    tempCoeffs = polyder(tempCoeffs);
                end
                values(j) = polyval(tempCoeffs,t(m+1));
            end
            
            for k=1:n-1
                if(constraintData_r(i,h,k)==eps)
                    %Continuity
                end
                c = zeros(1,n*(order+1)*m);
                if(~continuity(k))
                    c( ((m-1)*(order+1)*n+(k-1)*(order+1)+1) : ((m-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
                    C2(k + (h-1)*(n-1) + (n-1)*k_r,:) = c;
                    b2(k + (h-1)*(n-1) + (n-1)*k_r) = constraintData_r(i,h,k);
                end
            end

        else
            
            %Elsewhere
            values = zeros(1,order+1);
            for j=1:order+1
                tempCoeffs = computeMat(j,:);
                for k=1:h
                    tempCoeffs = polyder(tempCoeffs);
                end
                values(j) = polyval(tempCoeffs,t(i));
            end
            
            continuity = zeros(1,n-1);
            for k=1:n-1
                if(constraintData_r(i,h,k)==eps)
                    %Continuity
                    continuity(k) = true;
                end
                
                c = zeros(1,n*(order+1)*m);
                if(continuity(k))
                    c( ((i-2)*(order+1)*n+(k-1)*(order+1)+1) : ((i-2)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
                    c( ((i-1)*(order+1)*n+(k-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k-1)*(order+1))+order+1) = -values;
                    C2(k + (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r,:) = c;
                    b2(k + (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r) = 0;
                else
                    c( ((i-2)*(order+1)*n+(k-1)*(order+1)+1) : ((i-2)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
                    C2(k + (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r,:) = c;
                    b2(k + (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r) = constraintData_r(i,h,k);
                end
            end
            
            continuity = zeros(1,n-1);
            for k=1:n-1
                if(constraintData_r(i,h,k)==eps)
                    %Continuity
                    continuity(k) = true;
                end
                c = zeros(1,n*(order+1)*m);
                
                if(~continuity(k))
                    c( ((i-1)*(order+1)*n+(k-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k-1)*(order+1))+order+1) = values;
                    C2(k + (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r + (n-1)*k_r,:) = c;
                    b2(k + (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r + (n-1)*k_r) = constraintData_r(i,h,k);
                end
                
            end
            
        end
    end
end


% % % % % % % Yaw
% % % % % % C3 = zeros(2*m*k_psi,n*(order+1)*m);                    %(n-1) : yaw excluded here
% % % % % % b3 = zeros(2*m*k_psi,1);
% % % % % % constraintData;
% % % % % % 
% % % % % % for i=1:m
% % % % % %     for h=1:k_psi
% % % % % %         if(i==1)
% % % % % %             %Initial
% % % % % %             values = zeros(1,order+1);
% % % % % %             for j=1:order+1
% % % % % %                 tempCoeffs = computeMat(j,:);
% % % % % %                 for k=1:h
% % % % % %                     tempCoeffs = polyder(tempCoeffs);
% % % % % %                 end
% % % % % %                 values(j) = polyval(tempCoeffs,t(i));
% % % % % %             end
% % % % % %             
% % % % % %             for k=1:1
% % % % % %                 c = zeros(1,n*(order+1)*m);
% % % % % %                 c( ((i-1)*(order+1)*n+(k+3-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k+3-1)*(order+1))+order+1) = values;
% % % % % %                 C3(k + (h-1)*(1),:) = c;
% % % % % %                 b3(k + (h-1)*(1)) = constraintData_r(i,h,k);
% % % % % %             end
% % % % % %         
% % % % % %             %Final
% % % % % %             values = zeros(1,order+1);
% % % % % %             for j=1:order+1
% % % % % %                 tempCoeffs = computeMat(j,:);
% % % % % %                 for k=1:h
% % % % % %                     tempCoeffs = polyder(tempCoeffs);
% % % % % %                 end
% % % % % %                 values(j) = polyval(tempCoeffs,t(m+1));
% % % % % %             end
% % % % % %             
% % % % % %             for k=1:1
% % % % % %                 c = zeros(1,n*(order+1)*m);
% % % % % %                 c( ((m-1)*(order+1)*n+(k+3-1)*(order+1)+1) : ((m-1)*(order+1)*n+(k+3-1)*(order+1))+order+1) = values;
% % % % % %                 C3(k + (h-1)*(1) + (1)*k_r,:) = c;
% % % % % %                 b3(k + (h-1)*(1) + (1)*k_r) = constraintData_r(m,h,k);
% % % % % %             end
% % % % % % 
% % % % % %         else
% % % % % %             
% % % % % %             %Elsewhere
% % % % % % %             values = zeros(1,order+1);
% % % % % % %             for j=1:order+1
% % % % % % %                 tempCoeffs = computeMat(j,:);
% % % % % % %                 for k=1:h
% % % % % % %                     tempCoeffs = polyder(tempCoeffs);
% % % % % % %                 end
% % % % % % %                 values(j) = polyval(tempCoeffs,t(i));
% % % % % % %             end
% % % % % % %             
% % % % % % %             for k=1:1
% % % % % % %                 c = zeros(1,n*(order+1)*m);
% % % % % % %                 c( ((i-2)*(order+1)*n+(k+3-1)*(order+1)+1) : ((i-2)*(order+1)*n+(k+3-1)*(order+1))+order+1) = values;
% % % % % % %                 C3(k + (h-1)*(1) + 2*(i-1)*(1)*k_r,:) = c;
% % % % % % %                 b3(k + (h-1)*(1) + 2*(i-1)*(1)*k_r) = constraintData_r(i,h,k);
% % % % % % %             end
% % % % % % %             
% % % % % % %             for k=1:1
% % % % % % %                 c = zeros(1,n*(order+1)*m);
% % % % % % %                 c( ((i-1)*(order+1)*n+(k+3-1)*(order+1)+1) : ((i-1)*(order+1)*n+(k+3-1)*(order+1))+order+1) = values;
% % % % % % %                 C3(k + (h-1)*(1) + 2*(i-1)*(1)*k_r + (1)*k_r,:) = c;
% % % % % % %                 b3(k + (h-1)*(1) + 2*(i-1)*(1)*k_r + (1)*k_r) = constraintData_r(i,h,k);
% % % % % % %             end
% % % % % %             
% % % % % %         end
% % % % % %     end
% % % % % % end
% C = C1; % #:32
% b = b1;
C = [C1; C2]; % #:128
b = [b1; b2];
% C = [C1; C2; C3];
% b = [b1; b2; b3];
end