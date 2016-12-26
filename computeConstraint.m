function [C, b, C3, b3] = computeConstraint(order, m, k_r, k_psi, t, keyframe, corridor_position, n_intermediate, corridor_width)

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

%Corridor constraints

C3 = [];
%Size (2*3*n_intermediate,n*(order+1)*m);   
%2 : absoulute value constraint
%3 : x,y,z
%n_intermediate : number of intermediate points

b3 = [];
t_vector = (keyframe(1:3,corridor_position(2)) - keyframe(1:3,corridor_position(1)))...
/norm(keyframe(1:3,corridor_position(2)) - keyframe(1:3,corridor_position(1)));
%unit vector of direction of the corridor

t_intermediate = linspace(t(corridor_position(1)),t(corridor_position(2)),n_intermediate+2);
t_intermediate = t_intermediate(2:end-1);
%intermediate time stamps

computeMat = eye(order+1);          %Required for computation of polynomials
for i = 1:n_intermediate
%     intermediate_pos(1) = interp1([t(corridor_position(1)) t(corridor_position(2))],...
%         [keyframe(1,corridor_position(1)) keyframe(1,corridor_position(2))],t_intermediate(i));
%     intermediate_pos(2) = interp1([t(corridor_position(1)) t(corridor_position(2))],...
%         [keyframe(2,corridor_position(1)) keyframe(2,corridor_position(2))],t_intermediate(i));
%     intermediate_pos(3) = interp1([t(corridor_position(1)) t(corridor_position(2))],...
%         [keyframe(3,corridor_position(1)) keyframe(3,corridor_position(2))],t_intermediate(i));
    values = zeros(1,order+1);
    for j=1:order+1
        values(j) = polyval(computeMat(j,:),t_intermediate(i));
    end
    
    c = zeros(6, n*(order+1)*m);       %Absolute value constraint : two inequality constraints
    b = zeros(6, 1);
    
    rix = keyframe(1,corridor_position(1));
    riy = keyframe(2,corridor_position(1));
    riz = keyframe(3,corridor_position(1));
    %x
    c(1,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1))...
        = [values zeros(1,2*(order+1))]...
        - t_vector(1)*[t_vector(1)*values t_vector(2)*values t_vector(3)*values];
    b(1) = corridor_width +...
        rix+t_vector(1)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));
    c(2,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1))...
        = -c(1,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1));
    b(2) = corridor_width +...
        -rix-t_vector(1)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));
    %y
    c(3,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1))...
        = [zeros(1,order+1) values zeros(1,order+1)]...
        - t_vector(2)*[t_vector(1)*values t_vector(2)*values t_vector(3)*values];
    b(3) = corridor_width +...
        riy+t_vector(2)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));
    c(4,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1))...
        = -c(3,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1));
    b(4) = corridor_width +...
        -riy-t_vector(2)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));
    %z
    c(5,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1))...
        = [zeros(1,2*(order+1)) values]...
        - t_vector(3)*[t_vector(1)*values t_vector(2)*values t_vector(3)*values];
    b(5) = corridor_width +...
        riz+t_vector(3)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));
    c(6,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1))...
        = -c(5,(corridor_position(1)-1)*n*(order+1)+0*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1));
    b(6) = corridor_width +...
        -riz-t_vector(3)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));
    
    C3 = [C3; c];
    b3 = [b3; b];
end

C = [C1; C2];
b = [b1; b2];
end