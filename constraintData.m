%Only for the quadrotor system

%Position constraints
constraintData_r = zeros(m,k_r,3);

%velocity
if(k_r>=1)
    constraintData_r(1,1,1:3) = 0;          %At starting position
    constraintData_r(2:m,1,1:2) = eps;      %x,y velocities
    constraintData_r(2:m,1,3) = eps;        %z velocity
end
%acceleration
if(k_r>=2)
    constraintData_r(1,2,3) = 0;            %At starting position
    constraintData_r(2:m,2,1:2) = eps;      %x,y accelerations
    constraintData_r(2:m,2,3) = eps;        %z acceleration
end
%jerk
if(k_r>=3)
    %all zeros
end
%snap
if(k_r>=4)
    %all zeros
end

%Yaw constraints
constraintData_psi = zeros(m,k_psi,1);

%velocity
if(k_psi>=1)
    constraintData_psi(1,1,1) = 0;          %At starting position
end
%acceleration
if(k_psi>=2)
    %all zeros
end
%jerk
if(k_psi>=3)
   %all zeros 
end
%snap
if(k_psi>=4)
   %all zeros 
end
