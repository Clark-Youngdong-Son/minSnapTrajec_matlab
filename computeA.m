function A = computeA(order, m, mu_r, mu_psi, k_r, k_psi, t)

polynomial_r = ones(1,order+1);
for i=1:k_r
    polynomial_r = polyder(polynomial_r);       %Differentiation
end

polynomial_psi = ones(1,order+1);
for i=1:k_psi
    polynomial_psi = polyder(polynomial_psi);   %Differentiation
end

A = [];
for i=1:m
    A_x = zeros(order+1,order+1);
    A_y = zeros(order+1,order+1);
    A_z = zeros(order+1,order+1);
    A_psi = zeros(order+1,order+1);
    for j=1:order+1
        for k=j:order+1
            
            %Position
            if(j<=length(polynomial_r) && (k<=length(polynomial_r)))
                order_t_r = ((order-k_r-j+1)+(order-k_r-k+1));
                if(j==k)
                    A_x(j,k) = polynomial_r(j)^2/(order_t_r+1)...
                        *(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                    A_y(j,k) = polynomial_r(j)^2/(order_t_r+1)...
                        *(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                    A_z(j,k) = polynomial_r(j)^2/(order_t_r+1)...
                        *(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                else
                    A_x(j,k) = 2*polynomial_r(j)*polynomial_r(k)/(order_t_r+1)...
                        *(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                    A_y(j,k) = 2*polynomial_r(j)*polynomial_r(k)/(order_t_r+1)...
                        *(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                    A_z(j,k) = 2*polynomial_r(j)*polynomial_r(k)/(order_t_r+1)...
                        *(t(i+1)^(order_t_r+1)-t(i)^(order_t_r+1));
                end
            end
            
            %Yaw
            if(j<=length(polynomial_psi) && (k<=length(polynomial_psi)))
                order_t_psi = ((order-k_psi-j+1)+(order-k_psi-k+1));
                if(j==k)
                    A_psi(j,k) = polynomial_psi(j)^2/(order_t_psi+1)...
                        *(t(i+1)^(order_t_psi+1)-t(i)^(order_t_psi+1));
                else
                    A_psi(j,k) = 2*polynomial_psi(j)*polynomial_psi(k)/(order_t_psi+1)...
                        *(t(i+1)^(order_t_psi+1)-t(i)^(order_t_psi+1));
                end
            end
            
        end
    end
    A = blkdiag(A,mu_r*A_x,mu_r*A_y,mu_r*A_z,mu_psi*A_psi);
end
A = 0.5*(A + A.'); %Make it symmetric
end