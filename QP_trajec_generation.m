c = zeros(4*(order+1)*m);               %coefficients of polynomial functions


%Quadratic cost function (minimum snap) : J = c.' * A * c
mu_r = 1; mu_psi = 1;                   %constants that makes the integrand nondimensional
k_r = 4; k_psi = 2;

%Compute the cost function matrix, A
A = computeA(order, m, mu_r, mu_psi, k_r, k_psi, t);

%Compute the constraint function matrix, C
% [C, b] = computeConstraint(order, m, k_r, k_psi, t, keyframe);
[C, b, Cin, bin] = computeConstraint(order, m, 3, 2, t, keyframe, corridor_position, n_intermediate, corridor_width);

options = optimoptions('quadprog', 'Display', 'iter', 'MaxIterations', 4000);
tic;
solution_corridor = quadprog(2*A, [], Cin, bin, C, b, [], [], [], options);
toc;
tic;
solution = quadprog(2*A, [], [], [], C, b, [], [], [], options);
toc;
% solution = quadprog(2*A, [], [], [], [], [], [], [], [], options);