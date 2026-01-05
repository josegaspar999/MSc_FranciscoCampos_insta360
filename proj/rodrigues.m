function R = rodrigues(r)
theta = norm(r);
if theta < 1e-12
    R = eye(3);
    return;
end
k = r / theta;
K = [   0   -k(3)  k(2);
    k(3)   0   -k(1);
    -k(2) k(1)    0  ];
R = cos(theta)*eye(3) + (1-cos(theta))*(k*k') + sin(theta)*K;
