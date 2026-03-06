function m = thinprism(M, params)

    % M      - 3xN matrix containing the coordinates of the 3D points
    % params - 12x1 vector containing the parameters of the model
    % m      - 2xN matrix containing the coordinates of the 2D points

    % 0. Load thin prism params - fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
    fx  = params(1);
    fy  = params(2);
    cx  = params(3);
    cy  = params(4);
    k1  = params(5);
    k2  = params(6);
    p1  = params(7);
    p2  = params(8);
    k3  = params(9);
    k4  = params(10);
    sx1 = params(11);
    sy1 = params(12);

    % 1. Pinhole normalization
    xn = M(1,:)./M(3,:);
    yn = M(2,:)./M(3,:);
    
    % 2. Fisheye projection (equidistant)
    r = sqrt(xn.^2 + yn.^2);
    u = xn;
    v = yn;

    idx = r > 1e-12;
    theta = atan(r(idx));
    scale = theta ./ r(idx);
    u(idx) = xn(idx) .* scale;
    v(idx) = yn(idx) .* scale;

    % 3. Distortion
    u2 = u.^2;
    v2 = v.^2;
    uv = u .* v;

    r2 = u2 + v2;
    r4 = r2.^2;
    r6 = r4 .* r2;
    r8 = r6 .* r2;

    radial = k1*r2 + k2*r4 + k3*r6 + k4*r8;
    
    du = u .* radial + 2.0*p1.*uv + p2.*(r2 + 2.0*u2) + sx1.*r2;
    dv = v .* radial + 2.0*p2.*uv + p1.*(r2 + 2.0*v2) + sy1.*r2;

    ud = u + du;
    vd = v + dv;

    % 4. Pixel coordinates
    m(1,:) = fx * ud + cx;
    m(2,:) = fy * vd + cy;

end