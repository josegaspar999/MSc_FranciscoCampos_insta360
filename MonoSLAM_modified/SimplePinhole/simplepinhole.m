function m = simplepinhole(M, params)

    % M      - 3xN matrix containing the coordinates of the 3D points
    % params - 3x1 vector containing the parameters of the model
    % m      - 2xN matrix containing the coordinates of the 2D points

    % 0. Load simple pinhole params - f, cx, cy
    f  = params(1);
    cx  = params(2);
    cy  = params(3);

    % 1. Pinhole normalization
    xn = M(1,:)./M(3,:);
    yn = M(2,:)./M(3,:);
    
    % 2. Pixel coordinates
    m(1,:) = f * xn + cx;
    m(2,:) = f * yn + cy;

end