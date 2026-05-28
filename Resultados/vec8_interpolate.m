function vecOut = vec8_interpolate(vecStart, vecEnd, t)
if length(t)>1
    for i=1:length(t)
        vecOut(:,i)= vec8_interpolate(vecStart, vecEnd, t(i));
    end
    return
end

    % INTERPOLATEVECTOR8 Interpolates between two 8x1 transformation vectors.
    % Uses Linear Interpolation (Lerp) for translation and scale, 
    % and Spherical Linear Interpolation (Slerp) for the quaternion.
    % Inputs:
    %   vecStart - 8x1 vector at t = 0 [tx; ty; tz; qw; qx; qy; qz; s]
    %   vecEnd   - 8x1 vector at t = 1 [tx; ty; tz; qw; qx; qy; qz; s]
    %   t        - Interpolation factor (0 <= t <= 1)
    % Output:
    %   vecOut   - 8x1 interpolated vector

    % Clamp t to
    t = max(0, min(1, t));

    % 1. Linear Interpolation for Translation (elements 1 to 3)
    pStart = vecStart(1:3);
    pEnd = vecEnd(1:3);
    pInterp = (1 - t) * pStart + t * pEnd;

    % 2. Spherical Linear Interpolation (Slerp) for Quaternion (elements 4 to 7)
    qStart = vecStart(4:7);
    qEnd = vecEnd(4:7);

    % Compute the cosine of the angle between quaternions
    dotProd = dot(qStart, qEnd);

    % If the dot product is negative, invert one quaternion to take the shorter path
    if dotProd < 0
        qEnd = -qEnd;
        dotProd = -dotProd;
    end

    % If the quaternions are very close, use linear interpolation to avoid division by zero
    if dotProd > 0.9995
        qInterp = (1 - t) * qStart + t * qEnd;
    else
        % Standard Slerp formula
        theta0 = acos(dotProd);
        theta = theta0 * t;
        
        sinTheta0 = sin(theta0);
        sinTheta = sin(theta);
        
        sStart = sin(theta0 - theta) / sinTheta0;
        sEnd = sinTheta / sinTheta0;
        
        qInterp = sStart * qStart + sEnd * qEnd;
    end

    % Normalize the interpolated quaternion
    qInterp = qInterp / norm(qInterp);

    % 3. Linear Interpolation for Scale (element 8)
    sStart = vecStart(8);
    sEnd = vecEnd(8);
    sInterp = (1 - t) * sStart + t * sEnd;

    % Combine results into the final 8x1 vector
    vecOut = [pInterp; qInterp; sInterp];
end
