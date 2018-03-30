function image_denoising(file_name, f, r, k) 
    %% Read data and initialize outputs
    dat = exrread(strcat(file_name, '.exr'));
    datvar = exrread(strcat(file_name, '_var.exr'));
    flt = zeros(size(dat));
    wgtsum = zeros(size(dat));
    %% Loop over neighbors
    for dx = -r:r
        for dy= -r:r
            % compute distance to neighbor
            ngb =  circshift(dat,[dx,dy]);
            ngbvar = circshift(datvar,[dx,dy]);
            % non uniform variance equation
            d2pixel = (((dat - ngb).^2) - (datvar + min(ngbvar, datvar))) ...
               ./ ( eps + k^2 * (datvar + ngbvar));
            box_f = ones(2*f+1);
            % apply box filter
            d2patch = convn(d2pixel, box_f, 'same');
            wgt = exp(-max(0, d2patch));
            box_fm1 = ones(2*f-1);
            % box filkter weights for patch contribution
            wgt = convn(wgt, box_fm1, 'same');
            flt = flt + wgt .* ngb;
            wgtsum = wgtsum + wgt;
        end
    end
    %% Normalize and write result to a file
    flt =flt./wgtsum;
    exrwrite(flt, strcat(file_name, '_denoised.exr'));
end