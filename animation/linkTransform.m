function linkTransform(patch_obj, verts, Mat)
    % verts is a matrix of size n x 3
    %verts = get(patch_obj,'Vertices')

    % in order to multiply the verts matrix by 4x4 transf. Mat
    % addition of 4-th collumn with homogenous coordinare "1" is required
    verts = [verts, ones(size(verts,1),1)];
    % note that Transformation matrix has to be transposed once n x 4 matrix of points is multiplied
    new_verts = verts * transpose(Mat);

    % Update patch
    set(patch_obj,'Vertices', new_verts(:,1:3));
