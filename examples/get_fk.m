function T_mid = get_fk(DH)

    Rot_x = @(t) [1,            0,       0,       0;
                  0,       cos(t), -sin(t),       0;
                  0,       sin(t),  cos(t),       0;
                  0,            0,       0,       1];
    
    Rot_z = @(t) [cos(t), -sin(t),       0,       0;
                  sin(t),  cos(t),       0,       0;
                       0,       0,       1,       0;
                       0,       0,       0,       1];
    
    Trans_x = @(d) [1,  0,  0,  d;
                    0,  1,  0,  0;
                    0,  0,  1,  0;
                    0,  0,  0,  1];
    
    Trans_z = @(d) [1,  0,  0,  0;
                    0,  1,  0,  0;
                    0,  0,  1,  d;
                    0,  0,  0,  1];

    n = size(DH, 1);
    T = eye(4, 4, 'sym'); % init T_ee_0
    T_mid = cell(n, 1);
    for i = 1:n
        T = T * Rot_x(DH(i, 1)) * Trans_x(DH(i, 2)) * Trans_z(DH(i, 3)) * Rot_z(DH(i, 4));
        T_mid{i, 1} = simplify(T);
    end
end