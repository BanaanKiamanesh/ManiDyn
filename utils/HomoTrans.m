function T = HomoTrans(DHRow)
    % Function to Calculate a Homogeneous Transform from a Given DH Table Row

    alpha = DHRow(1);
    a     = DHRow(2);
    d     = DHRow(3);
    theta = DHRow(4);


    T = RotMatrix(theta, 'z') * TrnsMatrix(d, 'z') * TrnsMatrix(a, 'x') * RotMatrix(alpha, 'x');
end