function M = EulertoM(euler)
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    c = cos([phi, theta, psi]);
    s = sin([phi, theta, psi]);
%     M = [
%         c(3)*c(2),                -s(3)*c(2),                 s(2);
%         s(3)*c(1)+c(3)*s(2)*s(1), c(3)*c(1)-s(3)*s(2)*s(1),   -c(2)*s(1);
%         s(3)*s(1)-c(3)*s(2)*c(1), c(3)*s(1)+s(3)*s(2)*c(1),   c(2)*c(1);
%         ];
    
    M = [
        c(3)*c(2),                s(3)*c(2),                  -s(2);
        c(3)*s(2)*s(1)-s(3)*c(1), s(3)*s(2)*s(1)+c(3)*c(1),   c(2)*s(1);
        c(3)*s(2)*c(1)+s(3)*s(1), s(3)*s(2)*c(1)-c(3)*s(1),   c(2)*c(1);
        ];