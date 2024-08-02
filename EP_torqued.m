function derivs = EP_torqued(I1,I2,I3,L,condArr)

% Quaternion kinematic differential equations
% Inputs - I1, I2, I3 (moments of inertia), L (torque), condArr (quaternion
% and angular velocity)
% Outputs - quaternion and angular velocity derivatives

ep = condArr(1:4);
w = condArr(5:7);

K1 = -1*(I3-I2)/I1;
K2 = -1*(I1-I3)/I2;
K3 = -1*(I2-I1)/I3;

derivW1 = K1*w(2)*w(3) + L(1)/I1;
derivW2 = K2*w(3)*w(1) + L(2)/I2;
derivW3 = K3*w(1)*w(2) + L(3)/I3;

derivW = [derivW1;derivW2;derivW3];


derivEP13 = 0.5*(ep(4)*w + cross(ep(1:3),w));
derivEP4 = -0.5*dot(ep(1:3),w);

derivEP = [derivEP13;derivEP4];
derivs = [derivEP ; derivW];

end