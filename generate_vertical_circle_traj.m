function traj = generate_vertical_circle_traj(center,orientation,radius,speed,phi_start,phi_end,sampling_freq)

phi_total = phi_end - phi_start;
direction = phi_total / abs(phi_total);
omega = direction * abs(speed / radius);
angle_step = abs(omega / sampling_freq);
d_phi = 0:angle_step:abs(phi_total);

phi = phi_start + direction*d_phi;
cos_phi = cos(phi);
sin_phi = sin(phi);

p = radius*[cos_phi; zeros(size(d_phi)); -sin_phi] + center;
v = radius*[-omega*sin_phi; zeros(size(d_phi)); omega*cos_phi];
a = radius*[-omega^2.*cos_phi; zeros(size(d_phi)); omega^2.*sin_phi];
j = radius*[omega^3.*sin_phi; zeros(size(d_phi)); omega^3.*cos_phi];
s = radius*[omega^4.*cos_phi; zeros(size(d_phi)); -omega^4.*sin_phi];

% figure;
% plot3(p(1,:),p(2,:),p(3,:), 'LineWidth', 2); hold on;
% quiver3(p(1,:),p(2,:),p(3,:),a(1,:),a(2,:),a(3,:), 'r');
% axis equal;
% grid on;

T = 0:1/sampling_freq:abs(phi_total/omega);
Z = [p; v; a; j; s];
% U = generate_nominal_input(T,Z);

traj = [p; phi];
end
