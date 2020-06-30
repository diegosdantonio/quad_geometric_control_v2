function[traj] = get_load_traj_circle(t)

r = 3; % radius
w = pi/2; % angular velocity
c = 0; % center
a=0.4;
b=0.6;

%% Desired Load Position Generation

traj.x = [0.4*t; a*sin(w*t); b*cos(w*t)];
traj.dx = [0.4; w*a*cos(w*t); -w*b*sin(w*t)];
traj.d2x = [0; -w^2*a*sin(w*t); -w^2*b*cos(w*t)];
traj.d3x = [0; -w^3*a*cos(w*t); w^3*b*sin(w*t)];
traj.d4x = [0; w^4*a*sin(w*t); w^4*b*cos(w*t)];
traj.d5x = [0; w^5*a*cos(w*t); -w^5*b*sin(w*t)];
traj.d6x = [0; -w^6*a*sin(w*t); -w^6*b*cos(w*t)];

traj.b1d = [cos(w*t); sin(w*t); 0];
traj.db1d = [-w*sin(w*t); w*cos(w*t); 0];
traj.d2b1d = [-w^2*cos(w*t); -w^2*sin(w*t); 0];
traj.d3b1d = [w^3*sin(w*t); -w^3*cos(w*t); 0];
traj.d4b1d = [w^4*cos(w*t); w^4*sin(w*t); 0];
traj.d5b1d = [-w^5*sin(w*t); w^5*cos(w*t); 0];
traj.d6b1d = [-w^6*cos(w*t); -w^6*sin(w*t); 0];

end

