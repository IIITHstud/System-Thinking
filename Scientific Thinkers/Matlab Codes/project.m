m1 = 10;
m2 = 5;
l1 = 0.2;
l2 = 0.1;
g = 9.81;

q10 = 0.1;
q20 = 0.1;
dq1_0 = 0;
dq2_0 = 0;

q1f = 0;
q2f = 0;

KP1 = 50;
KP2 = 60;
KI1 = 80;
KI2 = 90;
KD1 = 80;
KD2 = 90;

%odefun = @(t, x)[system_dynamics(t, x, m1, m2, l1, l2, g, KP1, KP2, KD1, KD2, KI1, KI2, q1f, q2f)];

tspan = [0 10];

initial_conditions = [q10, dq1_0, q20, dq2_0,0,0];

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);
[T, y] = ode45(@(t, x)system_dynamics(t, x, m1, m2, l1, l2, g, KP1, KP2, KD1, KD2, KI1, KI2, q1f, q2f), tspan, initial_conditions,options);


q1 = y(:, 1);
q2 = y(:, 3);

e1 = q1f - q1;
e2 = q2f - q2;

figure;
subplot(4, 2, 1);
plot(T, q1, 'r');
xlabel('Time (s)');
ylabel('q1 (rad)');
title('Joint Angles vs. Time (PID Control)');

%{
subplot(4, 2, 2);
plot(T, e1, 'b');
xlabel('Time (s)');
ylabel('e1 (rad)');
title('Error in q1 vs. Time');
%}

subplot(4, 2, 3);
plot(T, q2, 'g');
xlabel('Time (s)');
ylabel('q2 (rad)');

%{
subplot(4, 2, 4);
plot(T, e2, 'p', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('e2 (rad)');
title('Error in q2 vs. Time');
%}
