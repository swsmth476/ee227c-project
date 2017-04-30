% plot results
position_indices = [1; 3];

headway = platoon_ref(:,1) - platoon_ref(:,3);

subplot(4,1,1);
plot(tout,headway);
title('Reference Platoon');
legend({'$\hat{x}_{L1}(t) - \hat{x}_{L2}(t)$'},'Interpreter','latex');
% xlabel('Time (s)');
ylabel('Headway (m)');
% legend('x_{L1}(t) - x_{L2}(t)');

velocity_indices = [2; 4; 6; 8; 10; 12];

% subplot(4,1,3);
% plot(tout,platoon_full(:,velocity_indices));
% title('Concrete Platoon Velocity');
% xlabel('Time (s)');
% ylabel('Value (m & m/s)');
% legend('v_{L1}(t)', 'v_1(t)', 'v_2(t)', 'v_{L2}(t)', 'v_3(t)', 'v_4(t)');

velocity_indices = [2; 4];

subplot(4,1,2);
plot(tout,platoon_ref(:,velocity_indices));
% title('Reference Platoon Velocity');
% legend('v_1(t)', 'v_2(t)');
legend({'$\hat{v}_{L1}(t)$', '$\hat{v}_{L2}(t)$'},'Interpreter','latex');
% xlabel('Time (s)');
ylabel('Velocity (m/s)');

% subplot(5,1,5);
% plot(tout,concrete_u);
% title('Concrete Platoon Input');
% xlabel('Time (s)');
% ylabel('Acceleration (m/s^2)');
% legend('u_1(t)', 'u_2(t)');

subplot(4,1,3);
plot(tout,ref_u);
% title('Reference Platoon Input');
ylabel('Input');
% legend('u_1(t)', 'u_2(t)');
legend({'$\hat{u}_{L1}(t)$', '$\hat{u}_{L2}(t)$'},'Interpreter','latex');

position_indices = [1; 3; 5; 7; 9; 11];

subplot(4,1,4);
plot(tout,platoon_full(:,position_indices));
title('Concrete Platoon');
legend({'$x_{L1}(t)$', '$x_1(t)$', '$x_2(t)$', '$x_{L2}(t)$', '$x_3(t)$', '$x_4(t)$'},'Interpreter','latex');
% xlabel('Time (s)');
xlabel('Time (s)');
ylabel('Position (m)');
% legend('x_{L1}(t)', 'x_1(t)', 'x_2(t)', 'x_{L2}(t)', 'x_3(t)', 'x_4(t)');
