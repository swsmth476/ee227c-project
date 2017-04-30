position_indices = [1; 3; 5; 7; 9; 11];

% plot results
subplot(3,2,1);
plot(tout,platoon_full(:,position_indices));
title('Concrete Platoon Position');
% xlabel('Time (s)');
% ylabel('Value (m & m/s)');
legend('x_{L1}(t)', 'x_1(t)', 'x_2(t)', 'x_{L2}(t)', 'x_3(t)', 'x_4(t)');

position_indices = [1; 3];

headway = platoon_ref(:,1) - platoon_ref(:,3);

subplot(3,2,2);
plot(tout,headway);
title('Reference Platoon Headway');
% xlabel('Time (s)');
% ylabel('Value (m & m/s)');
legend('x_{L1}(t) - x_{L2}(t)');

velocity_indices = [2; 4; 6; 8; 10; 12];

subplot(3,2,3);
plot(tout,platoon_full(:,velocity_indices));
title('Concrete Platoon Velocity');
% xlabel('Time (s)');
% ylabel('Value (m & m/s)');
legend('v_{L1}(t)', 'v_1(t)', 'v_2(t)', 'v_{L2}(t)', 'v_3(t)', 'v_4(t)');

velocity_indices = [2; 4];

subplot(3,2,4);
plot(tout,platoon_ref(:,velocity_indices));
title('Reference Platoon Velocity');
% xlabel('Time (s)');
% ylabel('Value (m & m/s)');

subplot(3,2,5);
plot(tout,concrete_u);
title('Concrete Platoon Input');
xlabel('Time (s)');
% ylabel('Acceleration (m/s^2)');
legend('u_1(t)', 'u_2(t)');

subplot(3,2,6);
plot(tout,ref_u);
title('Reference Platoon Input');
xlabel('Time (s)');
% ylabel('Acceleration (m/s^2)');
legend('v_1(t)', 'v_2(t)');
