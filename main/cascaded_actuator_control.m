k = 20000;
r = 0.05;
N = 40;
m = 80;
g = 9.81;
Jm = 5.3e-4;
T_max = 1.363;
t_f = 15;
t_start = 0;
dt = 0.001;
t_arr = t_start:dt:t_f;
Kp_o = 10;
Kd_o = 0;
Kp_i = 50;
Kd_i = 10;
y_des = [0.7, 0.8, 0.9];

for i = 1:length(y_des)
    
    x = zeros(4,1);
    x(1) = 1; 
    x(2) = 0; 
    x(3) = 0; 
    x(4) = 0; 

    
    x_h = zeros(4, length(t_arr));
    tau_h = zeros(1, length(t_arr));
    x_h(:,1) = x;

    
    y_0 = m * g / k + 1;
    F_s = 0;

    
    for step = 2:length(t_arr)
       
        y_error = y_des(i) - x(1);
        y_error_dot = -x(2);
        F_des = m * ((Kp_o * y_error) + (Kd_o * y_error_dot) + g);

        
        F_s = k * (y_0 - x(1) + r/N * x(3));

        
        theta_desired = N/(r*k) * ((F_des) - k * (y_0 - x(1)));

       
        theta_error = theta_desired - x(3);
        theta_error_dot = -x(4);
        motor_torque = Jm * (Kp_i * theta_error + Kd_i * theta_error_dot) + F_des * r / N;

       
        motor_torque = min(motor_torque, T_max);

       
        dx = zeros(4,1);
        dx(1) = x(2);
        dx(2) = F_s/m - g;
        dx(3) = x(4);
        dx(4) = (motor_torque - (F_s * (r/N))) / Jm;
        x = x + dx * dt;

       
        x_h(:,step) = x;
        tau_h(step) = motor_torque;
    end

    
    figure(1);
    hold on;
    plot(t_arr, x_h(1,:));
    title('Position vs Time', 'FontSize', 15);
    ylabel('Position (m)', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12);
    legend('y_{target} = 0.7', 'y_{target} = 0.8', 'y_{target} = 0.9', 'FontSize', 12);

 
    figure(3);
    hold on;
    plot(t_arr, tau_h);
    ylim([0, 1.4]);
    xlim([0, 3]);
    title('Torque vs Time', 'FontSize', 15);
    ylabel('Torque (Nm)', 'FontSize', 12);
    xlabel('Time (s)', 'FontSize', 12);
    legend('y_{target} = 0.7', 'y_{target} = 0.8', 'y_{target} = 0.9', 'FontSize', 12);
end


figure(1);
for i = 1:length(y_des)
    yline(y_des(i), '--', 'LineWidth', 1);
end

figure(3);
yline(T_max, '--', 'LineWidth', 1);
