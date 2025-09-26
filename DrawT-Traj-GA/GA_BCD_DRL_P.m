x = [30 32 34 36 38 40];
% y1 = [646.818453 810.2494728 1112.2968168 1546.6796928 1714.9827885];
y1 = [162.29421 177.89748 190.127048 199.152184 205.505268 209.822908];
% y2 = [359.343585 761.6494728 617.942676 859.266496];
% y2 = [1230 1560 1800 2040 2340];
y2 = [255.69 268.26 289.74 307.97 322.10 332.93];
y3 = [90.78 97.65 103.69 110.41 116.99 125.85];
figure(1)
hold on
plot(x, y1, 'bo-', 'MarkerSize', 8, 'LineWidth', 1.5);
% hold on
plot(x, y2, 'ms-', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(x, y3, 'r^-', 'MarkerSize', 8, 'LineWidth', 1.5);

xlabel({'P_{WPT} (dB)', '(c)'});
ylabel('Total Throughput (Mbits)');
% ylabel('Thời gian thực hiện (seconds)');
legend('BCD','GA','DRL');
grid on
box on
set(gca,'FontSize',14)