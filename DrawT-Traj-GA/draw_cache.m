x = [0.2 0.35 0.46 0.6 0.75 0.9];
y1 = [231.0553 254.8388 265.5197 277.7461 287.2301 294.9786];
y2 = [256.9536 283.8150 295.8781 309.6868 320.3980 	329.1490];

figure(1)
hold on
plot(x, y1, 'bo-', 'MarkerSize', 8, 'LineWidth', 1.5);
% hold on
plot(x, y2, 'ms-', 'MarkerSize', 8, 'LineWidth', 1.5);

xlabel({'Cache ratio', '(d)'});
ylabel('Total Throughput (Mbits)');
% ylabel('Tổng thông lượng (Mbits)');
legend('BCD', 'GA');
grid on
box on
set(gca, 'FontSize', 14);

% % Add non-bold title below the x-axis label
% text(mean(xlim), min(ylim) - 0.3*range(ylim), '(a)', ...
%     'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'normal', 'VerticalAlignment', 'top');

