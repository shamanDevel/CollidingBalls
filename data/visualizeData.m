clf
data=load('timingData')
vals = [];
for i = 0 : 11
  vals(end+1,:) = mean(data((i*100+1):(i+1)*100,:)) ;
end
h1 = plot(2:13, vals(:,2), 'c--o', 'LineWidth', 3); hold on;
h2 = plot(2:13, vals(:,3), 'r--o', 'LineWidth', 3); hold on;
h3 = plot(2:13, vals(:,4), 'b--o', 'LineWidth', 3); hold on;
h4 = plot(2:13, vals(:,5), 'g--o', 'LineWidth', 3); hold on;

legend([h1,h2,h3,h4], 'dt0', 'dt1', 'dt2', 'dt3','Location','NorthWest');
xlabel('Square root of number of balls ()', 'FontSize', 30);
ylabel('Operation time (ms)', 'FontSize', 30);
set(gcf,'color','w')
set(gca,'FontSize', 30)
title('Relationship between operation time and number of balls', 'FontSize', 35)