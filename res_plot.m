%% 位姿误差
for i=2:5
    err{i}(:,1) = eta{i}(:,1) - eta{1}(:,1) - Configuration{i}(1)*ones(size(eta{i},1),1);
    err{i}(:,2) = eta{i}(:,2) - eta{1}(:,2) - Configuration{i}(2)*ones(size(eta{i},1),1);
    err{i}(:,3) = eta{i}(:,3) - eta{1}(:,3) - Configuration{i}(3)*ones(size(eta{i},1),1);
end
figure(1)
subplot(3,1,1)
plot(Time, err{2}(1:end,1), Time, err{3}(1:end,1), Time, err{4}(1:end,1), Time, err{5}(1:end,1), 'LineWidth', 2 );
xlim([0,20]);
ylabel('$\eta_{e1}$(m)','interpreter','latex')
set(gca, 'FontSize', 16)
legend('ASV1', 'ASV2', 'ASV3', 'ASV4', 'NumColumns', 4, 'FontSize', 12);
subplot(3,1,2)
plot(Time, err{2}(1:end,2), Time, err{3}(1:end,2), Time, err{4}(1:end,2), Time, err{5}(1:end,2), 'LineWidth', 2 );
xlim([0,20]);
ylabel('$\eta_{e2}$(m)','interpreter','latex')
set(gca, 'FontSize', 16)
subplot(3,1,3)
plot(Time, err{2}(1:end,3), Time, err{3}(1:end,3), Time, err{4}(1:end,3), Time, err{5}(1:end,3), 'LineWidth', 2 );
xlim([0,20]);
xlabel('$t$(s)','interpreter','latex')
ylabel('$\eta_{e3}$(rad)','interpreter','latex')
set(gca, 'FontSize', 16)

% 速度误差
for i=2:5
    derr{i} = deta{i} - deta{1};
end

figure(2)
subplot(3,1,1)
plot(Time, derr{2}(1:end,1), Time, derr{3}(1:end,1), Time, derr{4}(1:end,1), Time, derr{5}(1:end,1), 'LineWidth', 2 );
xlim([0,30]);
ylabel('$\omega_{e1}$(m/s)','interpreter','latex')
set(gca, 'FontSize', 16)
legend('i=1', 'i=2', 'i=3', 'i=4', 'NumColumns', 4, 'FontSize', 12);
subplot(3,1,2)
plot(Time, derr{2}(1:end,2), Time, derr{3}(1:end,2), Time, derr{4}(1:end,2), Time, derr{5}(1:end,2), 'LineWidth', 2 );
xlim([0,30]);
ylabel('$\omega_{e2}$(m/s)','interpreter','latex')
set(gca, 'FontSize', 16)
subplot(3,1,3)
plot(Time, derr{2}(1:end,3), Time, derr{3}(1:end,3), Time, derr{4}(1:end,3), Time, derr{5}(1:end,3), 'LineWidth', 2 );
xlim([0,30]);
xlabel('$t$(s)','interpreter','latex')
ylabel('$\omega_{e3}$(rad/s)','interpreter','latex')
set(gca, 'FontSize', 16)

%% 权重矩阵
figure(5)
subplot(4,1,1)
plot(Time, Zeta{2}(:,2), Time, Zeta{3}(:,2), Time, Zeta{4}(:,2), Time, Zeta{5}(:,2), 'LineWidth', 2 );
legend('$\zeta_{11}$', '$\zeta_{21}$', '$\zeta_{31}$', '$\zeta_{41}$', 'NumColumns', 4, 'interpreter','latex', 'FontSize', 12);
xlim([0,50]);
set(gca, 'FontSize', 16)

subplot(4,1,2)
plot(Time, Zeta{2}(:,3), Time, Zeta{3}(:,3), Time, Zeta{4}(:,3), Time, Zeta{5}(:,3), 'LineWidth', 2 );
legend('$\zeta_{12}$', '$\zeta_{22}$', '$\zeta_{32}$', '$\zeta_{42}$', 'NumColumns', 4, 'interpreter','latex', 'FontSize', 12);
xlim([0,50]);
set(gca, 'FontSize', 16)

subplot(4,1,3)
plot(Time, Zeta{2}(:,4), Time, Zeta{3}(:,4), Time, Zeta{4}(:,4), Time, Zeta{5}(:,4), 'LineWidth', 2 );
legend('$\zeta_{13}$', '$\zeta_{23}$', '$\zeta_{33}$', '$\zeta_{43}$', 'NumColumns', 4, 'interpreter','latex', 'FontSize', 12);
xlim([0,50]);
set(gca, 'FontSize', 16)

subplot(4,1,4)
plot(Time, Zeta{2}(:,5), Time, Zeta{3}(:,5), Time, Zeta{4}(:,5), Time, Zeta{5}(:,5), 'LineWidth', 2 );
legend('$\zeta_{14}$', '$\zeta_{24}$', '$\zeta_{34}$', '$\zeta_{44}$', 'NumColumns', 4, 'interpreter','latex', 'FontSize', 12);
xlim([0,50]);
xlabel('$t$(s)','interpreter','latex')
set(gca, 'FontSize', 16)

% 全部的轨迹
figure(3)
plot(eta{1}(:,1),eta{1}(:,2),'k', 'LineWidth', 2)
hold on
plot(eta{2}(:,1),eta{2}(:,2), 'Color', [0 0.4470 0.7410], 'LineWidth', 2)
hold on
plot(eta{3}(:,1),eta{3}(:,2),'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
hold on
plot(eta{4}(:,1),eta{4}(:,2),'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2)
hold on
plot(eta{5}(:,1),eta{5}(:,2),'Color', [0.2706 0.5451 0.4549], 'LineWidth', 2)
xlabel('$x$(m)','interpreter','latex')
ylabel('$y$(m)','interpreter','latex')
set(gca, 'FontSize', 16)

temp=size(Time,2);
L=1.255*5;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
B=0.5*5;
L01=0.8*5;  
% 绘制拓扑
i = 1
[xx yy]=ship_contour(eta{1}(i,:)',L,B,L01);
fill(xx,yy,'k')
[xx yy]=ship_contour(eta{2}(i,:)',L,B,L01);
fill(xx,yy,[0 0.4470 0.7410])
[xx yy]=ship_contour(eta{3}(i,:)',L,B,L01);
fill(xx,yy,[0.8500 0.3250 0.0980])
[xx yy]=ship_contour(eta{4}(i,:)',L,B,L01);
fill(xx,yy,[0.9290 0.6940 0.1250])
[xx yy]=ship_contour(eta{5}(i,:)',L,B,L01);
fill(xx,yy,[0.2706 0.5451 0.4549])

i = 20000
[xx yy]=ship_contour(eta{1}(i,:)',L,B,L01);
p1 = fill(xx,yy,'k')
[xx yy]=ship_contour(eta{2}(i,:)',L,B,L01);
p2 = fill(xx,yy,[0 0.4470 0.7410])
[xx yy]=ship_contour(eta{3}(i,:)',L,B,L01);
p3 = fill(xx,yy,[0.8500 0.3250 0.0980])
[xx yy]=ship_contour(eta{4}(i,:)',L,B,L01);
p4 = fill(xx,yy,[0.9290 0.6940 0.1250])
[xx yy]=ship_contour(eta{5}(i,:)',L,B,L01);
p5 = fill(xx,yy,[0.2706 0.5451 0.4549])
line([eta{1}(i,1) eta{2}(i,1) eta{3}(i,1) eta{4}(i,1) eta{5}(i,1) eta{2}(i,1)], ...
     [eta{1}(i,2) eta{2}(i,2) eta{3}(i,2) eta{4}(i,2) eta{5}(i,2) eta{2}(i,2)],'color','k','linestyle',':','LineWidth',2) 
 
legend([p1, p2, p3, p4, p5], {'Leader', 'i=1', 'i=2', 'i=3', 'i=4'}, 'NumColumns', 5, 'FontSize', 12);
axis equal