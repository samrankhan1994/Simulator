f = load('path.txt');
x = f(:,1);
y = f(:,2);
z = f(:,3);
figure(1);
plot(y,  x, 'x', 'LineWidth', 3, 'MarkerSize', 10); zoom on; grid on; axis equal; hold on;
title('\bf Planned Path');
xlabel('\bf y');
ylabel('\bf x');
