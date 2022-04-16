clear
oblongOptim = readmatrix('oblongOut.csv');
oblongBounds = readmatrix('oblongBounds.csv');

% Plot Bounds
plot(oblongBounds(:,1), oblongBounds(:,2));
hold on
axis equal
plot(oblongBounds(:,3), oblongBounds(:,4));

% Plot Output
plot([oblongOptim(:,1);oblongOptim(1,1)], [oblongOptim(:,2);oblongOptim(1,2)]);
hold off
