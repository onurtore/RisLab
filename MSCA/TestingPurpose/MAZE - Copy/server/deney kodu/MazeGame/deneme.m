clear ; close all; clc
fprintf('Plotting Data ...\n')
data = load('onur2.txt');
x = data(:,1); y = data(:,2); z = data(:,3); t= data(:,4); 
plot(x);
hold on;
plot(y,'black');
plot(z,'red');
plot(t,'green');

legend('xDiff','yDiff','algorithmForceX','algorithmForceZ');
