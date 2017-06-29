clear ; close all; clc
fprintf('Plotting Data ...\n')
data = load('onur2.txt');
X = data(:,1); y = data(:,2); z = data(:,3); t= data(:,4); 
plot(X);
hold on;
plot(z,'black');
legend('algorithmForceX','scaledAlgorithmForceX');

figure;
plot(y,'red');
hold on;
plot(t,'green');

legend('algorithmForceZ', 'scaledAlgorithmForceZ');
