set = load('FullRangeOfMotion.mat');
data = set.FR;
%scatter3(data(:,7), data(:,8), data(:,9)) % for visualizing magnetometer data
%fit = ellipsoidalFit([data(:,7), data(:,8), data(:,9)]');
%showfit(fit)
%omega = deg2rad(fit.angle);
%theta = atan2((data(:,7) - fit.center(1)), (data(:,8) - fit.center(2)));
%xCenter = data(:,7) - fit.center(1);
%yCenter = data(:,8) - fit.center(2);
%xCorrect = xCenter ./ sqrt(xCenter.^2 + yCenter.^2);
%yCorrect = yCenter ./ sqrt(xCenter.^2 + yCenter.^2);
%scatter(xCorrect, yCorrect);
%xCorrect = xCenter ./ sqrt( (fit.b*cos(theta+omega)).^2 + (fit.a*sin(theta+omega)).^2);
%yCorrect = yCenter ./ sqrt( (fit.b*cos(theta+omega)).^2 + (fit.a*sin(theta+omega)).^2);
%scatter(xCorrect, yCorrect);

[A,b,expmfs] = magcal([data(:,7), data(:,8), data(:,9)]);
C = ([data(:,7), data(:,8), data(:,9)]-b)*A; % calibrated data
scatter3(C(:,1), C(:,2), C(:,3))
hold on
fit = ellipsoidalFit([C(:,1), C(:,2), C(:,3)]');
showfit(fit)



hold off