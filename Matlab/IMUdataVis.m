clear all
close all
fit = load('fit.mat');

% Stop all unnecicary timers
T = timerfind;
if ~isempty(T)
    stop(T)
    delete(T)
end

timerPeriod = 0.01;
magRotationCorrect = [1.08088, 0.00325, 0.03648; ...
                      0.00325, 0.95485, 0.06939; ...
                      0.03648, 0.06939, 0.97519];
magCenterCorrect = [-2.22501e+02,2.18238,5.46688e+02];

myApp = IMUVisApp();
sensor = serialport("COM4",9600);
quad = Quad();
tic;
TimerControl = timer( "Period", timerPeriod, "ExecutionMode", "fixedRate", "BusyMode", "drop");
TimerControl.TimerFcn = {@mainLoop, myApp, quad, sensor, magRotationCorrect, magCenterCorrect};
start(TimerControl);

%mainLoop(0, 0, myApp, quad, sensor, magRotationCorrect, magCenterCorrect)


function mainLoop(~, ~, myApp, quad, sensor, magRotationCorrect, magCenterCorrect)

    inputDataString = readline(sensor);
    splitstInputDataString = strsplit(inputDataString);
    inputData = str2double(splitstInputDataString);
    AccData = inputData(1:3);
    GyroData = inputData(4:6);
    MagData = inputData(7:9);
    AccData = AccData - [550, -100, -1750];
    GyroData = GyroData - [-72.584,-97.213,-50.795];
    GyroData = GyroData / 2^15 * 250 /180 *pi;
    MagData = (MagData-magCenterCorrect)*magRotationCorrect;
    
    %fprintf('Accelerometer data: [');
    %fprintf('%g, ', AccData(1:end-1));
    %fprintf('%g]\n', AccData(end));
        
    %fprintf('Giroscope data:     [');
    %fprintf('%g, ', GyroData(1:end-1));
    %fprintf('%g]\n', GyroData(end));
    %AccData = AccData / norm(AccData);
    
    deltaT = toc;
    [q, p] = Kalman(AccData, GyroData, MagData, ...
        quad.getAttitudeQuat(), deltaT, quad.getErrorCov());
    tic;
    quad.setAttitudeQuat(q)
    quad.setErrorCov(p)
        
    tmp = quad.getGyroBasedAttitudeQuat() +  0.5*deltaT* ...
        [0,       -GyroData(1), -GyroData(2), -GyroData(3); ...
        GyroData(1), 0       , GyroData(3) , -GyroData(2); ...
        GyroData(2), -GyroData(3), 0       , GyroData(1) ; ...
        GyroData(3), GyroData(2) , -GyroData(1), 0       ] * quad.getGyroBasedAttitudeQuat();
    tmp = (tmp * (1/norm(tmp)))';
    quad.setGyroBasedAttitudeQuat(tmp')
        
    g = [0 0 1];
    AccData = AccData/norm(AccData);
    n = cross(AccData,g) / norm(cross(AccData,g));
    omega = atan2( norm(cross(AccData,g)) , dot(AccData,g));
    gravityQuat = [cos(omega/2) n*sin(omega/2)]';
    gravityQuat(4) = 0;
    gravityQuat = gravityQuat*(1/norm(gravityQuat));
    
    MagData(3) = 0;
    m = [0 1 0];
    MagData = MagData/norm(MagData);
    n = cross(MagData,m) / norm(cross(MagData,m));
    omega = atan2( norm(cross(MagData,m)) , dot(MagData,m));
    magQuat = [cos(omega/2) n*sin(omega/2)]';
    magQuat([2 3]) = 0;
    magQuat = magQuat*(1/norm(magQuat));
   
   
   if(mod(quad.getCounter(), 2) == 0)
    myApp.UpdateApp(gravityQuat, myApp.AccPlot);
    myApp.UpdateApp(quad.getAttitudeQuat(), myApp.KalmanPlot);
   elseif(mod(quad.getCounter(), 2) == 1)
    myApp.UpdateApp(magQuat, myApp.MagPlot);
    myApp.UpdateApp(quad.getGyroBasedAttitudeQuat(), myApp.GyroPlot);
   end     
    
   quad.setCounter(1 + quad.getCounter())
   
end



%IMUdata = load('IMUdata.mat');
%figure(1)
%plot(1:1000, IMUdata.unnamed(1:1000, 1:3) / 16384)
%figure(2)
%plot(1:1000, (IMUdata.unnamed(1:1000, 4:6) - mean(IMUdata.unnamed(1:1000, 4:6),1)) / 2^15 * 250)

