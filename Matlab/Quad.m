classdef Quad < matlab.System
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TimeElapsed = 0;
        Counter = 1;
        
        AttitudeQuat = [1 0 0 0]';
        GyroBasedAttitudeQuat = [1 0 0 0]';
        ErrorCov = [0.1250, 0.0003, 0.0003, 0.0003; ... 
        0.0003, 0.1250, 0.0003, 0.0003; ... 
        0.0003, 0.0003, 0.1250, 0.0003; ... 
        0.0003, 0.0003, 0.0003, 0.1250]; 
        DataLog = [];
        DataLogSize = 1000;
    end
    
    methods
        function obj = Quad()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function setTimeElapsed(obj, timeElapsed)
            obj.TimeElapsed = timeElapsed;
        end
        
        function setCounter(obj, counter)
            obj.Counter = counter;
        end
        
        function setAttitudeQuat(obj, attitudeQuat)
            obj.AttitudeQuat = attitudeQuat;
        end
        
        function setGyroBasedAttitudeQuat(obj, gyroBasedAttitudeQuat)
            obj.GyroBasedAttitudeQuat = gyroBasedAttitudeQuat;
        end
        
        function setErrorCov(obj, errorCov)
            obj.ErrorCov = errorCov;
        end
        
        function addTooDataLog(obj, dataLog)
            obj.DataLog = [obj.DataLog; dataLog];
            
            if size(obj.DataLog, 2) > obj.DataLogSize
                obj.DataLog = obj.DataLog(2:end, 1:end);
            end
        end
        
        %
        % --------------------
        %
        
        function timeElapsed = getTimeElapsed(obj)
            timeElapsed = obj.TimeElapsed;
        end
        
        function counter = getCounter(obj)
            counter = obj.Counter;
        end
        
        function attitudeQuat = getAttitudeQuat(obj)
            attitudeQuat = obj.AttitudeQuat;
        end
        
        function gyroBasedAttitudeQuat = getGyroBasedAttitudeQuat(obj)
            gyroBasedAttitudeQuat = obj.GyroBasedAttitudeQuat;
        end
        
        function errorCov = getErrorCov(obj)
            errorCov = obj.ErrorCov;
        end
        
        function dataLog = getDataLog(obj)
            dataLog = obj.DataLog;
        end
        
    end
end

