classdef myClass
    %MYCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Name
        Value
    end
    
    methods
        function obj = myClass(Name,Value)
%             construct method
            if (nargin>0)        
            obj.Name = Name;
            obj.Value = Value;               
            end
        end
        function result = SinTA(obj,r)
            result =  sin(r);%sin(theta);
        end
    end
    
end

