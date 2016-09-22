classdef SampleClassDistribute
    %SAMPLECLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Symbol
        OpeningPrice
        Y
    end
    properties(GetAccess = 'protected', SetAccess = 'protected')
       % public read access, but private write access.
       m_protected
    end

    properties(GetAccess = 'private', SetAccess = 'public')
       % private read and write access
       m_private
    end
    methods
        function obj  = SampleClassDistribute(Symbol,OpeningPrice,Y)
            %class constructor
            if (nargin>0)
                obj.Symbol = Symbol;
                obj.OpeningPrice = OpeningPrice;
                obj.Y = Y;
            end
        end % compute method
        obj = rollY(obj,numY);
        obj = multirollY(obj,numY);
        function xB = SqrtB(inputA)
         xB = inputA;
        end % compute method
    end
    
end

