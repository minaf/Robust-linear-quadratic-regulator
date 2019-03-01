classdef controller_structure
    %state-space form of inverted pendulum
    properties
        T1
        T2
        W
        Z
        X
    end
    methods
        function obj = controller_structure(T1, T2, W, Z, X)
            obj.T1 = T1;
            obj.T2 = T2;
            obj.W = W;
            obj.Z = Z;
            obj.X = X;
        end
    end
end