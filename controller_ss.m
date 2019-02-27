classdef controller_ss
    properties
        K
        W
        cost_old
        cost_new
    end
    methods
        function obj = controller_ss(K, W, cost_old, cost_new)
            obj.K = K;
            obj.W = W;
            obj.cost_old = cost_old;
            obj.cost_new = cost_new;
        end
    end
end