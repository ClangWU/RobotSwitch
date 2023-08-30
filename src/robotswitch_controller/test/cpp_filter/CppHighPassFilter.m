classdef CppHighPassFilter < handle
    properties
        prev_input1
        prev_output1
        prev_input2
        prev_output2
        a0
        a1
        a2
        b0
        b1
        b2
        prev_input1_3f
        prev_output1_3f
        prev_input2_3f
        prev_output2_3f
        order_
    end
    
    methods
        function obj = CppHighPassFilter()
            obj.prev_input1 = 0;
            obj.prev_output1 = 0;
            obj.prev_input2 = [0; 0];
            obj.prev_output2 = [0; 0];
            obj.prev_input1_3f = zeros(3,1);
            obj.prev_output1_3f = zeros(3,1);
            obj.prev_input2_3f = zeros(3,2);
            obj.prev_output2_3f = zeros(3,2);
            obj.order_ = 1;
        end
        
        function InitFilter(obj, order, sampleRate, cutoffFreq)
            obj.order_ = order;
            if obj.order_ == 1
                dt = 1 / sampleRate;
                rc = 1 / (2 * pi * cutoffFreq);
                alpha = rc / (rc + dt);
                obj.a0 = 1;
                obj.a1 = -alpha;
                obj.b0 = alpha;
                obj.b1 = -alpha;
            else
                omega = 2 * pi * cutoffFreq / sampleRate;
                alpha = sin(omega) / (2 * sqrt(2));
                obj.a0 = 1 + alpha;
                obj.b0 = (1 + cos(omega)) / (2 * obj.a0);
                obj.b1 = -(1 + cos(omega)) / obj.a0;
                obj.b2 = obj.b0;
                obj.a1 = -2 * cos(omega) / obj.a0;
                obj.a2 = (1 - alpha) / obj.a0;
            end
        end
        
        function output = process(obj, input)
            if isnumeric(input) && numel(input) == 1  % Scalar input
                if obj.order_ == 1
                    output = obj.b0 * input + obj.b1 * obj.prev_input1 ...
                           - obj.a1 * obj.prev_output1;
                    obj.prev_input1 = input;
                    obj.prev_output1 = output;
                else
                    output = obj.b0 * input + obj.b1 * obj.prev_input2(1) ...
                           + obj.b2 * obj.prev_input2(2) ...
                           - obj.a1 * obj.prev_output2(1) ...
                           - obj.a2 * obj.prev_output2(2);
                    obj.prev_input2(2) = obj.prev_input2(1);
                    obj.prev_input2(1) = input;
                    obj.prev_output2(2) = obj.prev_output2(1);
                    obj.prev_output2(1) = output;
                end
                
            elseif isnumeric(input) && numel(input) == 3  % 3D Vector input
                if obj.order_ == 1
                    output = obj.b0 .* input + obj.b1 .* obj.prev_input1_3f ...
                           - obj.a1 .* obj.prev_output1_3f;
                    obj.prev_input1_3f = input;
                    obj.prev_output1_3f = output;
                else
                    output = obj.b0 .* input ...
                           + obj.b1 .* obj.prev_input2_3f(:,1) ...
                           + obj.b2 .* obj.prev_input2_3f(:,2) ...
                           - obj.a1 .* obj.prev_output2_3f(:,1) ...
                           - obj.a2 .* obj.prev_output2_3f(:,2);
                    obj.prev_input2_3f(:,2) = obj.prev_input2_3f(:,1);
                    obj.prev_input2_3f(:,1) = input;
                    obj.prev_output2_3f(:,2) = obj.prev_output2_3f(:,1);
                    obj.prev_output2_3f(:,1) = output;
                end
            else
                error('Input type not recognized. Use scalar or 3x1 vector.');
            end
        end
    end
end
