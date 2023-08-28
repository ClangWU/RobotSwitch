% 时域内梯形积分
function [xn, vn] = IntFcn_Time2(t, an)
vn = cumtrapz(t, an);  % 时域内梯形积分
vn = vn - repmat(mean(vn), size(vn,1), 1);
xn = cumtrapz(t, vn);
xn = xn - repmat(mean(xn), size(xn,1), 1);
end