% ʱ�������λ���
function [xn, vn] = IntFcn_Time2(t, an)
vn = cumtrapz(t, an);  % ʱ�������λ���
vn = vn - repmat(mean(vn), size(vn,1), 1);
xn = cumtrapz(t, vn);
xn = xn - repmat(mean(xn), size(xn,1), 1);
end