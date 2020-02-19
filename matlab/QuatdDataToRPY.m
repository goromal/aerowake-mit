function [phi, tht, psi] = QuatdDataToRPY(wdata, xdata, ydata, zdata)

n = length(wdata);
phi = zeros(size(wdata));
tht = zeros(size(wdata));
psi = zeros(size(wdata));

yaw_prev = 0.0;
yaw_buff = 0.1;
for i = 1:1:n
    q = Quatd([wdata(i); xdata(i); ydata(i); zdata(i)]);
    phi(i) = q.roll();
    tht(i) = q.pitch();
    psi(i) = q.yaw();
    % some rudimentary angle smoothing, keeping it around +pi if there's
    % +-pi ambiguity at the beginning of the data
    if i == 1 && psi(i) < -1.0 * (pi-yaw_buff/2)
        psi(i) = psi(i) + 2*pi;
    elseif abs(psi(i) - yaw_prev) >= 2*pi-yaw_buff
        psi(i) = psi(i) + sign(yaw_prev - psi(i)) * 2*pi;
    end
    yaw_prev = psi(i);
end