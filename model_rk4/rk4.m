function xout = rk4(f, dt, xk, ui, i)
f1 = f(xk,ui(:,i)');
f2 = f(xk+(dt/2).*f1, ((ui(:,i)+ui(:,i+1))./2)');
f3 = f(xk+(dt/2).*f2, ((ui(:,i)+ui(:,i+1))./2)');
f4 = f(xk+dt.*f3, ui(:,i+1)');
xout = xk + (dt/6).*(f1+2.*f2+2.*f3+f4);

% function xout = rk4(f, dt, xk)
% f1 = f(xk);
% f2 = f(xk+(dt/2).*f1);
% f3 = f(xk+(dt/2).*f2);
% f4 = f(xk+dt.*f3);
% xout = xk + (dt/6).*(f1+2.*f2+2.*f3+f4);
