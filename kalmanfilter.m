%====================Kanlman Filter==================================

% Q=[1 0 0;0 5 0;0 0 13]; % Change Q & R to determine how accurate the estimates are, w.r.t model and measurments noise.
% R=[20 0;0 20];
% P0=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 100]; %initial P



Xk_k1=A*X0+B*U_pass;

Pk_k1=A*P0*A'+Q;
Kk=Pk_k1*C'*inv(C*Pk_k1*C'+R);
Xk_k=Xk_k1+Kk*(y0-(C*Xk_k1));
Pk_k=(I-Kk*C)*Pk_k1;
X0=Xk_k; %update state estimate for next iteration
P0=Pk_k;  %update P matrix for next iteration

% Xhat=[Xhat;X0]; %store estimates

