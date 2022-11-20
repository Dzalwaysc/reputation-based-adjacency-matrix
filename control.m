function [tau, hat_Delta_STC] = control(Zeta, Neighbor, Configuration, eta_, deta_, eta__, deta__, delta_Delta_eta, d_delta_Delta_eta, delta_Delta_omega, index, Loop)

Ni = Neighbor{index};
zeta = Zeta{index}(Loop,:);

%% 计算辅助变量
z1 = 0;
z2 = 0;
hat_Delta_STC = -(d_delta_Delta_eta{index}(Loop,:) + delta_Delta_eta{index}(Loop,:));

for j=Ni
    etaErr = zeta(j) * (eta_{index}(Loop,:) - hat_Delta_STC - Configuration{index} - eta__{j}(Loop,:) + Configuration{j});
    if etaErr(3) > pi
        etaErr(3) = etaErr(3) - 2*pi;
    elseif etaErr(3) < -pi
        etaErr(3) = etaErr(3) + 2*pi;
    end
    z1 = z1 + etaErr;
    z2 = z2 + zeta(j) * ( deta_{index}(Loop,:) - hat_Delta_STC - deta__{j}(Loop,:) );
end

%% 计算控制器
varsigma = diag([16,16,16]); gamma = diag([5,5,5]);
R = reshape( obtain_R(eta_{index}(Loop,:)), 3, 3 );
M = 0.5*reshape( obtain_M(), 3, 3 );

tau = -M*R'*(delta_Delta_omega{index}(end,:)' + d_delta_Delta_eta{index}(end,:)' + varsigma*z2' + varsigma*gamma*z1');

[tau(1), ~] = constraint_input(tau(1), 500);
[tau(2), ~] = constraint_input(tau(2), 500);
[tau(3), ~] = constraint_input(tau(3), 300);
