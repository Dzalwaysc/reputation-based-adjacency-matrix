function [new_zeta, new_sigma] = weight_node(Zeta, Sigma, Neighbor, N, Delta_STC, Delta_VTV, index, dt, Loop)

Ni = Neighbor{index};
sigma = Sigma{index}(Loop,:);  
zeta = Zeta{index}(Loop,:);

%% 参数
mu = 5;

%% 计算相对位置
h = 0;
xita = 1.2;
%% 计算trustworthiness
g = zeros(1,N);
dsigma = zeros(1,N);
for j=Ni
    h = norm(Delta_STC{j}(Loop,:)+Delta_VTV{j}(Loop,:))^2;
    g(j) = exp( -log(h+1) / (sigma(j)^2 + xita) );
    dsigma(j) = - mu * sigma(j) * (g(j) - zeta(j) ); 
end

%% 计算weight node
dzeta = zeros(1,N);
for j=1:N
    sum = 0;
    for k=Ni
        sum = sum + Zeta{k}(end,j) - zeta(j);
    end
    %% 若j艇属于邻接艇
    if numel(find(Ni==j)) == 1
        dzeta(j) = 2*sum + 15*(g(j) - zeta(j));
    else
        dzeta(j) = 2*sum;
    end
end

new_zeta = zeta + dzeta*dt;
new_sigma = sigma + dsigma*dt;
