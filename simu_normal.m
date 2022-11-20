clear all

%% Graph contribution
% Neighbor{1} = [2,3,4,5]; Bias{1} = [0, 0, 0];
% Neighbor{2} = [1,3,4,5]; Bias{2} = [-10, 10, 0];
% Neighbor{3} = [1,2,4,5]; Bias{3} = [-20, 10, 0]; 
% Neighbor{4} = [1,2,3,5]; Bias{4} = [-20, -10, 0];
% Neighbor{5} = [1,2,3,4]; Bias{5} = [-10, -10, 0];

Neighbor{1} = [2]; 
Neighbor{2} = [1,3]; 
Neighbor{3} = [2,4];
Neighbor{4} = [3,5];
Neighbor{5} = [2,4];

Configuration{1} = [0, 0, 0]; 
Configuration{2} = [0, 10, 0];
Configuration{3} = [10, 0, 0];
Configuration{4} = [0, -10, 0];
Configuration{5} = [-10, 0, 0];



%% 第一次循环，即t = 0, 所有参数初始化
Loop = 1; N = 5; dt = 0.001;

init_eta(1,:) = [0, 0, 0]; init_eta(2,:) = [-2, 15, 330*pi/180]; init_eta(3,:) = [5, -5, 30*pi/180];
init_eta(4,:) = [-10, -8, 30*pi/180]; init_eta(5,:) = [-20, 8, 10*pi/180];
% init_State(1,:) = [0, 0, 0]; init_State(2,:) = [0, 10, 0]; init_State(3,:) = [10, 0, 0];
% init_State(4,:) = [0, -10, 0]; init_State(5,:) = [-10, 0, 0];

for i=1:N
    init_vartheta(i,:) = [0, 0, 0];
    tau{i}(Loop,:) = [0, 0, 0];
    delta_eta{i}(Loop,:) = [0, 0, 0]; delta_Delta_eta{i}(Loop,:) = [0, 0, 0]; d_delta_Delta_eta{i}(Loop,:) = [0, 0, 0]; 
    delta_omega{i}(Loop,:) = [0, 0, 0]; delta_Delta_omega{i}(Loop,:) = [0, 0, 0];
    hat_Delta_STC{i}(Loop,:) = [0, 0, 0];
end

for i=1:N
    [eta{i}(Loop,:), deta{i}(Loop,:), vartheta{i}(Loop,:), uncertainty{i}(Loop,:)] = plant(init_eta(i,:)', init_vartheta(i,:)', tau{i}(Loop,:)', dt, 1);
    Sigma{i}(Loop,:) = [0, 0, 0, 0, 0];
    Zeta{i}(Loop,:) = [0, 0, 0, 0, 0];
    for j=Neighbor{i}
        % Sigma{i}(Loop,j) = 1;
        Zeta{i}(Loop,j) = 1;
    end
end

T = 0*dt;
if T>15
    Delta_STC{1}(Loop,:) = [0, 0, 0]; Delta_STC{2}(Loop,:) = [0, 0, 0]; Delta_STC{3}(Loop,:) = [0, 0, 0]; Delta_STC{4}(Loop,:) = [3*(T-10)+5, 2*(T-10)+5, 0.01*pi*(T-10)]; Delta_STC{5}(Loop,:) = [0, 0, 0];
    d_Delta_STC{1}(Loop,:) = [0, 0, 0]; d_Delta_STC{2}(Loop,:) = [0, 0, 0]; d_Delta_STC{3}(Loop,:) = [0, 0, 0]; d_Delta_STC{4}(Loop,:) = [3, 2, 0.01*pi]; d_Delta_STC{5}(Loop,:) = [0, 0, 0];
    Delta_VTV{1}(Loop,:) = [0, 0, 0]; Delta_VTV{2}(Loop,:) = [0, 0, 0]; Delta_VTV{3}(Loop,:) = [0, 0, 0]; Delta_VTV{4}(Loop,:) = [10, 10, 10]; Delta_VTV{5}(Loop,:) = [0, 0, 0];
else
    Delta_STC{1}(Loop,:) = [0, 0, 0]; Delta_STC{2}(Loop,:) = [0, 0, 0]; Delta_STC{3}(Loop,:) = [0, 0, 0]; Delta_STC{4}(Loop,:) = [0, 0, 0]; Delta_STC{5}(Loop,:) = [0, 0, 0];
    d_Delta_STC{1}(Loop,:) = [0, 0, 0]; d_Delta_STC{2}(Loop,:) = [0, 0, 0]; d_Delta_STC{3}(Loop,:) = [0, 0, 0]; d_Delta_STC{4}(Loop,:) = [0, 0, 0]; d_Delta_STC{5}(Loop,:) = [0, 0, 0];
    Delta_VTV{1}(Loop,:) = [0, 0, 0]; Delta_VTV{2}(Loop,:) = [0, 0, 0]; Delta_VTV{3}(Loop,:) = [0, 0, 0]; Delta_VTV{4}(Loop,:) = [0, 0, 0]; Delta_VTV{5}(Loop,:) = [0, 0, 0];
end

for index = 1:N
    [eta_{index}(Loop,:), deta_{index}(Loop,:), eta__{index}(Loop,:), deta__{index}(Loop,:)] ...
        = state_underHCFs(eta, deta, Delta_STC, Delta_VTV, index, 1);
    
    [Delta_eta{i}(Loop,:), Delta_omega{i}(Loop,:)] ...
        = uncertainties(d_Delta_STC, Delta_STC, eta_, eta, uncertainty, tau, index, 1);  
end

%% 后续迭代
AT = 20;
Time = linspace(0, AT+dt, AT/dt+1); Tk = 1;
for i=1:AT/dt
    T = i*dt;
    % Tk = Loop;
    if mod(Loop, 500) == 0
        Tk = Loop;
    end
    %% follower
    for index = 2:5
        [tau{index}(Loop+1,:), hat_Delta_STC{index}(Loop+1,:)] = control(Zeta, Neighbor, Configuration, eta_, deta_, eta__, deta__, delta_Delta_eta, d_delta_Delta_eta, delta_Delta_omega, index, Loop);
        
        [Zeta{index}(Loop+1,:), Sigma{index}(Loop+1,:)] ...
            = weight_node(Zeta, Sigma, Neighbor, N, Delta_STC, Delta_VTV, index, dt, Loop);
        
        [delta_eta{index}(Loop+1,:), delta_Delta_eta{index}(Loop+1,:), d_delta_Delta_eta{index}(Loop+1,:), delta_omega{index}(Loop+1,:), delta_Delta_omega{index}(Loop+1,:)] ...
            = adaptive(eta_, deta_, delta_eta, delta_Delta_eta, delta_omega, delta_Delta_omega, tau, index, Loop, dt);
        
        [eta{index}(Loop+1,:), deta{index}(Loop+1,:), vartheta{index}(Loop+1,:), uncertainty{index}(Loop+1,:)] ...
            = plant(eta{index}(Loop,:)', vartheta{index}(Loop,:)', tau{index}(Loop+1,:)', dt, Loop);
    end
        
    
    xx = 2*T; xxd = 1; xxdd = 0;
    yy = 20+20*sin(0.05*xx+3*pi/2); yyd = 20*0.05*2*cos(0.05*xx+3*pi/2); yydd = -20*0.05*0.01*2*sin(0.05*xx+3*pi/2);
    fai = atan2(yyd, xxd); faid = (-xxdd*yyd + xxd*yydd)/(xxd^2+yyd^2);
    if fai < 0
        fai = fai + 2*pi;
    end
    
    eta{1}(Loop+1,:) = [xx, yy, fai]; vartheta{1}(Loop+1,:) = [yyd,xxd, faid]; deta{1}(Loop+1,:) = [yyd, xxd, faid]; 
    uncertainty{1}(Loop+1,:) = [1, 0, 0]; tau{1}(Loop+1,:) = [0, 0, 0];
    [Zeta{1}(Loop+1,:), Sigma{1}(Loop+1,:)] = weight_node(Zeta, Sigma, Neighbor, N, Delta_STC, Delta_VTV, 1, dt, Loop);
    delta_eta{1}(Loop+1,:) = [0, 0, 0]; delta_Delta_eta{1}(Loop+1,:) = [0, 0, 0]; d_delta_Delta_eta{1}(Loop+1,:) = [0, 0, 0];  delta_omega{1}(Loop+1,:) = [0, 0, 0]; delta_Delta_omega{1}(Loop+1,:) = [0, 0, 0];
   
    if T>15 
        Delta_STC{1}(Loop+1,:) = [0, 0, 0]; Delta_STC{2}(Loop+1,:) = [0, 0, 0]; Delta_STC{3}(Loop+1,:) = [0, 0, 0]; Delta_STC{4}(Loop+1,:) = [3*(T-10)+5, 2*(T-10)+5, 0.01*pi*(T-10)]; Delta_STC{5}(Loop+1,:) = [0, 0, 0];
        d_Delta_STC{1}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{2}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{3}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{4}(Loop+1,:) = [3, 2, 0.01*pi]; d_Delta_STC{5}(Loop+1,:) = [0, 0, 0];
        Delta_VTV{1}(Loop+1,:) = [0, 0, 0]; Delta_VTV{2}(Loop+1,:) = [0, 0, 0]; Delta_VTV{3}(Loop+1,:) = [0, 0, 0]; Delta_VTV{4}(Loop+1,:) = [10, 10, 10]; Delta_VTV{5}(Loop+1,:) = [0, 0, 0];
    else
        Delta_STC{1}(Loop+1,:) = [0, 0, 0]; Delta_STC{2}(Loop+1,:) = [0, 0, 0]; Delta_STC{3}(Loop+1,:) = [0, 0, 0]; Delta_STC{4}(Loop+1,:) = [0, 0, 0]; Delta_STC{5}(Loop+1,:) = [0, 0, 0];
        d_Delta_STC{1}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{2}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{3}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{4}(Loop+1,:) = [0, 0, 0]; d_Delta_STC{5}(Loop+1,:) = [0, 0, 0];
        Delta_VTV{1}(Loop+1,:) = [0, 0, 0]; Delta_VTV{2}(Loop+1,:) = [0, 0, 0]; Delta_VTV{3}(Loop+1,:) = [0, 0, 0]; Delta_VTV{4}(Loop+1,:) = [0, 0, 0]; Delta_VTV{5}(Loop+1,:) = [0, 0, 0];
    end
    
    for index = 1:N
        [eta_{index}(Loop+1,:), deta_{index}(Loop+1,:), eta__{index}(Loop+1,:), deta__{index}(Loop+1,:)] ...
            = state_underHCFs(eta, deta, Delta_STC, Delta_VTV, index, Loop);

        [Delta_eta{index}(Loop+1,:), Delta_omega{index}(Loop+1,:)] ...
            = uncertainties(d_Delta_STC, Delta_STC, eta_, eta, uncertainty, tau, index, Loop);  
    end
    Loop = Loop+1;
    fprintf('The time is %f\n', T);
end
