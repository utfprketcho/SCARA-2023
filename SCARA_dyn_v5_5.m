% ********************************************************************** %
%               Análise cinemática do atuador robótico                   %
%                                                                        %
%              ET70I - Trabalho De Conclusão De Curso 2                  %
%                                                                        %
% Professor orientador: Rubem Petry Carbente, Dr.                        %
% Professor co-orientador: Winderson Eugenio dos Santos, Dr.             %
%                                                                        %
% Graduandos: Gabrielle Agnez Cordeiro                                   %
%             Ketcho Henrique Fistel                                     %
% ********************************************************************** %

clc;
clear;
close all;

addpath('C:\Program Files\MATLAB\R2022b\toolbox\rvctools\');
startup_rvc;

load("Dados_mat\M86CM80.mat"); % Carrega as curvas do motor 86CM80
load("Dados_mat\M86CM35.mat"); % Carrega as curvas do motor 86CM35
load("Dados_mat\M57CM31.mat"); % Carrega as curvas do motor 57CM31
load("Dados_mat\CS_M22306.mat"); % Carrega as curvas do motor CS_M22306

M(1) = M86CM80; % Atribui as curvas dos motores de cada junta
M(2) = M86CM35;
M(3) = CS_M22306;
M(4) = CS_M22306;

ParametrosProj; % Carrega os parâmetros e constantes do projeto


%% Criação da sequencia de movimentos

% Declaração das poses na áreas detrabalho
pose.repouso = CinematicaInversa_v5_4(0, 0.270, "topo");
pose.montagem = CinematicaInversa_v5_4(0, 0.53852, "base", pi/2);

% Declaração das informações sobre os ingredientes e sequencias de montagem
ParametrosIng;

% Inicialização das variáveis na posição inicial de simulação
pose.anterior = [0 0 0 0];
pose.atual = [0 0 0 0];

% Inicialização dos vetores de trajetória
traj.t = [];
traj.q = [];
traj.qd = [];
traj.qdd = [];
traj.m_carga = [];
traj.I_carga = [];
traj.h_carga = [];
traj.h_pilha = [];

t0 = 0;         % Variável auxiliar para concatenar as parcelas da trajetória
res = 0.0001;    % Período utilizado para calculo dos pontos da trajetória
T(1) = 4;       % Taxa de deslocamento médio da junta 1 (rad/s)
T(2) = 4;       % Taxa de deslocamento médio da junta 2 (rad/s)
T(3) = 4;       % Taxa de deslocamento médio da junta 3 (rad/s)
T(4) = 0.15;    % Taxa de deslocamento médio da junta 4 (m/s)
T(5) = 0.6;     % Taxa de deslocamento médio no plano XY (m/s)
h = 0;          % Posição do EF no eixo Z
h_pilha = 0;    % Altura do empilhamento na área de montagem antes da colocação
h_pilha_pos = 0;    % Altura do empilhamento na área de montagem depois da colocação
vent = cast(0, "logical");  % Variável para indicar atuação da ventosa

% Verifica posições de início de simulação e calcula trajetória para a pose
% de repouso
    % Retorna à pose de repouso
    pose.anterior = pose.atual;
    pose.atual = pose.repouso;
    tempo = 2;
    tp=(0:res:tempo);
    
    % Calcula a trajetória até a pose de repouso
    [q, qd, qdd] = jtraj(pose.anterior, pose.atual, tp);
    
    % Concatena a trajetória do movimento atual com as anteriores
    traj.t = [traj.t (tp+t0)];
    traj.q = [traj.q; q];
    traj.qd = [traj.qd; qd];
    traj.qdd = [traj.qdd; qdd];
    traj.m_carga = [traj.m_carga; ones(numel(tp),1)*0];
    traj.h_carga = [traj.h_carga; ones(numel(tp),1)*0];
    traj.I_carga = [traj.I_carga; zeros(numel(tp),1)];
    traj.h_pilha = [traj.h_pilha; zeros(numel(tp),1)];
    t0 = t0+res+tempo;

% Definição da sequência de montagem
seq.escolhido = seq.xBurger; % xBurger, xSalada ou xBurgerDuplo

% Cálculo da trajetória de montagem
for passo = 1:numel(seq.escolhido)
    for k = 1:6
        switch k
            case 1
                x = ing(seq.escolhido(passo)).pos(1);
                y = ing(seq.escolhido(passo)).pos(2);
                h = ing(seq.escolhido(passo)).h + 0.05;
                h_pilha = h_pilha + ing(seq.escolhido(passo)).h;
                pos = CinematicaInversa_v5_4(x, y, h);
            case 2
                pos(4) = pos(4) - 0.05;
            case 3
                pos(4) = pos(4) + 0.05;
                vent = cast(1, "logical");
            case 4
                pos = CinematicaInversa_v5_4(0, 0.53852, h_pilha + 0.05, pi/2);
            case 5
                pos(4) = pos(4) - 0.05;
            case 6
                pos(4) = pos(4) + 0.05;
                vent = cast(0, "logical");
                h_pilha_pos = h_pilha;
            otherwise
                disp('Erro')
        end

        pose.anterior = pose.atual;
        pose.atual = pos;
        tempo = max(round(rms(CinematicaDireta_v5_1(pose.anterior) - CinematicaDireta_v5_1(pose.atual))/T(5),2), max(abs((pose.anterior(4) - pose.atual(4))*T(5)*10), max(abs(pose.atual - pose.anterior)./T(1:4))));
        tp=(0:res:(tempo));

        % Calcula a trajetória do movimento atual
        [q, qd, qdd] = jtraj(pose.anterior, pose.atual, tp);
        
        % Concatena a trajetória do movimento atual com as anteriores
        traj.t = [traj.t (tp+t0)];
        traj.q = [traj.q; q];
        traj.qd = [traj.qd; qd];
        traj.qdd = [traj.qdd; qdd];

        if vent
            traj.m_carga = [traj.m_carga; ones(numel(tp),1)*ing(seq.escolhido(passo)).m];
            traj.I_carga = [traj.I_carga; ones(numel(tp),1)*(ing(seq.escolhido(passo)).m/2)*0.05^2];
            traj.h_carga = [traj.h_carga; ones(numel(tp),1)*ing(seq.escolhido(passo)).h];
        else
            %traj.m_carga = [traj.m_carga; zeros(numel(tp),1)];
            traj.I_carga = [traj.I_carga; zeros(numel(tp),1)];
            traj.m_carga = [traj.m_carga; ones(numel(tp),1)*0];
            traj.h_carga = [traj.h_carga; ones(numel(tp),1)*0];
        end

        traj.h_pilha = [traj.h_pilha; ones(numel(tp),1)*h_pilha_pos];
            
        t0 = t0+res+tempo;
    end
end

% Retorna à pose de repouso
pose.anterior = pose.atual;
pose.atual = pose.repouso;
tempo = max(round(rms(CinematicaDireta_v5_1(pose.anterior) - CinematicaDireta_v5_1(pose.atual))/T(5),2), max(abs((pose.anterior(4) - pose.atual(4))*T(5)*10), max(abs(pose.atual - pose.anterior)./T(1:4))));
tp=(0:res:tempo);

% Calcula a trajetória até a pose de repouso
[q, qd, qdd] = jtraj(pose.anterior, pose.atual, tp);

% Concatena a trajetória do movimento atual com as anteriores
traj.t = [traj.t (tp+t0)];
traj.q = [traj.q; q];
traj.qd = [traj.qd; qd];
traj.qdd = [traj.qdd; qdd];
%traj.m_carga = [traj.m_carga; zeros(numel(tp),1)];
traj.m_carga = [traj.m_carga; ones(numel(tp),1)*0];
traj.I_carga = [traj.I_carga; zeros(numel(tp),1)];
traj.h_carga = [traj.h_carga; ones(numel(tp),1)*0];
traj.h_pilha = [traj.h_pilha; ones(numel(tp),1)*h_pilha_pos];

% Armazena o tempo total de simulação
traj.T = traj.t(max(size(traj.t)));

%% Obtenção de curvas

% f = figure();
% f.WindowState = "fullscreen";
% trajxy= [];
% for plau = 1:(numel(traj.q(:,1)))
%     plot(traj.t(1:plau), traj.q(1:plau,1:3))
%     xlim([0 18])
%     ylim([-4 3])
%     xlabel('Tempo (s)'); ylabel('Ângulo da junta (rad)');
%     legend(["Junta 1" "Junta 2" "Junta 3"]);
%     anim(plau) = getframe;
% end
% v = VideoWriter('path.avi');
% open(v);
% writeVideo(v,anim);
% close(v);
% movie(anim,numel(traj.q(:,1)))

f = figure();
f.WindowState = "fullscreen";
trajxy= [];
for plau = 1:(numel(traj.q(:,1)))
    plot(traj.t(1:plau), traj.q(1:plau,4))
    xlim([0 18])
    ylim([-0.3 0])
    xlabel('Tempo (s)'); ylabel('Eixo Z (m)');
    anim(plau) = getframe;
end
v = VideoWriter('path.avi');
open(v);
writeVideo(v,anim);
close(v);
movie(anim,numel(traj.q(:,1)))

% figure(1)
% plot(traj.t, traj.q(:,1:3))
% title('Ângulos das juntas na trajetória');
% xlabel('Tempo (s)'); ylabel('Ângulo da junta (rad)');
% legend(["Junta 1" "Junta 2" "Junta 3"]);

% figure(1)
% plot(traj.t, traj.q(:,4))
% title('Posição do efetuador final no eixo Z durante a trajetória');
% xlabel('Tempo (s)'); ylabel('Eixo Z (m)');

% figure(2)
% trajxy= [];
% for plau = 1:numel(traj.q(:,1))
%     trajxy = [trajxy; CinematicaDireta_v5_1(traj.q(plau,:))];
% end
% plot(trajxy(:,1),trajxy(:,2))
% xlabel('Eixo x (m)'); ylabel('Eixo y (m)');

% f = figure();
% f.WindowState = "fullscreen";
% trajxy= [];
% for plau = 1:(numel(traj.q(:,1))/30)
%     trajxy = [trajxy; CinematicaDireta_v5_1(traj.q(plau*30,:))];
%     plot(trajxy(:,1),trajxy(:,2));
%     hold on
%     plot(trajxy(plau,1),trajxy(plau,2),'-o');
%     xlim([-0.7 0.7])
%     ylim([0 0.6])
%     xlabel('Eixo x (m)'); ylabel('Eixo y (m)');
%     anim(plau) = getframe;
%     hold off
% end
% v = VideoWriter('path.avi');
% open(v);
% writeVideo(v,anim);
% close(v);
% movie(anim,numel(traj.q(:,1)))

% figure(3)
% plot(traj.t, traj.qd(:,1:3))
% title('Velocidades angulares das juntas na trajetória');
% xlabel('Tempo (s)'); ylabel('Velocidade angular da junta (rad/s)');
% legend(["Junta 1" "Junta 2" "Junta 3"]);

% figure(4)
% plot(traj.t, traj.qdd(:,1:3))
% title('Acelerações angulares das juntas na trajetória');
% xlabel('Tempo (s)'); ylabel('Aceleração angular da junta (rad/s^2)');
% legend(["Junta 1" "Junta 2" "Junta 3"]);

% figure(5)
% plot(traj.t, [traj.qd(:,4) traj.qdd(:,4)])
% title('Velocidade e aceleração do efetuador final no eixo Z durante a trajetória');
% xlabel('Tempo (s)');
% legend(["Velocidade (m/s)" "Aceleração (m/s^2)"]);


%% Calculo do torque nas juntas -----------------------------------------------------------

T1 = [];
T2 = [];
TA = [];
TB = [];

l1 = Elo(1).l; l2 = Elo(2).l;
m1 = Elo(1).m; m2 = Elo(2).m; m3 = Elo(3).m + EF.m;
r1 = Elo(1).r(1); r2 = Elo(2).r(1); r3 = Elo(3).r(1);
I1 = Elo(1).I(3); I2 = Elo(2).I(3); I3 = Elo(3).I(3) + EF.I(3);
Iporca = 0.00000000001;

I_motor1 = Junta(1).Jm; 
I_motor2 = Junta(2).Jm; 
I_motor3 = Junta(3).Jm;
I_motor4 = Junta(4).Jm;
G1 = Junta(1).G;
G2 = Junta(2).G;
G3 = Junta(3).G;
G4 = Junta(4).GA;
G5 = Junta(4).GB;

g = -9.80665;

th = traj.q;
dth = traj.qd;
ddth = traj.qdd;

th(:,4) = -traj.q(:,4)/G5;
dth(:,4) = -traj.qd(:,4)/G5;
ddth(:,4) = -traj.qdd(:,4)/G5;

% Calcula o torque a ser aplicado às juntas
for cont = 1:numel(traj.t)
    % Atualiza a carga esperada para o instante do cálculo
    m_carga = traj.m_carga(cont);
    Icarga = traj.I_carga(cont);

    % Calcula os torques e forças a partir das equações de dinâmica
    T1(cont) = (m1*r1^2 + I1 + I_motor1*G1^2)*ddth(cont,1) + m2*(l1^2*ddth(cont,1) + r2^2*(ddth(cont,1) + ddth(cont,2)) + l1*r2*(-sin(th(cont,2))*(2*dth(cont,1)*dth(cont,2) + dth(cont,2)^2) + cos(th(cont,2))*(2*ddth(cont,1) + ddth(cont,2)))) + I2*(ddth(cont,1) + ddth(cont,2)) + I_motor2*(ddth(cont,1) + ddth(cont,2)*G2) + (m_carga+m3)*(l1^2*ddth(cont,1) + l2^2*(ddth(cont,1) + ddth(cont,2)) + l1*l2*(cos(th(cont,2))*(2*ddth(cont,1)+ddth(cont,2)) - sin(th(cont,2))*(2*dth(cont,1)*dth(cont,2)+dth(cont,2)^2))) + (m_carga*EF.l + m3*r3)*(l1*cos(th(cont,2)+th(cont,3))*(2*ddth(cont,1) + ddth(cont,2) + ddth(cont,3)) - l1*sin(th(cont,2)+th(cont,3))*(dth(cont,2)+dth(cont,3))*(2*dth(cont,1)+dth(cont,2)+dth(cont,3)) + l2*cos(th(cont,3))*(2*ddth(cont,1) + 2*ddth(cont,2) + ddth(cont,3)) - l2*sin(th(cont,3))*(2*dth(cont,1)*dth(cont,3) + 2*dth(cont,2)*dth(cont,3) + dth(cont,3)^2)) + (Icarga + I3 + m_carga*EF.l^2 + m3*r3^2)*(ddth(cont,1) + ddth(cont,2) + ddth(cont,3)) + Iporca*(ddth(cont,1) + ddth(cont,2) + ddth(cont,3) + ddth(cont,4)) + I_motor4*(ddth(cont,1) + ddth(cont,2) + ddth(cont,3)*G4) + I_motor3*(ddth(cont,1) + ddth(cont,2) + G3*(ddth(cont,3) + ddth(cont,4)));
    T2(cont) = m2*(r2^2*(ddth(cont,2) + ddth(cont,1)) + l1*r2*(-sin(th(cont,2))*dth(cont,2)*dth(cont,1) + cos(th(cont,2))*ddth(cont,1))) + I2*(ddth(cont,2) + ddth(cont,1)) + I_motor2*(ddth(cont,2)*G2^2 + ddth(cont,1)*G2) + (m_carga+m3)*l2^2*(ddth(cont,2)+ddth(cont,1)) + (m_carga*EF.l^2 + m3*r3^2)*(ddth(cont,2) + ddth(cont,1) + ddth(cont,3)) + (m_carga + m3)*l1*l2*(cos(th(cont,2))*ddth(cont,1) - sin(th(cont,2))*dth(cont,1)*dth(cont,2)) + (m_carga*EF.l + m3*r3)*l1*(cos(th(cont,2) + th(cont,3))*ddth(cont,1) - sin(th(cont,2) + th(cont,3))*(dth(cont,1)*dth(cont,2) + dth(cont,1)*dth(cont,3))) + (m_carga*EF.l + m3*r3)*l2*(cos(th(cont,3))*(2*ddth(cont,2) + 2*ddth(cont,1) + ddth(cont,3)) - sin(th(cont,3))*(2*dth(cont,2)*dth(cont,3) + 2*dth(cont,1)*dth(cont,3) + dth(cont,3)^2)) + (Icarga+I3)*(ddth(cont,2) + ddth(cont,1) + ddth(cont,3)) + Iporca*(ddth(cont,2) + ddth(cont,1) + ddth(cont,3) + ddth(cont,4)) + I_motor4*(ddth(cont,2) + ddth(cont,1) + ddth(cont,3)*G4) + I_motor3*(ddth(cont,2) + ddth(cont,1) + G3*(ddth(cont,3) + ddth(cont,4))) + m2*l1*r2*sin(th(cont,2))*(dth(cont,1)^2 + dth(cont,1)*dth(cont,2)) + (m_carga + m3)*l1*l2*sin(th(cont,2))*(dth(cont,1)^2 + dth(cont,1)*dth(cont,2)) + (m_carga*EF.l + m3*r3)*l1*sin(th(cont,2) + th(cont,3))*(dth(cont,1)^2 + dth(cont,1)*dth(cont,2) + dth(cont,1)*dth(cont,3));
    TA(cont) = (Iporca*(ddth(cont,1) + ddth(cont,2) + ddth(cont,3) + ddth(cont,4)) + I_motor3*G3*(ddth(cont,1) + ddth(cont,2) + G3*(ddth(cont,3) + ddth(cont,4))) + (m3+m_carga)*G5*(g + G5*ddth(cont,4)))/G3;
    TB(cont) = ((m_carga*EF.l^2 + m3*r3^2 + Icarga + I3)*(ddth(cont,1) + ddth(cont,2) + ddth(cont,3)) + (m_carga*EF.l + m3*r3)*(l1*(sin(th(cont,2)+th(cont,3))*dth(cont,1)^2 + cos(th(cont,2) + th(cont,3))*ddth(cont,1)) + l2*(sin(th(cont,3))*(dth(cont,1) + dth(cont,2))^2 + cos(th(cont,3))*(ddth(cont,1) + ddth(cont,2)))) + I_motor4*(ddth(cont,1) + ddth(cont,2) + ddth(cont,3)*G4)*G4 - (m3 + m_carga)*G5*(g + G5*ddth(cont,4)))/G4;
end

figure(1);
plot(traj.t,T1/G1,traj.t,T2/G2,traj.t,TA, traj.t,TB);
title('Torque nos motores');
xlabel('Tempo (s)'); ylabel('Torque (N.m)');
legend(["Motor 1" "Motor 2" "Motor A" "Motor B"]);

figure(2);
subplot(411), plot(traj.t,T1/G1);
%title('Torque nos motores');
xlabel('Tempo (s)'); ylabel('Torque (N.m)');
legend(["Motor 1"]);
subplot(412), plot(traj.t,T2/G2);
%title('Torque nos motores');
xlabel('Tempo (s)'); ylabel('Torque (N.m)');
legend(["Motor 2"]);
subplot(413), plot(traj.t,TA);
%title('Torque nos motores');
xlabel('Tempo (s)'); ylabel('Torque (N.m)');
legend(["Motor A"]);
subplot(414), plot(traj.t,TB);
%title('Torque nos motores');
xlabel('Tempo (s)'); ylabel('Torque (N.m)');
legend(["Motor B"]);


% Calcula o torque a ser exercido pelos motores
tau_M = [(T1/Junta(1).G)' (T2/Junta(2).G)' TA' TB'];

% Tempo total de simulação
sim.T = traj.T;

% Aplicação do torque calculado no modelo do simulink
sim.torq.J1 = timetable(seconds(traj.t'), T1(:));
sim.torq.J2 = timetable(seconds(traj.t'), T2(:));
sim.torq.A = timetable(seconds(traj.t'), TA(:));
sim.torq.B = timetable(seconds(traj.t'), TB(:));

% Aplicação da massa variável no modelo do simulink
sim.carga.m = timetable(seconds(traj.t'), traj.m_carga(:));
sim.carga.I = timetable(seconds(traj.t'), traj.I_carga(:));
sim.carga.h = timetable(seconds(traj.t'), traj.h_carga(:));

% Visualização da pilha na área de montagem
sim.h_pilha = timetable(seconds(traj.t'), traj.h_pilha(:));

% Para a malha de controle no modelo do simulink
sim.q.J1 = timetable(seconds(traj.t'), traj.q(:,1));
sim.q.J2 = timetable(seconds(traj.t'), traj.q(:,2));
sim.q.J3 = timetable(seconds(traj.t'), traj.q(:,3));
sim.q.J4 = timetable(seconds(traj.t'), traj.q(:,4));


%% Curva de Torque x Velocidade no motor das juntas

% Curva de velocidades em cada motor [RPM]
qd_M_RPM = [];
for i=1:3 
    qd_M_RPM(:,i) = traj.qd(:,i) * Junta(i).G * 60 / 2 / pi;
    %qd_M_RPM_max(i) = max(abs(traj.qd(:,i))) * Junta(i).G * 60 / 2 / pi;
end
qd_M_RPM(:,4) = (traj.qd(:,4)/G5 + traj.qd(:,3)) * Junta(4).GA * 60 / 2 / pi;

Legenda = ["Motor 1" "Motor 2" "Motor B" "Motor A"];
Titulo = ["CS-M23480 - RMS 6.0A" "CS-M23435 - RMS 4.0A" "CS_M22306 - RMS 3.0A" "CS_M22306 - RMS 3.0A"];

figure(4)
for j = 1:4
    subplot(2,2,j);
    plot(qd_M_RPM(:,j),tau_M(:,j), "Color", [0.9290 0.6940 0.1250]);
    hold on
    plot(M(j).Vmin.RPM, M(j).Vmin.Nm, "Color", "red")
    plot(M(j).Vmed.RPM, M(j).Vmed.Nm, "Color", "blue")
    plot(M(j).Vmax.RPM, M(j).Vmax.Nm, "Color", "green")
    plot(M(j).Vmin.RPM, -M(j).Vmin.Nm, "Color", "red")
    plot(M(j).Vmed.RPM, -M(j).Vmed.Nm, "Color", "blue")
    plot(M(j).Vmax.RPM, -M(j).Vmax.Nm, "Color", "green")
    plot(-M(j).Vmin.RPM, M(j).Vmin.Nm, "Color", "red")
    plot(-M(j).Vmed.RPM, M(j).Vmed.Nm, "Color", "blue")
    plot(-M(j).Vmax.RPM, M(j).Vmax.Nm, "Color", "green")
    plot(-M(j).Vmin.RPM, -M(j).Vmin.Nm, "Color", "red")
    plot(-M(j).Vmed.RPM, -M(j).Vmed.Nm, "Color", "blue")
    plot(-M(j).Vmax.RPM, -M(j).Vmax.Nm, "Color", "green")
    hold off
    title(Titulo(j))
    xlabel(M(j).xlabel)
    ylabel(M(j).ylabel)
    legend([Legenda(j) M(j).legend])
end


%% Obtenção de curvas

% figure(1)
% plot(traj.t, traj.q(:,1:3))
% title('Ângulos das juntas na trajetória');
% xlabel('Tempo (s)'); ylabel('Ângulo da junta (rad)');
% legend(["Junta 1" "Junta 2" "Junta 3"]);

% figure(1)
% plot(traj.t, traj.q(:,4))
% title('Posição do efetuador final no eixo Z durante a trajetória');
% xlabel('Tempo (s)'); ylabel('Eixo Z (m)');

% figure(2)
% trajxy= [];
% for plau = 1:numel(traj.q(:,1))
%     trajxy = [trajxy; CinematicaDireta_v5_1(traj.q(plau,:))];
% end
% plot(trajxy(:,1),trajxy(:,2))
% xlabel('Eixo x (m)'); ylabel('Eixo y (m)');

f = figure();
f.WindowState = "fullscreen";
grafv = [];
graft = [];
j = 4; % Número do gráfico
for plau = 1:(numel(qd_M_RPM(:,1)))

    grafv = [grafv; qd_M_RPM(plau,:)];
    graft = [graft; tau_M(plau,:)];
    plot(grafv(:,j), graft(:,j), "Color", [0.9290 0.6940 0.1250], "LineWidth", 2);
    hold on
    plot(M(j).Vmin.RPM, M(j).Vmin.Nm, "Color", "red", "LineWidth", 2)
    plot(M(j).Vmed.RPM, M(j).Vmed.Nm, "Color", "blue", "LineWidth", 2)
    plot(M(j).Vmax.RPM, M(j).Vmax.Nm, "Color", "green", "LineWidth", 2)
    plot(M(j).Vmin.RPM, -M(j).Vmin.Nm, "Color", "red", "LineWidth", 2)
    plot(M(j).Vmed.RPM, -M(j).Vmed.Nm, "Color", "blue", "LineWidth", 2)
    plot(M(j).Vmax.RPM, -M(j).Vmax.Nm, "Color", "green", "LineWidth", 2)
    plot(-M(j).Vmin.RPM, M(j).Vmin.Nm, "Color", "red", "LineWidth", 2)
    plot(-M(j).Vmed.RPM, M(j).Vmed.Nm, "Color", "blue", "LineWidth", 2)
    plot(-M(j).Vmax.RPM, M(j).Vmax.Nm, "Color", "green", "LineWidth", 2)
    plot(-M(j).Vmin.RPM, -M(j).Vmin.Nm, "Color", "red", "LineWidth", 2)
    plot(-M(j).Vmed.RPM, -M(j).Vmed.Nm, "Color", "blue", "LineWidth", 2)
    plot(-M(j).Vmax.RPM, -M(j).Vmax.Nm, "Color", "green", "LineWidth", 2)
    plot(grafv(plau,j), graft(plau,j), '-*', "MarkerSize", 15);
    hold off
    title(Titulo(j))
    xlabel(M(j).xlabel)
    ylabel(M(j).ylabel)
    legend([Legenda(j) M(j).legend])
    anim(plau) = getframe;

end
%%
v = VideoWriter('TransEF');
open(v);
writeVideo(v,anim);
close(v);

% movie(anim,numel(grafv(:,1)))


%% Curva caminhante posição

j = 4;

f = figure();
f.Units="pixels";
f.PaperPositionMode = "manual";
f.PaperSize = [400 240];
k = plot(traj.t, traj.q(:,j), "LineWidth", 2);
yl = ylim;
for plau = 1:(numel(qd_M_RPM(:,1)))
    plot(traj.t, traj.q(:,j), "LineWidth", 2);
    xlim([(round(1/30*plau,2)-1) (round(1/30*plau,2)+1)])
    ylim(yl)
    f.InnerPosition =  [300 300 400 240];
    hold on
    plot(traj.t(plau), traj.q(plau,j), '-*', "MarkerSize", 15);
    xlim([(round(1/30*plau,2)-1) (round(1/30*plau,2)+1)])
    ylim(yl)
    hold off
    f.InnerPosition =  [300 300 400 240];
    f.PaperSize = [400 240];
    anim(plau) = getframe;
end

v = VideoWriter('Theta ' + string(j) + ' - q');
open(v);
writeVideo(v,anim);
close(v);


f = figure();
f.Units="pixels";
f.PaperPositionMode = "manual";
f.PaperSize = [400 240];
k = plot(traj.t, traj.qd(:,j), "LineWidth", 2);
yl = ylim;
for plau = 1:(numel(qd_M_RPM(:,1)))
    plot(traj.t, traj.qd(:,j), "LineWidth", 2);
    xlim([(round(1/30*plau,2)-1) (round(1/30*plau,2)+1)])
    ylim(yl)
    f.InnerPosition =  [300 300 400 240];
    hold on
    plot(traj.t(plau), traj.qd(plau,j), '-*', "MarkerSize", 15);
    xlim([(round(1/30*plau,2)-1) (round(1/30*plau,2)+1)])
    ylim(yl)
    hold off
    f.InnerPosition =  [300 300 400 240];
    f.PaperSize = [400 240];
    anim(plau) = getframe;
end

v = VideoWriter('Theta ' + string(j) + ' - qd');
open(v);
writeVideo(v,anim);
close(v);


f = figure();
f.Units="pixels";
f.PaperPositionMode = "manual";
f.PaperSize = [400 240];
k = plot(traj.t, traj.qdd(:,j), "LineWidth", 2);
yl = ylim;
for plau = 1:(numel(qd_M_RPM(:,1)))
    plot(traj.t, traj.qdd(:,j), "LineWidth", 2);
    xlim([(round(1/30*plau,2)-1) (round(1/30*plau,2)+1)])
    ylim(yl)
    f.InnerPosition =  [300 300 400 240];
    hold on
    plot(traj.t(plau), traj.qdd(plau,j), '-*', "MarkerSize", 15);
    xlim([(round(1/30*plau,2)-1) (round(1/30*plau,2)+1)])
    ylim(yl)
    hold off
    f.InnerPosition =  [300 300 400 240];
    f.PaperSize = [400 240];
    anim(plau) = getframe;
end

v = VideoWriter('Theta ' + string(j) + ' - qdd');
open(v);
writeVideo(v,anim);
close(v);