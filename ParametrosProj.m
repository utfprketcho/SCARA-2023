% ********************************************************************** %
%                 Parâmetros e constantes de projeto                     %
%                                                                        %
%              ET70I - Trabalho De Conclusão De Curso 2                  %
%                                                                        %
% Professor orientador: Rubem Petry Carbente, Dr.                        %
% Professor co-orientador: Winderson Eugenio dos Santos, Dr.             %
%                                                                        %
% Graduandos: Gabrielle Agnez Cordeiro                                   %
%             Ketcho Henrique Fistel                                     %
% ********************************************************************** %

%  Todas os comprimentos são dados em m
%  Todos os ângulos são dados em rad
%  Todas as massas são dadas em kg
%  Todos os momentos de inércia são dados em kg*m^2

% Estrutura base do robô

Base.d = 0.272;         % Altura da base


% Elo 1

Elo(1).l = 0.360;       % Comprimento da primeira estrutura rotativa (eixo X)
Elo(1).d = 0.0563;      % Altura da primeira estrutura rotativa (eixo Z)
%Elo(1).theta = 0;       % Variável independente - Rotação da primeira estrutura rotativa (eixo Z)
Elo(1).alpha = 0;       % Rotação da primeira estrutura rotativa (eixo X)
Elo(1).m = 5.084212;    % Massa do Elo
Elo(1).r = [0.065297 -2e-05 0.045362];      % Coordenadas do CG em relação à origem do elo
Elo(1).I = [0.015064 0.101589 0.097688];    % Momentos de inércia principais em relação ao centro de gravidade


% Elo 2

Elo(2).l = 0.320;       % Comprimento da segunda estrutura rotativa (eixo X)
Elo(2).d = -0.0325;     % Altura da segunda estrutura rotativa (eixo Z)
%Elo(2).theta = 0;      % Variável independente - Rotação da segunda estrutura rotativa (eixo Z)
Elo(2).alpha = 0;       % Rotação da segunda estrutura rotativa (eixo X)
Elo(2).m = 6.38486;     % Massa do Elo
Elo(2).r = [0.135119 -2e-06 0.061128];      % Coordenadas do CG em relação à origem do elo
Elo(2).I = [0.015714 0.13044 0.119861];     % Momentos de inércia principais em relação ao centro de gravidade


% Elo 3

Elo(3).l = 0;           % Comprimento da terceira estrutura rotativa (eixo X)
Elo(3).d = 0;           % Altura da terceira estrutura rotativa (eixo Z)
%Elo(3).theta = 0;      % Variável independente
Elo(3).alpha = 0;       % Rotação da terceira estrutura rotativa (eixo X)
Elo(3).m = 0.26131965;  % Massa do Elo
Elo(3).r = [0 0 0.20144030];                    % Coordenadas do CG em relação à origem do elo
Elo(3).I = [0.00353766 0.00353765 0.00001173];  % Momentos de inércia principais em relação ao centro de gravidade


% Efetuador final

EF.l = 0.02;            % Comprimento da estrutura do efetuador final
EF.d1 = -0.019;         % Deslocamento em Z entre o ponto de fixação do EF e de ancoragem da carga
EF.d2 = 0.025;          % Deslocamento em Z entre o ponto de fixação e ponto mais alto do EF, que evitará colisões com o Elo 3
EF.m = 0.00732235;      % Massa do Efetuador final
EF.I = [0.00000133 0.00000286 0.00000275];      % Momentos de inércia principais em relação ao ponto de fixação


% Junta 1 (União entre a base fixa e o Elo 1)
% Tipo: Rotativa
Junta(1).G = 11;            % Coeficiente de redução
Junta(1).Jm = 2.5*10^-4;    % Momento de inércia do motor


% Junta 2 (União entre o Elo 1 e o Elo 2)
% Tipo: Rotativa
Junta(2).G = 25;            % Coeficiente de redução
Junta(2).Jm = 1.0*10^-2;      % Momento de inércia do motor


% Junta 3 (União entre o Elo 2 e a Haste roscada)
% Tipo: Rotativa
Junta(3).G = 2;             % Coeficiente de redução
Junta(3).Jm = 0.131*10^-4;  % Momento de inércia do motor


% Junta 4 (União entre o Elo 2 e a Haste roscada)
% Tipo: Prismática
Junta(4).GA = 2;
Junta(4).GB = 0.016/(2*pi);
Junta(4).G = Junta(4).GA / Junta(4).GB; % Coeficiente de redução (considerando 18 dentes no motor)
Junta(4).Jm = 0.69*10^-4;   % Momento de inércia do motor