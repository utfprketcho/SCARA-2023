% ********************************************************************** %
%                  Informações sobre os ingredientes                     %
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


% Ingrediente 1 (Base do pão)
ing(1).nome = "Base do pão";
ing(1).m = 0.03;                % Massa do ingrediente
ing(1).h = 0.015;               % Altura do ingrediente
ing(1).I = 0.00004;             % Momento de inércia em relação ao ponto de fixação em relação ao eixo Z
ing(1).pos = [-0.520, 0.140];   % Posição do ingrediente no espaço de trabalho

% Ingrediente 2 (Tampa do pão)
ing(2).nome = "Tampa do pão";
ing(2).m = 0.03;                % Massa do ingrediente
ing(2).h = 0.015;               % Altura do ingrediente
ing(2).I = 0.00004;             % Momento de inércia em relação ao ponto de fixação em relação ao eixo Z
ing(2).pos = [-0.250, 0.140];   % Posição do ingrediente no espaço de trabalho

% Ingrediente 3 (Queijo)
ing(3).nome = "Queijo";
ing(3).m = 0.01;                % Massa do ingrediente
ing(3).h = 0.001;               % Altura do ingrediente
ing(3).I = 0.00001;             % Momento de inércia em relação ao ponto de fixação em relação ao eixo Z
ing(3).pos = [-0.3808, 0.3808]; % Posição do ingrediente no espaço de trabalho

% Ingrediente 4 (Hamburguer)
ing(4).nome = "Hamburguer";
ing(4).m = 0.180;               % Massa do ingrediente
ing(4).h = 0.012;               % Altura do ingrediente
ing(4).I = 0.00023;             % Momento de inércia em relação ao ponto de fixação em relação ao eixo Z
ing(4).pos = [0.3808, 0.3808];  % Posição do ingrediente no espaço de trabalho

% Ingrediente 5 (Alface)
ing(5).nome = "Alface";
ing(5).m = 0.01;                % Massa do ingrediente
ing(5).h = 0.001;               % Altura do ingrediente
ing(5).I = 0.00001;             % Momento de inércia em relação ao ponto de fixação em relação ao eixo Z
ing(5).pos = [0.250, 0.140];    % Posição do ingrediente no espaço de trabalho

% Ingrediente 6 (Tomate)
ing(6).nome = "Tomate";
ing(6).m = 0.03;                % Massa do ingrediente
ing(6).h = 0.005;               % Altura do ingrediente
ing(6).I = 0.00001;             % Momento de inércia em relação ao ponto de fixação em relação ao eixo Z
ing(6).pos = [0.520, 0.140];    % Posição do ingrediente no espaço de trabalho



% Sequências de montagem predefinidas -------------------------------------

% Sanduiche tipo "X-Burger"
% Base do pão, Queijo, Hamburguer, Queijo e Tampa do pão
seq.xBurger = [1 3 4 3 2];

% Sanduiche tipo "X-Salada"
% Base do pão, Hamburguer, Queijo, Alface, Tomate e Tampa do pão
seq.xSalada = [1 4 3 5 6 2];

% Sanduiche tipo "X-Burger Duplo"
% Base do pão, Queijo, Hamburguer, Queijo, Hamburguer, Queijo e Tampa do pão
seq.xBurgerDuplo = [1 3 4 3 4 3 2];
