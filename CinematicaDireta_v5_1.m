function answ = CinematicaDireta_v5_1(pos)
% Calcula a cinemática direta do atuador robótico
% ********************************************************************** %
%          Calculo da cinemática direta do atuador robótico              %
%                                                                        %
%              ET70I - Trabalho De Conclusão De Curso 2                  %
%                                                                        %
% Professor orientador: Rubem Petry Carbente, Dr.                        %
% Professor co-orientador: Winderson Eugenio dos Santos, Dr.             %
%                                                                        %
% Graduandos: Gabrielle Agnez Cordeiro                                   %
%             Ketcho Henrique Fistel                                     %
% ********************************************************************** %
%                           Notas de versão                              %
%                                                                        %
% ********************************************************************** %

ParametrosProj;

x = Elo(1).l*cos(pos(1)) + Elo(2).l*cos(pos(1)+pos(2)) + (Elo(3).l + EF.l)*cos(pos(1)+pos(2)+pos(3));
y = Elo(1).l*sin(pos(1)) + Elo(2).l*sin(pos(1)+pos(2)) + (Elo(3).l + EF.l)*sin(pos(1)+pos(2)+pos(3));
h = pos(4);
answ = [x y h];

end

