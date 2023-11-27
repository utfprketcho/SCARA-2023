function answ = CinematicaInversa_v5_4(x, y, alt3, ang3)
% Calcula a cinemática inversa do atuador robótico
% ********************************************************************** %
%          Calculo da cinemática inversa do atuador robótico             %
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
% * Incluída a terceira junta rotacional                                 %
%                                                                        %
% * Alterado o formato da resposta da função para corresponder ao        %
% atual formato de pose utilizado no projeto                             %
%                                                                        %
% * Alterada a referência dos angulos de juntas                          %
%                                                                        %
% ********************************************************************** %
arguments
    x
    y
    alt3 = 0
    ang3 = 0
end

ParametrosProj;

xp = x - EF.l*cos(ang3);
yp = y - EF.l*sin(ang3);

cos_th2 = (yp^2 + xp^2 - Elo(2).l^2 - Elo(1).l^2) / (2*Elo(1).l*Elo(2).l);
sin_th2 = sqrt(1-cos_th2^2);
th2 = atan2(sin_th2, cos_th2);

a = atan2(yp, xp);
b = real(acos((yp^2 + xp^2 + Elo(1).l^2 - Elo(2).l^2) / (2*Elo(1).l*sqrt(xp^2 + yp^2))));

th1 = a - b;

th3 = ang3 - th1 - th2;


% Para implementação futura ----------------------------------------------

h3_base = -Base.d - Elo(1).d - Elo(2).d - Elo(3).d - EF.d1;
h3_topo = -EF.d2;

if isnumeric(alt3)
    if alt3 >= (h3_topo-h3_base)
        h3 = h3_topo;
    elseif alt3 <= 0
        h3 = h3_base;
    else
        h3 = h3_base + alt3;
    end
elseif alt3 == "topo"
    h3 = h3_topo;
elseif alt3 == "base"
    h3 = h3_base;
else
end
% -----------------------------------------------------------------------

answ = [th1 th2 th3 h3];

end

