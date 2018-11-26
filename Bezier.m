% función que devuelve la curva de Bèzier para C, n, u
% Variables de entrada:
%       C: coordendas de control
%       n: grado de la curva
%       u: valor en el que se evalúa
function Bez = Bezier(C,n,u)

Bez = 0;


    
for i=1:length(C)
    Bez = Bez + C(i)*Berstein(n,i-1,u);
end