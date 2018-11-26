% función que devuelve el polinomio de Berstein para n, i, u
% Variables de entrada:
%       n: grado del polinomio
%       i: elemento de la base
%       u: valor en el que se evalúa
function B = Berstein(n,i,u)

if n<0
    B = 0;
else
    B = factorial(n)/(factorial(i)*factorial(n-i))*u^i*(1-u)^(n-i);
end