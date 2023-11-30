clear
clc

% --------------------- Exercicios MATLAB 2A a) ---------------%

% i) 
ang1 = pi/18
ang2 = pi/9
ang3 = pi/6

% --------------- Angulos de euler X-Y-Z em matriz rotacional ---------%

Matrix3 = [cos(ang1) -sin(ang1) 0; sin(ang1) cos(ang1) 0; 0 0 1] 
Matrix2 = [cos(ang2) 0 sin(ang2); 0 1 0; -sin(ang2) 0 cos(ang2)]
Matrix1 = [1 0 0; 0 cos(ang3) -sin(ang3); 0 sin(ang3) cos(ang3)]

Result = Matrix3*Matrix2*Matrix1

Tot = rpy2tr(ang3, ang2, ang1, 'ZYX') %matriz de Transformação a partir dos angulos de euler
RT = t2r(Tot) %matriz de rotação em função de matriz de transformação

% ii)
angi1 = pi/6
angi2 = pi/2
angi3 = -pi/3.2727

% --------------- Angulos de euler X-Y-Z em matriz rotacional ---------%

Matrixi3 = [cos(angi1) -sin(angi1) 0; sin(angi1) cos(angi1) 0; 0 0 1] 
Matrixi2 = [cos(angi2) 0 sin(angi2); 0 1 0; -sin(angi2) 0 cos(angi2)]
Matrixi1 = [1 0 0; 0 cos(angi3) -sin(angi3); 0 sin(angi3) cos(angi3)]

Resulti = Matrixi3*Matrixi2*Matrixi1


Toti = rpy2tr(angi3, angi2, angi1, 'ZYX') %matriz de Transformação a partir dos angulos de euler
RTi = t2r(Toti) %matriz de rotação em função de matriz de transformação

% ----------------------------------------------------------------------%

% ----------------------------- Outra maneira de fazer -----------------%

%MATRIX = [cos(ang1)*cos(ang2) cos(ang1)*sin(ang2)*sin(ang3)-sin(ang1)*cos(ang3) cos(ang1)*sin(ang2)*cos(ang3)+sin(ang1)*sin(ang3); 
%         sin(ang1)*cos(ang2) sin(ang1)*sin(ang2)*sin(ang3)+cos(ang1)*cos(ang3) sin(ang1)*sin(ang2)*cos(ang3)+cos(ang1)*sin(ang3);
%         -sin(ang2) cos(ang2)*sin(ang3) cos(ang2)*cos(ang3)]

% ---------------- Exercicícios MATLAB 2A b) -----------------%

% Escreva um programa para o MATLAB para calcular os ângulos a–b–g de Euler quando o usuário entra uma matriz rotacional B
% AR (o problema inverso). Calcule ambas as soluções possíveis. 
% Demonstre a solução inversa para os dois casos do item (a). Faça uma verificação circular para 
% conferir os resultados (isto é, entre os ângulos de Euler no código do item (a); tome a matriz rotacional resultante B
% AR e use-a como entrada para o código do item (b); você obterá dois conjuntos 
% de respostas – uma deve ser a entrada original do usuário e a outra pode ser verificada usando-se 
% mais uma vez o código no item a)

% determinando os ângulos de euler X-Y-Z da matriz do item i) da letra a)

phi = atan2(Result(3,2), Result(3,3))
psi = atan2(-Result(3,1), sqrt(Result(3,2)^2 + Result(3,3)^2))
teta = atan2(Result(2,1), Result(1,1))

% Agrupando os Ângulos em uma matriz

Ang = [teta, psi, phi]

% Usando a ferramenta de conversão pela Robotics Toolbox
eulZYX = rotm2eul(Result)

% Determinando os ângulos de euler X-Y-Z da matriz do item ii) da letra a)

phi2 = atan2(Resulti(3,2), Resulti(3,3))
psi2 = atan2(-Resulti(3,1), sqrt(Resulti(3,2)^2 + Resulti(3,3)^2))
teta2 = atan2(Resulti(2,1), Resulti(1,1))

% Matriz com os ângulos de euler 

Ang2 = [teta2, psi2, phi2]

eul2ZYX = rotm2eul(Resulti)

% Matriz de entrada do usuário

Matrix = [0.1344 1.4 0; 9.124 1.13 0; 0 -13 0.124]

A = atan2(Matrix(3,2), Matrix(3,3))
B = atan2(-Matrix(3,1), sqrt(Matrix(3,2)^2 + Matrix(3,3)^2))
C = atan2(Matrix(2,1), Matrix(1,1))

angulos = [A B C]


%  Reinserindo os angulos de Euler para obter as matrizes de a)

% ---- ang1 = teta ---- ang2 = psi ------ ang3 = phi

m3 = [cos(teta) -sin(teta) 0; sin(teta) cos(teta) 0; 0 0 1]
m2 = [cos(psi) 0 sin(psi); 0 1 0; -sin(psi) 0 cos(psi)]
m1 = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]

mr = m3*m2*m1
