L1 = 2
L2 = 5

    % Os parâmetros devem ser colocados nessa ordem: theta, d, a, alpha
DH = [0 0 0 0; 0 0 L1 0; 0 0 L2 0]

% Criando os links do robô

L(1) = Link(DH(1,1:4), 'modified')
L(2) = Link(DH(2,1:4), 'modified')
L(3) = Link(DH(3,1:4), 'modified')

% Criando o objeto do robô

My3R = SerialLink(L, 'name', 'Primeiro robô planar')


% Angulo inicial das juntas
qi = [pi/3, pi/4, pi/5]

% Angulo posterior das juntas
qd = [2*pi/3, 2*pi/4, 2*pi/5]

% Pose cartesiana inicial usando cinemática direta
Ti = fkine (My3R, qi)

% Pose cartesiana posterior usando cinemática direta
Td = fkine (My3R, qd)

% Mostrar as posições em 2D
plot(My3R, qi), view(2)
plot(My3R, qd), view(2)



clc; clear



L1 = 2
L2 = 5

    % Os parâmetros devem ser colocados nessa ordem: theta, d, a, alpha
DH = [0 0 0 0; 0 0 0 pi/2; 0 L2 0 0]

% Criando os links do robô de outra maneira

L(1) = Link('revolute', 'd', DH(1,2),'a', DH(1,3), 'alpha', DH(1,4), 'modified')
L(2) = Link('prismatic', 'theta', DH(2,2),'a', DH(2,3), 'alpha', DH(2,4), 'modified')
L(3) = Link('revolute', 'd', DH(3,2),'a', DH(3,3), 'alpha', DH(3,4), 'modified')

% Criando o objeto do robô

MyRPR = SerialLink(L, 'name', 'Primeiro robô planar')


% Angulo inicial das juntas
qi = [pi/4 7  pi/2]

% Angulo posterior das juntas
qd = [pi/8 2  pi/4]

% Pose cartesiana inicial usando cinemática direta
Ti = fkine (MyRPR, qi)

% Pose cartesiana posterior usando cinemática direta
Td = fkine (MyRPR, qd)

% Mostrar as posições em 2D
MyRPR.plot(qi, 'workspace', [-10, 10, -10, 10, -10, 10])
MyRPR.plot(qd, 'workspace', [-10, 10, -10, 10, -10, 10])


