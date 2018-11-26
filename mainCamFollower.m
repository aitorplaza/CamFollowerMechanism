clear all
close all

L=10;

L1 = L; 
L2 = L; 
L3 = L;

beta4 = pi/2;
beta1 = (2*pi-beta4)/(1+(2*L2+4*L3)/(3*L1)+sqrt(4*(L1+L2+L3)/L1));
beta2 = 2*L2/(3*L1)*beta1;
beta3 = 2*L3/L2*beta2;
beta5 = sqrt(4*(L1+L2+L3)/L1)*beta1;

phiA = 0;
phiB = beta1;
phiC = phiB + beta2;
phiD = phiC + beta3;
phiE = phiD + beta4;
phiF = phiE + beta5;

%###############################
%# Follower postions           #
%###############################

div=100;
fis1 = phiA:beta1/div:phiB;
fis2 = phiB:beta2/div:phiC;
fis3 = phiC:beta3/div:phiD;
fis4 = phiD:beta4/div:phiE;
fis5 = phiE:beta5/div:phiF;

% Bézier ordinates :
c1=L*[0 0 1/2 1]';
n1=numel(c1)-1;

c2 = [L1; L1+L2];
n2=numel(c2)-1;

c3 = [L1+L2; L1+L2+L3/2 ;L1+L2+L3 ;L1+L2+L3 ;L1+L2+L3];
n3=numel(c3)-1;

c4 = [L1+L2+L3; L1+L2+L3];
n4=numel(c4)-1;

c5 = [L1+L2+L3 ;L1+L2+L3 ;L1+L2+L3; 0; 0];
n5=numel(c5)-1;

% Bézier Control Points:
xPC1 = phiA:beta1/n1:phiB;
xPC2 = phiB:beta2/n2:phiC;
xPC3 = phiC:beta3/n3:phiD;
xPC4 = phiD:beta4/n4:phiE;
xPC5 = phiE:beta5/n5:phiF;
   
xPC=[xPC1(:);xPC2(:);xPC3(:);xPC4(:);xPC5(:)];
yPC=[c1(:);c2(:);c3(:);c4(:);c5(:)];

% Positions:
for j=1:div+1
   u=(j-1)/div;
   ys1(j)=Bezier(c1,n1,u);
   ys2(j)=Bezier(c2,n2,u);
   ys3(j)=Bezier(c3,n3,u);
   ys4(j)=Bezier(c4,n4,u);   
   ys5(j)=Bezier(c5,n5,u);   
   
end

figure(1)
hold on
grid on
title('Follower Position [$mm$] - Cam Angle [$rad$]','interpreter','latex','Fontsize', 12) 
yMin=0;
yMax=3*L;
axis([0 2*pi yMin yMax])
plot(xPC,yPC,'b*')
plot(xPC,yPC,'b')
plot(fis1,ys1,'r')
plot(fis2,ys2,'r')
plot(fis3,ys3,'r')
plot(fis4,ys4,'r')
plot(fis5,ys5,'r')
hold off

%###############
%# Velocities  #
%###############

% Bézier ordinates :
cp1 = n1/beta1*L*[0 1/2 1/2]';
cp2 = n2/beta2*L2;
cp3 = n3/beta3*[L3/2; L3/2; 0; 0];
cp4 = 0;
cp5 = n5/beta5*[0; 0; -(L1+L2+L3); 0];

% Update polinomial degree:
n1=numel(cp1)-1;
n2=numel(cp2)-1;
n3=numel(cp3)-1;
n4=numel(cp4)-1;
n5=numel(cp5)-1;

% Bézier Control Points:
dxPC1 = phiA:beta1/n1:phiB;
dxPC2 = phiB:beta2/n2:phiC;
dxPC3 = phiC:beta3/n3:phiD;
dxPC4 = phiD:beta4/n4:phiE;
dxPC5 = phiE:beta5/n5:phiF;

dxPC=[dxPC1(:);dxPC2(:);dxPC3(:);dxPC4(:);dxPC5(:)];
dyPC=[cp1(:);cp2(:);cp3(:);cp4(:);cp5(:)];


% Velocities:
for j=1:div+1
   u=(j-1)/div;
   dys1(j)=Bezier(cp1,n1,u);
   dys2(j)=Bezier(cp2,n2,u);
   dys3(j)=Bezier(cp3,n3,u);
   dys4(j)=Bezier(cp4,n4,u);   
   dys5(j)=Bezier(cp5,n5,u);   
end

figure(2)
hold on
grid on
title('Follower Velocity [$mm/s$] - Cam Angle [$rad$]','interpreter','latex','Fontsize', 12)
yMin=min(dyPC);
yMax=max(dyPC);
axis([0 2*pi yMin yMax])
plot(dxPC,dyPC,'b*')
plot(dxPC,dyPC,'b')
plot(fis1,dys1,'r')
plot(fis2,dys2,'r')
plot(fis3,dys3,'r')
plot(fis4,dys4,'r')
plot(fis5,dys5,'r')
hold off


%#################
%# Accelerations #
%#################

% Bézier ordinates :
cpp1 = (n1+1)*n1/beta1^2*L*[1/2 0]';
cpp2 = 0;
cpp3 = (n3+1)*n3/beta3^2*[0; -L3/2; 0];
cpp4 = 0;
cpp5 = (n5+1)*n5/beta5^2*[0; -(L1+L2+L3); L1+L2+L3];

% Update polinomial degree:
n1=numel(cpp1)-1;
n2=numel(cpp2)-1;
n3=numel(cpp3)-1;
n4=numel(cpp4)-1
n5=numel(cpp5)-1;

% Bézier Control Points:
ddxPC1 = phiA:beta1/n1:phiB;
ddxPC2 = phiB:beta2/n2:phiC;
ddxPC3 = phiC:beta3/n3:phiD;
ddxPC4 = phiD:beta4/n4:phiE;
ddxPC5 = phiE:beta5/n5:phiF;

ddxPC=[ddxPC1(:);ddxPC2(:);ddxPC3(:);ddxPC4(:);ddxPC5(:)];
ddyPC=[cpp1(:);cpp2(:);cpp3(:);cpp4(:);cpp5(:)];


% Acclerations:
for j=1:div+1
   u=(j-1)/div;
   ddys1(j)=Bezier(cpp1,n1,u);
   ddys2(j)=Bezier(cpp2,n2,u);
   ddys3(j)=Bezier(cpp3,n3,u);
   ddys4(j)=Bezier(cpp4,n4,u);   
   ddys5(j)=Bezier(cpp5,n5,u);   
end


figure(3)
hold on
grid on
title('Follower Acceleration [$mm/s^2$] - Cam Angle [$rad$]','interpreter','latex','Fontsize', 12)
yMin=min(ddyPC);
yMax=max(ddyPC);
axis([0 2*pi yMin yMax])
plot(ddxPC,ddyPC,'b*')
plot(ddxPC,ddyPC,'b')
plot(fis1,ddys1,'r')
plot(fis2,ddys2,'r')
plot(fis3,ddys3,'r')
plot(fis4,ddys4,'r')
plot(fis5,ddys5,'r')
hold off

%%
%#####################################
% Cam profile for eccentric follower
%#####################################

% Eccentricity
e=20;
% Base circle radious
rb=20;

% Join all the Bézier curves in a single vector
y    = [ys1,ys2(2:end),ys3(2:end),ys4(2:end),ys5(2:end)];
dy   = [dys1,dys2(2:end),dys3(2:end),dys4(2:end),dys5(2:end)];
ddy  = [ddys1,ddys2(2:end),ddys3(2:end),ddys4(2:end),ddys5(2:end)];
ySeg = rb+[ys1,ys2(2:end),ys3(2:end),ys4(2:end),ys5(2:end)];
Phi  = [fis1,fis2(2:end),fis3(2:end),fis4(2:end),fis5(2:end)];


%Plot cam-follower mechanism:
h=figure(4)
x_leva =  e*cos(Phi) + ySeg.*sin(Phi);
y_leva = -e*sin(Phi) + ySeg.*cos(Phi);
x_seg = [e,e];
y_seg = [ySeg(1),ySeg(1)+60]; 
y_segaux = 0;

subplot(6,3,1:9)
hold on;axis equal;grid on; 
axis([-100 100 -100 100]);
title('Cam-Follower Mechanism','interpreter','latex','Fontsize', 20)
leva = plot(x_leva,y_leva,'b','LineWidth',2)
seg  = plot(x_seg,y_seg,'g', 'LineWidth',4)
plot(0,0,'.k', 'MarkerSize',20)
set(gca,'xtick',[])
set(gca,'ytick',[])
set(gca,'xcolor',[1 1 1])
set(gca,'ycolor',[1 1 1])
hold off

subplot(6,3,10:12)
hold on;grid on; 
axis([0 2*pi -5 35]);
title('Follower Position [$mm$] - Cam Angle [$rad$]','interpreter','latex','Fontsize', 12, 'Units', 'normalized','Position', [0.5, -0.3, 0])
plot(Phi,y,'r')
pnt = plot(0,y(1),'.k', 'MarkerSize',20)
hold off

subplot(6,3,13:15)
hold on;grid on; 
axis([0 2*pi -30 30]);
title('Follower Velocity [$mm/s$] - Cam Angle [$rad$]','interpreter','latex','Fontsize', 12,'Units', 'normalized','Position', [0.5, -0.3, 0])
plot(Phi,dy,'r')
dpnt = plot(0,dy(1),'.k', 'MarkerSize',20)
hold off

subplot(6,3,16:18)
hold on;grid on; 
axis([0 2*pi -50 80]);
title('Follower Acceleration [$mm/s^2$] - Cam Angle [$rad$]','interpreter','latex','Fontsize', 12,'Units', 'normalized','Position', [0.5, -0.3, 0])
plot(Phi,ddy,'r')
ddpnt = plot(0,ddy(1),'.k', 'MarkerSize',20)
hold off

% Animate plot:

disp('Press a key to continue!')  
pause;
omega = 1;%rad/s
dt = 0.04;
filename = 'CamFollower.gif';

for t=0:dt:(2*pi)/omega  
     
    dphi = omega*t;
    
    Phi_ = Phi + dphi;
   
    x_leva =  e*cos(Phi_) + ySeg.*sin(Phi_);
    y_leva = -e*sin(Phi_) + ySeg.*cos(Phi_);
    set(leva,'Xdata', x_leva, 'YData',y_leva )
    
    y_segaux = interp1(Phi,ySeg,mod(dphi,2*pi) );
    y_seg = [y_segaux,y_segaux+60];
    set(seg,'YData',y_seg)
    
    y_aux   = interp1(Phi,y,mod(dphi,2*pi) );
    dy_aux = interp1(Phi,dy,mod(dphi,2*pi) );
    ddy_aux = interp1(Phi,ddy,mod(dphi,2*pi) );    
    set(pnt,'Xdata',mod(dphi,2*pi) ,'YData',y_aux);
    set(dpnt,'Xdata',mod(dphi,2*pi) ,'YData',dy_aux);   
    set(ddpnt,'Xdata',mod(dphi,2*pi) ,'YData',ddy_aux);   


    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);

    if t == 0;
        imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',dt);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dt);
    end  
    
    
end 



