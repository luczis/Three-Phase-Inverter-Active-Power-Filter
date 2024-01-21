close all;

%Variaveis
w=377;
Vtri=5;
Vcc=60;
R1=100;
R2=2;
C1=100e-9;
L1=256e-3;
Kh=10;
Vref=1;

te=5*10^-3;
lambda=0.05;
theta=-5;

absCsGs=(Kh*lambda)^(-1)*sqrt((cosd(theta)-lambda)^2+sind(theta)^2);
absCsGsDB=20*log10(absCsGs);
angleCsGs=(180/pi)*atan(sind(theta)/(cosd(theta)-lambda));

sigma=log(lambda^-1)/te;

sgain=(1-lambda)/(Kh*lambda);


Ky=Vcc/Vtri;
A2=R2*C1*L1;
A1=R1*R2*C1+L1;
A0=R2+R1;
%(A0-A2*w^2)
B=asin((A1*w)/(sqrt((A1*w)^2+(A0-A2*w^2)^2)));
C=asin(sind(theta)/(sqrt((cosd(theta)-lambda)^2+(sind(theta))^2)));




alpha=(1+sin(C+B))/(1-sin(C+B));

Kp=sqrt(alpha*((A1*w)^2+(A0-A2*w^2)^2)*((cosd(theta)-lambda)^2+(sind(theta))^2))/(Ky*Kh*lambda);

%Kp=sqrt(alpha)*sqrt((A1*w)^2+(A0-A2*w^2)^2)/(Ky*Kh);

p=w*sqrt(alpha);
z=w/sqrt(alpha);

Cx2=500e-6;
Rx2=1/(Cx2*p);
Cx1=(Kp*alpha-1)*Cx2;
Rx1=Cx2*w*(Kp*alpha-alpha)/sqrt(alpha);




Tmax=0.05;
t = 0:0.00001:Tmax;




%Kp=3;
%z=-100;
%p=-1000;
%T=tf([10 0],[1 0 (w*1.01)^2]);
%LL=Kp*T;

LL=Kp*tf([1 z],[1 p]);

%LL=tf([1 50],[1 0])*tf([1 100],[1 0])*tf([1 150],[1 0])*tf([1 200],[1 0])*tf([1 250],[1 0])*tf([1 300],[1 0])*tf([1 350],[1 0]);
%LL=tf([1 150],[1 0])*tf([1 200],[1 0])*tf([1 250],[1 0]);

% %Resolver o sistema
Gs= tf([Vcc/Vtri],[R2*C1*L1 R1*R2*C1+L1 R2+R1]);
% %u=Vref*sin(w*t);
% u=Vref*sin(w*t);
% result = lsim(LL*Gs/(1+Kh*LL*Gs),u,t);

%result = step(LL*Gs/(1+Kh*LL*Gs),t);

%Us=tf([Vref*w],[1 0 w^2]);


u=Vref*sin(w*t);

result=lsim(LL*Gs/(1+Kh*LL*Gs),u,t);
figure; hold on;
plot(t,Kh*result,'k');
plot(t,u,'--k');
hold off;

title('Compensador de avanço')
xlabel('Tempo(s)');
ylabel('Amplitude');
legend('K_{H}i_{o}(t)','v_{ref}(t)');


figure; hold on; grid off;
ax=findobj(gcf,'type','axes');
rootlocus_ax=ax(1);
rlocus(Gs*LL/(1+Kh*Kp*Gs*LL),'k-');
plot([-sigma -sigma],[10e8,-10e8],'--k');
hold off;
% axis([-800 100 -1000 1000]);
title('Compensador de avanço');
legend(rootlocus_ax,'C(s)G_{C}(s)','t_{e min}');

% figure; hold on; grid off;
% bode(Gs*LL,'k');
% hold off;
% ax=findobj(gcf,'type','axes');
% phase_ax=ax(1);
% mag_ax=ax(2);
% ax_xlim=phase_ax.XLim;
% phase_ylim=phase_ax.YLim;
% mag_ylim=mag_ax.YLim;
% hold(phase_ax,'on');
% plot(phase_ax,[w w],[phase_ylim(1) phase_ylim(2)],'-.k')
% plot(phase_ax,[ax_xlim(1), ax_xlim(2)],[-angleCsGs -angleCsGs],'--k');
% plot(phase_ax,[ax_xlim(1), ax_xlim(2)],[angleCsGs angleCsGs],'--k');
% hold(mag_ax,'on');
% plot(mag_ax,[w w],[mag_ylim(1) mag_ylim(2)],'-.k');
% plot(mag_ax,[ax_xlim(1), ax_xlim(2)],[absCsGsDB absCsGsDB],'--k');
% title('Compensador de avanço');
% 
% legend(mag_ax,'C(j\omega)G_{C}(j\omega)','\omega_{ref}','ganho_{min}');
% legend(phase_ax,'C(j\omega)G_{C}(j\omega)','\omega_{ref}','\theta_{max}','\theta_{min}')


% Cc1=100e-6;
% Rc1=z/Cc1;
% Rc2=(p-z)/Cc1;


% Cc1=100e-6;
% Rc2=z/Cc1;
% Rc1=p*Rc1/(p-z);



Ksensor=0.625/5;


R19=20e3;
R20=R19/(Kh/Ksensor);
R21=2*R19;

Cc1=100e-9;
Cc2=100e-9;
Rc1=1/Cc1/z;
Rc2=1/Cc2/p;
