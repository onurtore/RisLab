close all,clear all,clc;
load ('cigil.txt');
disp(sprintf('table loaded') );

angle=cigil(:,3);
time=cigil(:,2)/1000;
fcX=cigil(:,4);
fcZ=cigil(:,5);
fhX=cigil(:,6);
fhZ=cigil(:,7);
bpX=cigil(:,8);bpZ=cigil(:,9);
%bvX=cigil(:,10);bvZ=cigil(:,11);
%baX=cigil(:,12);baZ=cigil(:,13);
cpX=cigil(:,14);cpZ=cigil(:,15);
%cvX=cigil(:,16);cvZ=cigil(:,17);
hpX=cigil(:,18);hpZ=cigil(:,19);
%hvX=cigil(:,20);hvZ=cigil(:,21);


for i=1:length(time)-1
    bvX(i)=(bpX(i+1)-bpX(i))/(time(2)-time(1));
    bvZ(i)=(bpZ(i+1)-bpZ(i))/(time(2)-time(1));
     hvX(i)=(hpX(i+1)-hpX(i))/(time(2)-time(1));
    hvZ(i)=(hpZ(i+1)-hpZ(i))/(time(2)-time(1));
     cvX(i)=(cpX(i+1)-cpX(i))/(time(2)-time(1));
    cvZ(i)=(cpZ(i+1)-cpZ(i))/(time(2)-time(1));
   
end
for i=1:length(time)-2
  baX(i)=(bvX(i+1)-bvX(i))/(time(2)-time(1));
  baZ(i)=(bvZ(i+1)-bvZ(i))/(time(2)-time(1));
end
for i=1:length(time)-3
  jerkX(i)=(baX(i+1)-baX(i))/(time(2)-time(1));
 jerkZ(i)=(baZ(i+1)-baZ(i))/(time(2)-time(1));
end
    
% figure(1)
% subplot(1,2,1)
% plot(time(2001:end),fcX(2001:end),'r',time(2001:end),fhX(2001:end),'g')
% grid on;
% legend('CIP','HIP')
% title('force_x')
% subplot(1,2,2)
% plot(time(2001:end),fcZ(2001:end),'r',time(2001:end),fhZ(2001:end),'g')
% legend('CIP','HIP')
% grid on;
% title('force_z')
% 
% figure(2)
% subplot(1,2,1)
% plot(time(2001:end),cpX(2001:end),'r',time(2001:end),hpX(2001:end),'g',time(2001:end),bpX(2001:end),'m')
% grid on;
% legend('CIP','HIP','ball');
% title('position_x')
% subplot(1,2,2)
% plot(time(2001:end),cpZ(2001:end),'r',time(2001:end),hpZ(2001:end),'g',time(2001:end),bpZ(2001:end),'m')
% legend('CIP','HIP','ball')
% grid on;
% title('position_z')
% 
% 
% figure(3)
% subplot(1,2,1)
% plot(time(2001:end-1),cvX(2001:end),'r')
% %hold on;
% %plot(time(2001:end-1),hvX(2001:end),'g')%,time(2001:end-1),bvX(2001:end),'m')
% grid on;
% legend('CIP','HIP','ball');
% title('velocity_x')
% subplot(1,2,2)
% plot(time(2001:end-1),cvZ(2001:end),'r')
% %hold on
% %plot(time(2001:end-1),hvZ(2001:end),'g')%,time(2001:end-1)),bvZ(2001:end),'m')
% legend('CIP','HIP')%,'ball')
% grid on;
% title('velocity_z')
% 
% figure(4)
% subplot(2,2,1)
% plot(time(2001:end),bpX(2001:end))
% title('ball position x')
% grid on;
% subplot(2,2,2)
% plot(time(2001:end-1),bvX(2001:end))
% title('ball_velocity x')
% subplot(2,2,3)
% plot(time(2001:end-2),baX(2001:end))
% title('ball acceleration x')
% subplot(2,2,4)
% plot(time(2001:end-3),jerkX(2001:end))
% title('jerk x')
% 
% figure(5)
% subplot(2,2,1)
% plot(time(2001:end),bpZ(2001:end))
% grid on;
% title('ball position z')
% grid on;
% subplot(2,2,2)
% plot(time(2001:end-1),bvZ(2001:end))
% grid on;
% title('ball_velocity z')
% subplot(2,2,3)
% plot(time(2001:end-2),baZ(2001:end))
% grid on;
% title('ball acceleration z')
% subplot(2,2,4)
% plot(time(2001:end-3),jerkZ(2001:end))
% grid on;
% title('jerk z')
% 
% 
% figure(6)
% plot(angle(2001:end))
% grid on;
% title('angle vs time')


figure(7)
  plot(bpX(1),bpZ(1),'r');
axis([-10,65,-30,50]);
for i=2:length(time)
    
    plot(bpX(i),-bpZ(i),'r','linewidth',2);
    axis([-15,65,-30,50]);
   % pause(.1)
    hold on;
    grid on;
end









