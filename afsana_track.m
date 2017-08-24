clc
clear all
close all



t = 0:0.007:1;
% figure(1)
% h = plot(0,0,'rs');
% 
% axis([-50 100 -100 100])


%box on
x(1)=0;y(1)=0;omega(1)=0;u(1)=5*rand(1,1);
a=0;b=1;r = a + (b-a).*rand(1,1);
%creat_track();
X=[ -50 20 40 20 0 -40 -50];
Y=[-50 -60 40 40  60 20 -50];
figure(1)
fill(X,Y,'w','LineWidth',8,'edgecolor','y')%'LineWidth',4,'r')
%axis([-100 100 -100 100])
hold on

h = plot(0,0,'rs');
h = gcf; %current figure handle  
% axesObjs = get(h, 'Children');  %axes handles
% 
% dataObjs = get(axesObjs, 'Children'); %handles to low-level graphics objects in axes   
% xdata = get(dataObjs, 'XData');  %data from low-level grahics objects
% 
% ydata = get(dataObjs, 'YData');

%x1=getframe()

for n = 2:numel(t)
    
% set(h,{'Xdata','Ydata'},{x(n-1) y(n-1) });
  plot(x(n-1),y(n-1),'rs')
%   hold on
      if u(n-1)>5 
      u(n)=u(n-1)- r;
      else u(n)=u(n-1)+ r;
      end
      
  omega(n)=omega(n-1)+rand(1,1);
  if omega(n-1)>pi & omega(n-1)<-pi
      omega(n)=0;
  end
  %%sensor()
  x(n)=x(n-1)+u(n)*cos(omega(n));
  y(n)=y(n-1)+u(n)*sin(omega(n));
  title(t(n));

  drawnow;
end
% idx=3
% x1=getframe()
% x2=reshape(x1.cdata,[],1);
% x3=im2double(x1);
%rect=[20 50 60 100];
axis tight
rect = get(gcf,'Position');
rect(1:2) = [0 0];
F = getframe(gcf,rect);

[x1, Map] = frame2im(F);
x2=reshape(x1,[],1);
figure(2)
image(x1)
