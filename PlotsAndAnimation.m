%% Plotting angular positions and velocities
for target_num=1%:4
    switch target_num
        case 1
            th1=35*pi/180;th2=75*pi/180;
            X_nom=squeeze(Xn1(1,:,:))+repmat([th1;th2;0;0],1,length(t));
            X_new=squeeze(Xn1(end,:,:))+repmat([th1;th2;0;0],1,length(t));
        case 2
            th1=70.75*pi/180;th2=55.27*pi/180;
            X_nom=squeeze(Xn2(1,:,:))+repmat([th1;th2;0;0],1,length(t));
            X_new=squeeze(Xn2(end,:,:))+repmat([th1;th2;0;0],1,length(t));
        case 3
            th1=93.23*pi/180;th2=57.65*pi/180;
            X_nom=squeeze(Xn3(1,:,:))+repmat([th1;th2;0;0],1,length(t));
            X_new=squeeze(Xn3(end,:,:))+repmat([th1;th2;0;0],1,length(t));
        case 4
            th1=103.07*pi/180;th2=84.25*pi/180;
            X_nom=squeeze(Xn4(1,:,:))+repmat([th1;th2;0;0],1,length(t));
            X_new=squeeze(Xn4(end,:,:))+repmat([th1;th2;0;0],1,length(t));
    end
    figure(target_num)
    subplot(2,1,1);
    hold on;
    plot((1:length(t))*dt,X_nom([1,2],1:end),'r','LineWidth',1);
    plot((1:length(t))*dt,X_new([1,2],1:end),'b','LineWidth',1);
    line([0 t(end)],[th1 th1],'color','k','LineWidth',1);
    line([0 t(end)],[th2 th2],'color','k','LineWidth',1);
    subplot(2,1,2);
    hold on;
    plot((1:length(t))*dt,X_nom([3,4],1:end),'r','LineWidth',1);
    plot((1:length(t))*dt,X_new([3,4],1:end),'b','LineWidth',1);
    line([0 t(end)],[0 0],'color','k','LineWidth',1);
    print('States','-dpng','-r300');
end
%% Plotting all the trajectories
for target_num=1:4
    figure(2)
    plot(l1*cos(th11)+l2*cos(th22+th11)...
        ,l2*sin(th11)+l2*sin(th11+th22),'o','MarkerFaceColor','k','MarkerSize',8);
    switch target_num
        case 1
            th1=35*pi/180;th2=75*pi/180;
            PlotTrajectories(Xn1,th1,th2,length(J1),t);
        case 2
            th1=70.75*pi/180;th2=55.27*pi/180;
            PlotTrajectories(Xn2,th1,th2,length(J2),t);
        case 3
            th1=93.23*pi/180;th2=57.65*pi/180;
            PlotTrajectories(Xn3,th1,th2,length(J3),t);
        case 4
            th1=103.07*pi/180;th2=84.25*pi/180;
            PlotTrajectories(Xn4,th1,th2,length(J4),t);
    end
end
print('NominalTrajectory','-dpng','-r300');
%% Plotting cost functions
for target_num=1:4
    figure (20)
    subplot(2,2,1);
    hold on;
    plot(J1,'ro','MarkerFaceColor','b');
    plot(J1,'LineWidth',1,'color','r');
    subplot(2,2,2);
    hold on;
    plot(J2,'ro','MarkerFaceColor','b');
    plot(J2,'LineWidth',1,'color','r');
    subplot(2,2,3);
    hold on;
    plot(J3,'ro','MarkerFaceColor','b');
    plot(J3,'LineWidth',1,'color','r');
    subplot(2,2,4);
    hold on;
    plot(J4,'ro','MarkerFaceColor','b');
    plot(J4,'LineWidth',1,'color','r');
end
print('LMACostData','-dpng','-r300');
%% Comparing cost for LMA and non LMA
for j=1:4
    figure (20)
    subplot(2,2,1);
    hold on;
    plot(J11,'LineWidth',1,'color','b');
    plot(J1,'LineWidth',1,'color','r');
    subplot(2,2,2);
    hold on;
    plot(J21,'LineWidth',1,'color','b');
    plot(J2,'LineWidth',1,'color','r');
    subplot(2,2,3);
    hold on;
    plot(J31,'LineWidth',1,'color','b');
    plot(J3,'LineWidth',1,'color','r');
    subplot(2,2,4);
    hold on;
    plot(J41,'LineWidth',1,'color','b');
    plot(J4,'LineWidth',1,'color','r');
end
print('CompareCostData','-dpng','-r300');
%% Generating animation of the two link arm
M(4*length(t)) = struct('cdata',[],'colormap',[]);
figure (10)
for j=1:4
    switch j
        case 1
            th1=35*pi/180;th2=75*pi/180;
            Xt=squeeze(Xn1(1,:,:))+repmat([th1;th2;0;0],1,length(t));
        case 2
            th1=70.75*pi/180;th2=55.27*pi/180;
            Xt=squeeze(Xn2(1,:,:))+repmat([th1;th2;0;0],1,length(t));
        case 3
            th1=93.23*pi/180;th2=57.65*pi/180;
            Xt=squeeze(Xn3(1,:,:))+repmat([th1;th2;0;0],1,length(t));
        case 4
            th1=103.07*pi/180;th2=84.25*pi/180;
            Xt=squeeze(Xn4(1,:,:))+repmat([th1;th2;0;0],1,length(t));
    end
    for i=1:length(t)
        i*0.01;
        hold on;
        plot(l1*cos(th11)+l2*cos(th22+th11)...
            ,l2*sin(th11)+l2*sin(th11+th22),'o','MarkerFaceColor','k','MarkerSize',8);
        plot(l1*cos(th1)+l2*cos(th2+th1)...
            ,l2*sin(th1)+l2*sin(th1+th2),'o','MarkerFaceColor','k','MarkerSize',8);
        line([0 l1*cos(Xt(1,i))],[0 l1*sin(Xt(1,i))],'color',[255,224,180]./255,'LineWidth',8);
        line([l1*cos(Xt(1,i)),l1*cos(Xt(1,i))+l2*cos(Xt(2,i)+Xt(1,i))]...
            ,[l1*sin(Xt(1,i)),l2*sin(Xt(1,i))+l2*sin(Xt(2,i)+Xt(1,i))],'color',[255,224,180]./255,'LineWidth',8);
        plot(0,0,'o','MarkerFaceColor',[255,224,210]./255,'MarkerSize',8);
        plot(l1*cos(Xt(1,i)),l1*sin(Xt(1,i)),'o','MarkerFaceColor',[255,224,210]./255,'MarkerSize',8);
        plot(l1*cos(Xt(1,1:i))+l2*cos(Xt(2,1:i)+Xt(1,1:i)),l2*sin(Xt(1,1:i))+l2*sin(Xt(2,1:i)+Xt(1,1:i))...
            ,'color','r','LineWidth',1);
        axis square;
        axis([-0.45 0.4 -0.1 0.75]);
        M((j-1)*length(t)+i)=getframe(gca);
        pause(0.0005);
        if (i<length(t))
            clf;
        end
    end
end
Video = VideoWriter('AnimateLinksInitial.mp4');
Video.FrameRate = 50;
open(Video)
for i = 1:length(M)
    img = M(i).cdata;
    writeVideo(Video,img);
end
close(Video)