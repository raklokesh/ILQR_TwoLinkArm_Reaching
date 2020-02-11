function []=PlotTrajectories(X_all,th1,th2,N,t)
l1=0.3;l2=0.33;
for i=[1]
        X=squeeze(X_all(i,:,:))+repmat([th1;th2;0;0],1,length(t));;
        if (i==N)
            C=[0 0 1];
        elseif (i==1)
            C=[1 0 0];
             elseif (i==2)
            C=[0.4 0.4 0.4];
        else
            C=[0.8 0.8 0.8];
        end
        hold on;
        plot(l1*cos(th1)+l2*cos(th2+th1)...
            ,l2*sin(th1)+l2*sin(th1+th2),'o','MarkerFaceColor','k','MarkerSize',8);
        plot(l1*cos(X(1,:))+l2*cos(X(2,:)+X(1,:)),...
            l2*sin(X(1,:))+l2*sin(X(2,:)+X(1,:))...
            ,'color',C,'LineWidth',1);
        axis square;
        axis([-0.45 0.25 0.05 .75]);
end
end