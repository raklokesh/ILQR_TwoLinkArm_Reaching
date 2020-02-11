function [X_All,U_All,J,lambda]=LMA_ILQR(X,U,A,B,C,Q,Qf,R,t,N)
lambdaFactor = 10;lambda=zeros(1,N+1);lambda(1)=0.1;
CostSameCount=0;
J=zeros(1,N+1);
% Calculate entry cost from nominal control vector
for k=1:length(t)-1
    J(1)=J(1)+1/2*X(:,k)'*Q*X(:,k)+1/2*U(:,k)'*R*U(:,k);
end
J(1)=J(1)+1/2*X(:,end)'*Qf*X(:,end);
for iter_num=1:N
    iter_num
    X_All(iter_num,:,:)=X;
    U_All(iter_num,:,:)=U;
    %plotting trajectories for each pass
    %     figure (iter_num)
    % %     subplot(2,N/2,n);
    %     l1=0.3;l2=0.33;
    %     hold on;
    %     plot(l1*cos(th11)+l2*cos(th22+th11)...
    %         ,l2*sin(th11)+l2*sin(th11+th22),'o','MarkerFaceColor','g','MarkerSize',8);
    %     plot(l1*cos(th1)+l2*cos(th2+th1)...
    %         ,l2*sin(th1)+l2*sin(th1+th2),'o','MarkerFaceColor','r','MarkerSize',8);
    %     p1=plot(l1*cos(X(1,:)+th1)+l2*cos(X(2,:)+X(1,:)+th2+th1),...
    %         l2*sin(X(1,:)+th1)+l2*sin(X(2,:)+X(1,:)+th1+th2)...
    %         ,'color','r','LineWidth',1);
    %     axis square;
    %     axis([-0.45 0.4 -0.1 0.75]);
    % Backward pass to find K,Kv,Ku,Sk,Vk.
    % Adopted DDP method from Tassa et al., 2014
    V(:,length(t))=Qf*(X(:,end)); % Value function
    S(:,:,length(t))=Qf; 
    Kk(:,length(t))=zeros(1,2);
    KK(:,:,length(t))=zeros(2,4);
    for i=1:length(t)-1
        x1=X(1,length(t)-i);x2=X(2,length(t)-i);x3=X(3,length(t)-i);x4=X(4,length(t)-i);
        A1=double(subs(A));
        B1=double(subs(B));
        C1=[1 0 0 0;0 1 0 0];
        ContSys=ss(A1,B1,C1,0);
        DiscSys=c2d(ContSys,0.01);
        Ak=DiscSys.A;
        Bk=DiscSys.B;
        Sk=squeeze(S(:,:,length(t)-i+1));
        Vk=V(:,length(t)+1-i);
        Qx=Ak'*Vk+Q*X(:,length(t)-i);
        Quu=R+Bk'*Sk*Bk;
        Qux=Bk'*Sk*Ak;
        Qu=Bk'*Vk+R*U(:,length(t)-i);
        Qxx=Q+Ak'*Sk*Ak;
        % Compute the eigen values of the Hessian (second derivative
        % information)
        [P,D]=eig(Quu);
        d=diag(D);
        d(d<0) = 0;
        % Set the eigen values to 0 to ensure stability
        d=d+lambda(iter_num);
        Quu = P*diag(1./d)*P';
        k=-Quu*Qu;
        K=-Quu*Qux;
        Sk=Qxx-K'*Quu*K;
        Vk=Qx-K'*Quu*k;
        Kk(:,length(t)-i)=k;
        KK(:,:,length(t)-i)=K;
        V(:,length(t)-i)=Vk;
        S(:,:,length(t)-i)=Sk;
    end
    % Forward pass to evaluate control and new states
    X_n = X*0;
    X_n(:,1)=X(:,1);
    Un=zeros(2,length(t)-1);
    for i=1:length(t)-1
        k=squeeze(Kk(:,i));
        K=squeeze(KK(:,:,i));
        du=K*(X_n(:,i)-X(:,i))+k;
        Un(:,i)=du+U(:,i);
        
        % Forward integration of the state
        x1=X_n(1,i);x2=X_n(2,i);x3=X_n(3,i);x4=X_n(4,i);
        A1=double(subs(A));
        B1=double(subs(B));
        ContSys=ss(A1,B1,C1,0);
        DiscSys=c2d(ContSys,0.01);
        Ak=DiscSys.A;
        Bk=DiscSys.B;
        
        X_n(:,i+1)=Ak*X_n(:,i)+Bk*Un(:,i);
    end
    % Calculate cost and update lambda
    for k=1:length(t)-1
        J(iter_num+1)=J(iter_num+1)+1/2*X_n(:,k)'*Q*X_n(:,k)+1/2*Un(:,k)'*R*Un(:,k);
    end
    J(iter_num+1)=J(iter_num+1)+1/2*X_n(:,end)'*Qf*X_n(:,end);
    if J(iter_num+1)<J(iter_num)
        lambda(iter_num+1)=lambda(iter_num)/lambdaFactor;
        X=X_n;
        U=Un;
    else
        lambda(iter_num+1)=lambda(iter_num)*lambdaFactor;
    end
    if (abs(J(iter_num+1)-J(iter_num))/J(iter_num)<0.004)
        CostSameCount=CostSameCount+1;
    end
    if CostSameCount>2
        break;
    end

end
X_All(iter_num+1,:,:)=X;
U_All(iter_num+1,:,:)=U;
% Only carrying the iterations that were performed
J=J(J~=0);
lambda=lambda(lambda~=0);
sprintf('Done!')
end

