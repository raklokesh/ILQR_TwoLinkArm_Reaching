%% ILQR for two link arm using additive noise
clear all;
for target_num=1:4
    Initialisation;
    X_current=Xin;
    X_t(:,1)=Xin;
    X_est(:,1)=Xin;
    % Generating the nominal control vector and nominal state trajectory
    for i=1:length(t)-1
        x1=X_current(1);x2=X_current(2);x3=X_current(3);x4=X_current(4);
        A1=double(subs(A));
        B1=double(subs(B));
        C1=[1 0 0 0;0 1 0 0];
        ContSys=ss(A1,B1,C1,0);
        DiscSys=c2d(ContSys,0.01);
        A1=DiscSys.A;
        B1=DiscSys.B;
        P=dare(A1,B1,Q,R);
        % Solves discrete time steady state Algebraic Ricatti equation
        % of the form P= Q+A'*P*A-A'*P*B*inv(R+B'PB)*B'*P*A;
        u=-inv(R+B1'*P*B1)*B1'*P*A1*X_current;
        % True states and outputs of the system. Only output is measured
        X_current=A1*X_current+B1*u;
        X_t(:,i+1)=X_current;
        ut(:,i)=u;
    end
    
    U=ut; % Nominal control vector for ILQR
    N=12; % Max No of iterations for ILQR
    switch target_num
        case 1
            [Xn1,Un1,J1,lambda1]=LMA_ILQR(X_t,U,A,B,C,Q,Qf,R,t,N);
%             [Xn11,Un11,J11]=ILQR(X,U,A,B,C,Q,Qf,R,W,V,wn,vn,t,N,th1,th2,th11,th22);
        case 2
            [Xn2,Un2,J2,lambda2]=LMA_ILQR(X_t,U,A,B,C,Q,Qf,R,t,N);
%             [Xn21,Un21,J21]=ILQR(X,U,A,B,C,Q,Qf,R,W,V,wn,vn,t,N,th1,th2,th11,th22);
        case 3
            [Xn3,Un3,J3,lambda3]=LMA_ILQR(X_t,U,A,B,C,Q,Qf,R,t,N);
%             [Xn31,Un31,J31]=ILQR(X,U,A,B,C,Q,Qf,R,W,V,wn,vn,t,N,th1,th2,th11,th22);
        case 4
            [Xn4,Un4,J4,lambda4]=LMA_ILQR(X_t,U,A,B,C,Q,Qf,R,t,N);
%             [Xn41,Un41,J41]=ILQR(X,U,A,B,C,Q,Qf,R,W,V,wn,vn,t,N,th1,th2,th11,th22);
    end
end
