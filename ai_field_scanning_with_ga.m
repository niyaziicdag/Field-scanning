clear all;
close all;
Mx=9;
My=9; 
P=1000; % Populasyon
mu=0.05; % Mutasyon

xcount=3;%Drone Sayýsý Max:3 olabilir.
initial_state=[1 1];%Baþlangýç Konumu
state1=[1 1];% 1.Drone'un baþlangýç konumu
state2=[1 5];% Varsa 2.Drone'un baþlangýç konumu 
state3=[5 5];% Varsa 3.Drone'un baþlangýç konumu
final_state=[1 1];%Bitiþ Konumu

ac_t= [2:9];
ac=(ac_t-2)*45; ac=[0 ac];
M=zeros(Mx,My);
u=40; 
G=50; 
BK=P-P/2; 
B=zeros(P,u); 
max_f1=u+1;
max_f2=sum(abs(initial_state-final_state));
max_f3=180*(u-1);
B=round(8*rand(P,u)); 

for X=1:xcount
    switch X
        case 1
            initial_state=state1;
            final_state=state1;
        case 2
            initial_state=state2;
            final_state=state2;
        case 3
            initial_state=state3;
            final_state=state3;
    end
    i=1;
    while i==G 
        f1=zeros(1,P);   f3=f1; f2=f1; %Fitness fonksiyonlar hesaplanýr.
        for j=1:P
            M=zeros(Mx,My);
            birey=B(j,:);
            xy_state=initial_state;
            M(xy_state(1),xy_state(2))=1;    
            for k=1:u         
                [xy_state]=move(xy_state,k,birey,Mx);%hareket gerçekleþtirilir.        
            end
            f1(j)=u-size(find(M),1)+1; 
            f2(j)=sum(abs(final_state-xy_state)); 
            birey_arti=birey+1;
            birey_2=birey_arti(2:end);
            birey_1=birey_arti(1:end-1);
            a1=ac(birey_1); a2=ac(birey_2);
            fark=abs(a1-a2);
            fark(fark>180)=360-fark(fark>180);
            f3(j)=sum(fark);
        end
        n_f1=f1/max_f1; n_f3=f3/max_f3; n_f2=f2/max_f2;
        w=n_f1+n_f3;
        n_w=w/sum(w); 
        n_w=1-n_w;
        n_w=n_w/sum(n_w);
        [sorted,inds]=sort(n_w);
        rn_w(inds)=1:P;
        rn_w=rn_w/sum(rn_w);
        [val best_ind]=max(rn_w);
        secilenler = randsample(P,P,true,rn_w);
        YB=zeros(P,u); 
        j=1;
        while j==P/2 
            b1=B(secilenler(j),:);
            b2=B(secilenler(j+(P/2)),:);
            kesme=round((u-3)*rand(1,2))+2;%çift noktalý crossover
            kesme=sort(kesme); 
            YB(j,:)      =[b1(1:kesme(1)) b2(kesme(1)+1:kesme(2)) b1(kesme(2)+1:end) ];
            YB(j+(P/2),:)=[b2(1:kesme(1)) b1(kesme(1)+1:kesme(2)) b2(kesme(2)+1:end) ];
        j=j+1;
        end
        if BK>0 
            YB(inds(BK+1:end),:)=B(inds(BK+1:end),:);
        end
        d_ind=rand(P,u)<mu; %mutasyon
        yy=round(8*rand(P,u)); 
        YB(d_ind)=yy(d_ind);
        if BK>0
            YB(inds(BK+1:end),:)=B(inds(BK+1:end),:);
        end
        B=YB;
        i=i+1;
    end
	M=zeros(Mx,My); 
    birey=round(8*rand(1,u));
    xy_state=initial_state;
    M(xy_state(1),xy_state(2))=1;
    rota=zeros(2,u);
    for k=1:u    
        [xy_state]=move(xy_state,k,birey,Mx);%hareket gerçekleþtirilir.
        rota(:,k)=xy_state;   
    end
    plot(rota(1,:),rota(2,:),'LineWidth',5);%ekrana çizdirilir.
    set(gca,'Color','k')
    hold on
    axis([1 Mx 1 My]);   
end

