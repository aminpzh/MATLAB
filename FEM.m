clear all;clc;close all
format long
tedadeeleman = 'What is the original value? ';
ElementNum = input(tedadeeleman)
tedadegereh = 'What is the tedade gereh? ';
NodeNum=input(tedadegereh);%
darajatazadigheyrefaal = 'What is the darajatazadigheyrefaal? ';
U0Num=input(darajatazadigheyrefaal);
Eelemanha = 'What is the Eelemanha? ';
EMatrix=input(Eelemanha);
Aelemanha = 'What is the Aelemanha? ';
AMatrix=input(Aelemanha);
gerehebteda = 'What is the gerehebteda? ';
amin3=input(gerehebteda);
gerehenteha = 'What is the gerehenteha? ';
emad=input(gerehenteha);
ElementNodes=zeros(ElementNum,2);
ElementNodes(:,1)=amin3
ElementNodes(:,2)=emad
xegerehha = 'What is the xegerehha? ';
amin1=input(xegerehha);
yegerehha = 'What is the yegerehha? ';
emad1=input(yegerehha);
NodeCoordinates=zeros(ElementNum,2)
NodeCoordinates(:,1)=amin1
NodeCoordinates(:,2)=emad1
xx=NodeCoordinates(:,1);
yy=NodeCoordinates(:,2);
AllDof=2*NodeNum;
U=zeros(AllDof,1);
force=zeros(AllDof,1);
niruyegerehha = 'What is the niruyegerehha? ';
amin2=input(niruyegerehha);
force=zeros(AllDof,1);
force(:,1)=amin2; 
DeactivedDof=zeros(U0Num,1);
shomaredarajatazadigheyrefaal = 'What is the shomaredarajatazadigheyrefaal? ';
emad2=input(shomaredarajatazadigheyrefaal);
DeactivedDof=zeros(U0Num,1)
DeactivedDof(:,1)=emad2
stiffness=zeros(AllDof);
for e=1:ElementNum
    E=EMatrix(e);
    A=AMatrix(e);
    EA=E*A;
    ElementCourse=ElementNodes(e,:);
    ElementDof=[[2*ElementNodes(e,1)-1] [2*ElementNodes(e,1)] [2*ElementNodes(e,2)-1] [2*ElementNodes(e,2)]];
    xa=xx(ElementCourse(2))-xx(ElementCourse(1));
    ya=yy(ElementCourse(2))-yy(ElementCourse(1));
    length_element=sqrt(xa*xa+ya*ya);
    cosinus=xa/length_element;
    sinus=ya/length_element;
    ll=length_element;
    tetta=atan(ya/xa);
    k=(EA/ll)*[cosinus^2,0.5*sin(2*tetta),-cosinus^2,-0.5*sin(2*tetta);0.5*sin(2*tetta),sinus^2,-0.5*sin(2*tetta),-sinus^2;-cosinus^2,-0.5*sin(2*tetta),cosinus^2,0.5*sin(2*tetta);-0.5*sin(2*tetta),-sinus^2,0.5*sin(2*tetta),sinus^2];
    stiffness(ElementDof,ElementDof)=stiffness(ElementDof,ElementDof)+k;
    Ke(1:4,e*4-3:e*4)=k;
end
activeDof=setdiff([1:AllDof]', [DeactivedDof]);
U=stiffness(activeDof,activeDof)\force(activeDof);
Displacements=zeros(AllDof,1);
Displacements(activeDof)=U;
disp('***Displacement***')
NNode=1:AllDof;
[NNode' Displacements]
F=stiffness*Displacements;
reactions=F(DeactivedDof);
disp('***Reactions***')
[DeactivedDof reactions]
for p=1:ElementNum
    Ut=Displacements;
    u1=Ut(2*ElementNodes(p,1)-1);
    u2=Ut(2*ElementNodes(p,1));
    u3=Ut(2*ElementNodes(p,2)-1);
    u4=Ut(2*ElementNodes(p,2));
    disp('Internal Force,Stiffness and Displacement Matrix For Element');disp(p)
    ue=[u1;u2;u3;u4]
    ke=Ke(1:4,p*4-3:p*4)
    fe=ke*ue
    
end

