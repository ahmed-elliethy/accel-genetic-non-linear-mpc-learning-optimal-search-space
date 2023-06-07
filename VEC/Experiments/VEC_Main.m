clear all;  close all; clc;
Duration = 2;  
   [C1,E1,P1] =  VEC_exp1_FG(Duration);
   [C2,E2,P2] =  VEC_exp1_AG(Duration);
x = {'FG','AG'};
%%
figure(1);
subplot(1,3,1)
E = [C1 C2 ];
EE = bar(E);
EE.BarWidth = .4 ;
xticklabels(x);set(gca, 'fontsize', 20)
ylabel('Avg. comp. time [ms]','FontSize', 20);

subplot(1,3,2)
E = [P1 P2];
EE = bar(E);
EE.BarWidth = .4 ;
xticklabels(x);set(gca, 'fontsize', 20)
ylabel('Converg. percent.','FontSize', 24);

subplot(1,3,3)
E = [E1 E2];
EE = bar(E);
EE.BarWidth = .4 ;
xticklabels(x);set(gca, 'fontsize', 20)
ylabel('$E$','Interpreter','latex','FontSize', 24);