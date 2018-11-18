% =========================================================================
% =         Main file for the filtering demo
% = 
% =         Filipe Rocha - f.rocha41@gmail.com
% = 
% =         GSCAR - COPPE/UFRJ - Rio de Janeiro / Brasil
% =         2018
% =========================================================================
clear all;
close all;
clc;

%% Simulation Parameters

% Plot style
%   1 - Demonstracao do filtro simples
%   2 - Erro por parametro de cada filtro
%   3 - Comparacao geral de todos os obj_filters e erro absoluto de posicao
%   4 - Filtro de particulas
%   5 - Filtro de Bayes
conf_plotStyle = 5;

%% Instanciacao de objetos

% Cria um objeto para a comunicacao com o V-REP
obj_vrepComm = class_vrep_manipulation;

% Cria um objeto de obj_filters
obj_filters = class_filters;

 
%% Inicio do tratamento

% Figura onde serao plotados graficos
fig1 = figure;
hold on;
grid on;

% Variaveis auxiliares
flag_first_read = true;

% Inicia o loop com aux_i = 2
aux_i = 1;


% Apenas um loop infinito
disp('Iniciado o loop');
 
 while(1)
     
     %% ======= PRE TRATAMENTO =================
     
     % Pequeno pause
     pause(0.05);  
    
     % Obtendo os dados necessarios da simulacao
     [eul_ang, p_mundo_robo, vel, tempo_atual] = obj_vrepComm.obterDadosSim;
     
     %Primeiro tempo a ser lido
     if flag_first_read == 1
         tempo_first = tempo_atual;
     end
         
     % Salvando o tempo atual do calculo
     base_tempo(aux_i) = tempo_atual - tempo_first;
     disp(strcat('Tempo de simulacao: ',num2str(base_tempo(aux_i))));
     
     % Atribuindo o vetor com as leituras reais o V-REP
     x_n(1:4, aux_i) = [p_mundo_robo(1), p_mundo_robo(2) vel(1) vel(2)]';
     
     % Tratando dados para os obj_filters
     if flag_first_read
         
     else
          % Calculando o deltaT
         deltaT = base_tempo(length(base_tempo)) - base_tempo(length(base_tempo)-1);
     end
     
     %% ============ CALCULO DOS obj_filters ======================
         
     %% Tratamento para o filtro de Kalman Simples
     
     % Criando uma leitura com ruido 
     kf_y_n(1:4, aux_i) = obj_filters.kf_adicionarRuidoLeitura(x_n(1:4, aux_i));
     
     % Inicializa os parametros do filtro, caso seja o primeiro scan
     if flag_first_read
         
         % Inicializa a posicao com dados obtidos diretamente do simulador
         kf_m_n(1:4, aux_i) = x_n(1:4, aux_i);
         
         % Inicializa a matriz de covariancia como uma I_2x2
         kf_P_n(1:4, 1:4, aux_i) = eye(length(x_n));
         
         % Comeca a filtragem caso seja aux_i >= 2
     else
         
         % Chamando o filtro de Kalman
         [kf_m_n(1:4,aux_i), kf_P_n(1:4,1:4,aux_i)] = obj_filters.kf(kf_m_n(1:4,aux_i-1), kf_P_n(1:4,1:4,aux_i-1), kf_y_n(1:4, aux_i), deltaT);
     end
     
     %% Tratamento para o filtro de Kalman EXTENDIDO
     
     % Criando uma leitura com ruido
     ekf_y_n(1:3, aux_i) = obj_filters.ekf_adicionarRuidoLeitura(x_n(1:4, aux_i));
     
     % Inicializa os parametros do filtro, caso seja o primeiro scan
     if flag_first_read
         
         % Inicializa a posicao com dados obtidos diretamente do simulador
         ekf_m_n(1:4, aux_i) = x_n(1:4, aux_i);
         
         % Inicializa a matriz de covariancia como uma I_4x4
         ekf_P_n(1:4, 1:4, aux_i) = eye(length(x_n));
         
         % Comeca a filtragem caso seja aux_i >= 2
     else
         
         % Chamando o filtro de Kalman Extendido
         [ekf_m_n(1:4,aux_i), ekf_P_n(1:4,1:4,aux_i)] = obj_filters.ekf(ekf_m_n(1:4,aux_i-1), ekf_P_n(1:4,1:4,aux_i-1), ekf_y_n(1:3, aux_i), deltaT);
     end
     
     %% Tratamento para a utilizacao do filtro de Kalman UNSCENTED
     
     % Inicializacao dos parametros para o UKF
     if flag_first_read
         
         % Inicializa a posicao com dados obtidos diretamente do simulador
         ukf_m_n(1:4, aux_i) = x_n(1:4, aux_i);
         
         % Inicializa a matriz de covariancia como uma I_4x4
         ukf_P_n(1:4, 1:4, aux_i) = eye(length(x_n));
         
     else
         
         % Chamando o filtro de Kalman Unscented
         % Note que a entrada de sensor utilizada eh a mesma para o EKF
         [ukf_m_n(1:4,aux_i), ukf_P_n(1:4,1:4,aux_i)] = obj_filters.ukf(ukf_m_n(1:4,aux_i-1), ukf_P_n(1:4,1:4,aux_i-1), ekf_y_n(1:3, aux_i), deltaT);
         
     end
     
     %% Tratamento para a utilizacao do filtro de PARTICULAS
     
     % Inicializacao dos parametros para o FP
     % O filtro de particulas é calculado para as duas situacoes de
     % resample requisitadas
     if flag_first_read
        
          % Inicia as particulas com media baseada na posicao atual real do
          % robo
          fp1_xCal(:,:,aux_i) = obj_filters.fp_estInicial(x_n(1:4, aux_i));
          fp2_xCal(:,:,aux_i) = obj_filters.fp_estInicial(x_n(1:4, aux_i));
          
          % Inicializa a primeira predicao do filtro de particulas como a
          % posicao real do robo
          fp1_m_n(:,aux_i) = x_n(1:4, aux_i);
          fp2_m_n(:,aux_i) = x_n(1:4, aux_i);
         
     % Quando nao eh a primeira execucao
     else
         
         % Invoca o filtro de particulas com metodo de resample uniforme
         fp1_xCal = obj_filters.fp(fp1_xCal, [], ekf_y_n(1:3, aux_i), deltaT, 1);
         
         % Invoca o filtro de particulas com o metodo de resample low
         % variance
         fp2_xCal = obj_filters.fp(fp2_xCal, [], ekf_y_n(1:3, aux_i), deltaT, 2);
         
         % Obtem a previsao do filtro de particulas a partir da metrica de
         % media das particulas
         fp1_m_n(:, aux_i) = obj_filters.fp_obter_m_n(fp1_xCal);
         fp2_m_n(:, aux_i) = obj_filters.fp_obter_m_n(fp2_xCal);
         
     end
     
     %% Tratamento para a utilizacao do filtro de bayes
     
     % Executa a inicializacao do filtro de bayes
     if flag_first_read
         
         % Gera o inicializador do filtro de Bayes
         %fb_Pk_n = obj_filters.fb_inicializacao();    
         
         % Encontra a predicao do filtro de Bayes para o mapa inicial
         % fb_X_n = obj_filters.fb_obter_m_n(fb_Pk_n);
         fb_m_n(:,aux_i) = x_n(1:2, aux_i);
          
     % Chamada recursiva da funcao do filtro
     else
         
         % Gera os pontos em torno da medicao
         fb_xP = obj_filters.fb_gerarPontos(kf_y_n(1:2,aux_i));
         
         % Chama o filtro de Bayes
         fb_Pk_n = obj_filters.fb(fb_xP);
         
         % De acordo com o mapa gerado pelo filtro de Bayes, encontra sua
         % predicao do estado do robo
         fb_m_n(:,aux_i) = obj_filters.fb_obter_m_n(fb_Pk_n);
               
     end
     
     
     %% ====== POS TRATAMENTO ========================
     
     %% Calculo do erro dos obj_filters
     
     % Erros dos parametros   
     geral_erroKF(1:4, aux_i) = abs(x_n(1:4,aux_i) - kf_m_n(1:4,aux_i));
     geral_erroEKF(1:4, aux_i) =  abs(x_n(1:4,aux_i) - ekf_m_n(1:4,aux_i));
     geral_erroUKF(1:4, aux_i) =  abs(x_n(1:4,aux_i) - ukf_m_n(1:4,aux_i));
     geral_erroFP1(1:4, aux_i) = abs(x_n(1:4,aux_i) - fp1_m_n(1:4,aux_i));
     geral_erroFP2(1:4, aux_i) = abs(x_n(1:4,aux_i) - fp2_m_n(1:4,aux_i));
     
     % Erro absoluto de posicao
     geral_erroKF_pos(aux_i) = sqrt((x_n(1,aux_i)- kf_m_n(1,aux_i))^2 + (x_n(2,aux_i)- kf_m_n(2,aux_i))^2);
     geral_erroEKF_pos(aux_i) = sqrt((x_n(1,aux_i)- ekf_m_n(1,aux_i))^2 + (x_n(2,aux_i)- ekf_m_n(2,aux_i))^2);
     geral_erroUKF_pos(aux_i) = sqrt((x_n(1,aux_i)- ukf_m_n(1,aux_i))^2 + (x_n(2,aux_i)- ukf_m_n(2,aux_i))^2);
     geral_erroFP1_pos(aux_i) = sqrt((x_n(1,aux_i)- fp1_m_n(1,aux_i))^2 + (x_n(2,aux_i)- fp1_m_n(2,aux_i))^2);
     geral_erroFP2_pos(aux_i) = sqrt((x_n(1,aux_i)- fp2_m_n(1,aux_i))^2 + (x_n(2,aux_i)- fp2_m_n(2,aux_i))^2);
     geral_erroFB_pos(aux_i) = sqrt((x_n(1,aux_i)- fb_m_n(1,aux_i))^2 + (x_n(2,aux_i)- fb_m_n(2,aux_i))^2);
     
     %% Atualizando os graficos
     
     if ~flag_first_read
         [~, num_leit] = size(x_n);
         
         % Switch que decide que tipo de plot sera printado
         switch conf_plotStyle
             % Demonstracao do KF
             case 1
                 
                 plot(0, 0, 'dk');
                 plot([x_n(1,num_leit) x_n(1,num_leit-1)], [x_n(2,num_leit) x_n(2,num_leit-1)],'--k');
                 plot([kf_y_n(1,num_leit) kf_y_n(1,num_leit-1)], [kf_y_n(2,num_leit) kf_y_n(2,num_leit-1)],'-r');
                 plot([kf_m_n(1,num_leit) kf_m_n(1,num_leit-1)], [kf_m_n(2,num_leit) kf_m_n(2,num_leit-1)],'-b');
                 
                 legend('Origem', 'Pos. Real','EKF','UKF');
              
             % Plota o erro absoluto em x
             case 2
                 
                 % Plota o erro em xd
                 subplot(4,1,1);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(1,aux_i-1) geral_erroKF(1,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(1,aux_i-1) geral_erroEKF(1,aux_i)],'-r');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(1,aux_i-1) geral_erroUKF(1,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP1(1,aux_i-1) geral_erroFP1(1,aux_i)],'-c');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP2(1,aux_i-1) geral_erroFP2(1,aux_i)],'-y');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP2(1,aux_i-1) geral_erroFP2(1,aux_i)],'-y');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF', 'Erro FP1', 'Erro FP2');
                 
             % Plota o erro absoluto em y
                 subplot(4,1,2);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(2,aux_i-1) geral_erroKF(2,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(2,aux_i-1) geral_erroEKF(2,aux_i)],'-r');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(2,aux_i-1) geral_erroUKF(2,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP1(2,aux_i-1) geral_erroFP1(2,aux_i)],'-c');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP2(2,aux_i-1) geral_erroFP2(2,aux_i)],'-y');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF', 'Erro FP1', 'Erro FP2');
               
             % Plota o erro absoluto em xd    
                 subplot(4,1,3);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(3,aux_i-1) geral_erroKF(3,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(3,aux_i-1) geral_erroEKF(3,aux_i)],'-r');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(3,aux_i-1) geral_erroUKF(3,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP1(3,aux_i-1) geral_erroFP1(3,aux_i)],'-c');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP2(3,aux_i-1) geral_erroFP2(3,aux_i)],'-y');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF', 'Erro FP1', 'Erro FP2');
                 
             % Plota o erro absoludo em yd  
                 subplot(4,1,4);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(4,aux_i-1) geral_erroKF(4,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(4,aux_i-1) geral_erroEKF(4,aux_i)],'-r');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(4,aux_i-1) geral_erroUKF(4,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP1(4,aux_i-1) geral_erroFP1(4,aux_i)],'-c');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP2(4,aux_i-1) geral_erroFP2(4,aux_i)],'-y');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF', 'Erro FP1', 'Erro FP2');
                 
             % Plota todos os obj_filters e o erro absoluto
             case 3
                 
                 % Plotando as trajetorias
                 
                 subplot(2,1,1);
                 hold on;
                 plot(0,0,'dk');
                 plot([x_n(1,num_leit) x_n(1,num_leit-1)], [x_n(2,num_leit) x_n(2,num_leit-1)],'--k');
                 plot([ekf_m_n(1,num_leit) kf_m_n(1,num_leit-1)], [kf_m_n(2,num_leit) kf_m_n(2,num_leit-1)],'-g');
                 plot([ekf_m_n(1,num_leit) ekf_m_n(1,num_leit-1)], [ekf_m_n(2,num_leit) ekf_m_n(2,num_leit-1)],'-r');
                 plot([ukf_m_n(1,num_leit) ukf_m_n(1,num_leit-1)], [ukf_m_n(2,num_leit) ukf_m_n(2,num_leit-1)],'-b');
                 plot([fp1_m_n(1,num_leit) fp1_m_n(1,num_leit-1)], [fp1_m_n(2,num_leit) fp1_m_n(2,num_leit-1)],'-c');
                 plot([fp2_m_n(1,num_leit) fp2_m_n(1,num_leit-1)], [fp2_m_n(2,num_leit) fp2_m_n(2,num_leit-1)],'-y');
                 plot([fb_m_n(1,num_leit) fb_m_n(1,num_leit-1)], [fb_m_n(2,num_leit) fb_m_n(2,num_leit-1)],'-m');
                 legend('Origem', 'Pos. Real','KF','EKF','UKF','FP1','FP2','FB');
                 axis([-5 5 -5 5]);
                 hold off;
                 
                 % Plotando os erros
                 subplot(2,1,2);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF_pos(aux_i-1) geral_erroKF_pos(aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF_pos(aux_i-1) geral_erroEKF_pos(aux_i)],'-r');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF_pos(aux_i-1) geral_erroUKF_pos(aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP1_pos(aux_i-1) geral_erroFP1_pos(aux_i)],'-c');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFP2_pos(aux_i-1) geral_erroFP2_pos(aux_i)],'-y');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroFB_pos(aux_i-1) geral_erroFB_pos(aux_i)],'-m');
                 legend('KF','EKF','UFK', 'FP1', 'FP2','FB');
                 hold off;
                 
             % Plotando os obj_filters de particulas
             case 4
                 
                  % Plotando o resultado para o filtro de particulas
                  %plot([x_n(1,num_leit) x_n(1,num_leit-1)], [x_n(2,num_leit) x_n(2,num_leit-1)],'--g');
                  
                  % Plota o filtro de particulas com o primeiro metodo de
                  % resample
                  subplot(2,1,1);
                 
                  plot(x_n(1,num_leit), x_n(2, num_leit), 'ob');
                  hold on;
                  plot(fp1_m_n(1,num_leit), fp1_m_n(2,num_leit), 'or');
                  plot(fp1_xCal(1,:), fp1_xCal(2,:),'xk');
                  %pbaspect([1 1 1]);
                  hold off;
                  axis([-10 10 -10 10]);
                  
                  % Plota o filtro de particulas com o segundo metodo de
                  % resample
                  subplot(2,1,2);
                  plot(x_n(1,num_leit), x_n(2, num_leit), 'ob');
                  hold on;
                  plot(fp2_m_n(1,num_leit), fp2_m_n(2,num_leit), 'or');
                  plot(fp2_xCal(1,:), fp2_xCal(2,:),'xk');
                  hold off;
                  %pbaspect([1 1 1]);
                  axis([-5 5 -5 5]);
                  
             % Plotando para o filtro de Bayes
             case 5
      
                 mesh(fb_Pk_n);
       
             otherwise
                           
         % Fim do switch case
         end
     
     % Caso seja o primeiro scan
     else
         switch conf_plotStyle
             
             % Inicializacao do grafico tipo 2
             case 2
                 
                 % Plota o erro em xd
                 subplot(4,1,1);
                 hold on;
                 title('Erro absoluto em x');
                 xlabel('tempo [s]'); ylabel('erro [m]');
                 
             % Plota o erro absoluto em y
                 subplot(4,1,2);
                 title('Erro absoluto em y');
                 xlabel('tempo [s]'); ylabel('erro [m]');
               
             % Plota o erro absoluto em xd    
                 subplot(4,1,3);
                 title('Erro absoluto em xd');
                 xlabel('tempo [s]'); ylabel('erro [m/s]');
                 
             % Plota o erro absoludo em yd  
                 subplot(4,1,4);
                 title('Erro absoluto em yd');
                 xlabel('tempo [s]'); ylabel('erro [m/s]');
             
             % Inicializacao do grafico tipo 3
             case 3
                 
                 % Trajetoria
                 subplot(2,1,1);
                 grid on;
                 pbaspect([1 1 1]);
                 title('Comparacao KF, EKF e UKF');
                 xlabel('pos x [m]'); ylabel('pos y [m]');
                 hold off;
                 
                 % Plotando os erros
                 subplot(2,1,2);
                 grid on;
                 
                 title('Comp. Erro absoluto de Pos.');
                 xlabel('tempo [t]'); ylabel('erro absoluto [m]');
                 hold off;
                 
             % Inicializacao do grafico dos obj_filters de particulas
             case 4
                 
                  % resample
                  subplot(2,1,1);
                  pbaspect([1 1 1]);
                  axis([-10 10 -10 10]);
                  title('Filtro de Particulas - Uniform');
                  xlabel('pos x [m]'); ylabel('pos y [m]');
                  
                  % Plota o filtro de particulas com o segundo metodo de
                  % resample
                  subplot(2,1,2);
                  pbaspect([1 1 1]);
                  axis([-10 10 -10 10]);
                  title('Filtro de Particulas - Low Variance');
                  xlabel('pos x [m]'); ylabel('pos y [m]');
                 
             otherwise
                 disp('Nenhum tipo de grafico selecionado');
                 close all;       
                 
         % Fim do switch case
         end
         
     % Fim do if
     end
                 
     %% Pos processamento
     % Incrementa um indice na contagem
     aux_i = aux_i + 1;
     
     % Indica que já foi passada a primeira leitura
     if flag_first_read
         flag_first_read = false;
     end
 end
 



