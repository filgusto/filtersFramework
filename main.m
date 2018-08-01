% Codigo para a utilização de filtros em uma plataforma robótica movel no
% plano
% Filipe Rocha
% f.rocha41@gmail.com
% COPPE/UFRJ
% agosto de 2018
clear all;
close all;
clc;

%% Parametros da simulacao

% Estilo do plot
%   1 - Demonstracao do filtro simples
%   2 - Erro por parametro de cada filtro
%   3 - Comparacao geral de todos os filtros e erro absoluto de posicao
tipo_plot = 3;

%% Instanciacao de objetos

% Cria um objeto para a comunicacao com o V-REP
vrepComm = classeManipulacaoVREP;

% Cria um objeto de Filtros
filtros = classeFiltrosAutonomos;

 
%% Inicio do tratamento

% Figura onde serao plotados graficos
fig1 = figure;
hold on;
grid on;

% Variaveis auxiliares
flag_first_read = true;

% Inicia o loop com aux_i = 2
aux_i = 1;

% Variaveis para o filtro de Kalman
%base_tempo = [];
%kf_P_n = [];
%x_n = [];
%y_n = [];
 

% Apenas um loop infinito
disp('Iniciado o loop');
 
 while(1)
     
     %% ======= PRE TRATAMENTO =================
     
     % Pequeno pause
     pause(0.01);  
    
     % Obtendo os dados necessarios da simulacao
     [eul_ang, p_mundo_robo, vel, tempo_atual] = vrepComm.obterDadosSim;
     
     %Primeiro tempo a ser lido
     if flag_first_read == 1
         tempo_first = tempo_atual;
     end
         
     % Salvando o tempo atual do calculo
     base_tempo(aux_i) = tempo_atual - tempo_first;
     
     % Atribuindo o vetor com as leituras reais o V-REP
     x_n(1:4, aux_i) = [p_mundo_robo(1), p_mundo_robo(2) vel(1) vel(2)]';
     
     % Tratando dados para os filtros
     if flag_first_read
         
     else
          % Calculando o deltaT
         deltaT = base_tempo(length(base_tempo)) - base_tempo(length(base_tempo)-1);
     end
     
     %% ============ CALCULO DOS FILTROS ======================
         
     %% Tratamento para o filtro de Kalman Simples
     
     % Criando uma leitura com ruido 
     kf_y_n(1:4, aux_i) = filtros.kf_adicionarRuidoLeitura(x_n(1:4, aux_i));
     
     % Inicializa os parametros do filtro, caso seja o primeiro scan
     if flag_first_read
         
         % Inicializa a posicao com dados obtidos diretamente do simulador
         kf_m_n(1:4, aux_i) = x_n(1:4, aux_i);
         
         % Inicializa a matriz de covariancia como uma I_2x2
         kf_P_n(1:4, 1:4, aux_i) = eye(length(x_n));
         
         % Comeca a filtragem caso seja aux_i >= 2
     else
         
         % Chamando o filtro de Kalman
         [kf_m_n(1:4,aux_i), kf_P_n(1:4,1:4,aux_i)] = filtros.kf(kf_m_n(1:4,aux_i-1), kf_P_n(1:4,1:4,aux_i-1), kf_y_n(1:4, aux_i), deltaT);
     end
     
     %% Tratamento para o filtro de Kalman EXTENDIDO
     
     % Criando uma leitura com ruido
     ekf_y_n(1:3, aux_i) = filtros.ekf_adicionarRuidoLeitura(x_n(1:4, aux_i));
     
     % Inicializa os parametros do filtro, caso seja o primeiro scan
     if flag_first_read
         
         % Inicializa a posicao com dados obtidos diretamente do simulador
         ekf_m_n(1:4, aux_i) = x_n(1:4, aux_i);
         
         % Inicializa a matriz de covariancia como uma I_4x4
         ekf_P_n(1:4, 1:4, aux_i) = eye(length(x_n));
         
         % Comeca a filtragem caso seja aux_i >= 2
     else
         
         % Chamando o filtro de Kalman Extendido
         [ekf_m_n(1:4,aux_i), ekf_P_n(1:4,1:4,aux_i)] = filtros.ekf(ekf_m_n(1:4,aux_i-1), ekf_P_n(1:4,1:4,aux_i-1), ekf_y_n(1:3, aux_i), deltaT);
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
         [ukf_m_n(1:4,aux_i), ukf_P_n(1:4,1:4,aux_i)] = filtros.ukf(ukf_m_n(1:4,aux_i-1), ukf_P_n(1:4,1:4,aux_i-1), ekf_y_n(1:3, aux_i), deltaT);
         
     end
     
     %% ====== POS TRATAMENTO ========================
     
     %% Calculo do erro dos filtros
     
     % Erros dos parametros 
     geral_erroKF(1:4, aux_i) = abs(x_n(1:4,aux_i) - kf_m_n(1:4,aux_i));
     geral_erroEKF(1:4, aux_i) =  abs(x_n(1:4,aux_i) - ekf_m_n(1:4,aux_i));    
     geral_erroUKF(1:4, aux_i) =  abs(x_n(1:4,aux_i) - ukf_m_n(1:4,aux_i));
    
     % Erro absoluto de posicao
      geral_erroKF_pos(aux_i) = sqrt((x_n(1,aux_i)- kf_m_n(1,aux_i))^2 + (x_n(2,aux_i)- kf_m_n(2,aux_i))^2);
      geral_erroEKF_pos(aux_i) = sqrt((x_n(1,aux_i)- ekf_m_n(1,aux_i))^2 + (x_n(2,aux_i)- ekf_m_n(2,aux_i))^2);
      geral_erroUKF_pos(aux_i) = sqrt((x_n(1,aux_i)- ukf_m_n(1,aux_i))^2 + (x_n(2,aux_i)- ukf_m_n(2,aux_i))^2);
     
     %% Atualizando os graficos
     
     if ~flag_first_read
         [~, num_leit] = size(x_n);
         
         % Switch que decide que tipo de plot sera printado
         switch tipo_plot
             % Demonstracao do KF
             case 1
                 
                 plot(0, 0, 'dk');
                 plot([x_n(1,num_leit) x_n(1,num_leit-1)], [x_n(2,num_leit) x_n(2,num_leit-1)],'--g');
                 plot([kf_y_n(1,num_leit) kf_y_n(1,num_leit-1)], [kf_y_n(2,num_leit) kf_y_n(2,num_leit-1)],'-r');
                 plot([kf_m_n(1,num_leit) kf_m_n(1,num_leit-1)], [kf_m_n(2,num_leit) kf_m_n(2,num_leit-1)],'-b');
                 
                 legend('Origem', 'Pos. Real','EKF','UKF');
              
             % Plota o erro absoluto em x
             case 2
                 
                 % Plota o erro em xd
                 subplot(4,1,1);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(1,aux_i-1) geral_erroKF(1,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(1,aux_i-1) geral_erroEKF(1,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(1,aux_i-1) geral_erroUKF(1,aux_i)],'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
                 
             % Plota o erro absoluto em y
                 subplot(4,1,2);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(2,aux_i-1) geral_erroKF(2,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(2,aux_i-1) geral_erroEKF(2,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(2,aux_i-1) geral_erroUKF(2,aux_i)],'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
               
             % Plota o erro absoluto em xd    
                 subplot(4,1,3);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(3,aux_i-1) geral_erroKF(3,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(3,aux_i-1) geral_erroEKF(3,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(3,aux_i-1) geral_erroUKF(3,aux_i)],'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
                 
             % Plota o erro absoludo em yd  
                 subplot(4,1,4);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF(4,aux_i-1) geral_erroKF(4,aux_i)],'-b');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF(4,aux_i-1) geral_erroEKF(4,aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF(4,aux_i-1) geral_erroUKF(4,aux_i)],'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');

                 
             % Plota todos os filtros e o erro absoluto
             case 3
                 
                 % Plotando as trajetorias
                 
                 subplot(2,1,1);
                 hold on;
                 plot([x_n(1,num_leit) x_n(1,num_leit-1)], [x_n(2,num_leit) x_n(2,num_leit-1)],'--k');
                 plot([ekf_m_n(1,num_leit) kf_m_n(1,num_leit-1)], [kf_m_n(2,num_leit) kf_m_n(2,num_leit-1)],'-g');
                 plot([ekf_m_n(1,num_leit) ekf_m_n(1,num_leit-1)], [ekf_m_n(2,num_leit) ekf_m_n(2,num_leit-1)],'-r');
                 plot([ukf_m_n(1,num_leit) ukf_m_n(1,num_leit-1)], [ukf_m_n(2,num_leit) ukf_m_n(2,num_leit-1)],'-b');
                 legend('Origem', 'Pos. Real','KF','EKF','UKF');
                 hold off;
                 
                 % Plotando os erros
                 subplot(2,1,2);
                 hold on;
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroKF_pos(aux_i-1) geral_erroKF_pos(aux_i)],'-g');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroEKF_pos(aux_i-1) geral_erroEKF_pos(aux_i)],'-r');
                 plot([base_tempo(aux_i-1) base_tempo(aux_i)],[geral_erroUKF_pos(aux_i-1) geral_erroUKF_pos(aux_i)],'-b');
                 legend('KF','EKF','UFK');
                 hold off;
      
             otherwise
                 
            
         % Fim do switch case
         end
     
     % Caso seja o primeiro scan
     else
         switch tipo_plot
             
             % Inicializacao do grafico tipo 2
             case 2
                 
                 % Plota o erro em xd
                 subplot(4,1,1);
                 hold on;
                 plot(0,0,'-b');
                 plot(0,0,'-g');
                 plot(0,0,'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
                 title('Erro absoluto em x');
                 xlabel('tempo [s]'); ylabel('erro [m]');
                 
             % Plota o erro absoluto em y
                 subplot(4,1,2);
                 hold on;
                 plot(0,0,'-b');
                 plot(0,0,'-g');
                 plot(0,0,'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
                 title('Erro absoluto em y');
                 xlabel('tempo [s]'); ylabel('erro [m]');
               
             % Plota o erro absoluto em xd    
                 subplot(4,1,3);
                 hold on;
                 plot(0,0,'-b');
                 plot(0,0,'-g');
                 plot(0,0,'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
                 title('Erro absoluto em xd');
                 xlabel('tempo [s]'); ylabel('erro [m/s]');
                 
             % Plota o erro absoludo em yd  
                 subplot(4,1,4);
                 hold on;
                 plot(0,0,'-b');
                 plot(0,0,'-g');
                 plot(0,0,'-k');
                 legend('Erro KF', 'Erro EKF', 'Erro UKF');
                 title('Erro absoluto em yd');
                 xlabel('tempo [s]'); ylabel('erro [m/s]');
             
             % Inicializacao do grafico tipo 3
             case 3
                 
                 % Trajetoria
                 subplot(2,1,1);
                 hold on;
                 grid on;
                 plot(0, 0, 'dk');
                 plot(0, 0,'--k');
                 plot(0, 0,'-g');
                 plot(0, 0,'-r');
                 plot(0, 0,'-b');
                 pbaspect([1 1 1]);
                 title('Comparacao KF, EKF e UKF');
                 xlabel('pos x [m]'); ylabel('pos y [m]');
                 hold off;
                 
                 % Plotando os erros
                 subplot(2,1,2);
                 hold on;
                 grid on;
                 plot(0, 0,'-g');
                 plot(0, 0,'-r');
                 plot(0, 0,'-b');
                 
                 title('Comp. Erro absoluto de Pos.');
                 xlabel('tempo [t]'); ylabel('erro absoluto [m]');
                 hold off;
                 
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
 



