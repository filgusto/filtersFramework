classdef classeFiltrosAutonomos
   % Classe para a definicao dos filtros do trabalho de autonomos
       
   %% Definicao das propriedades
    properties
        
        %% Parametros do filtro de Kalman classico
        % Desvio Padrao em x
        kf_desv_x = 0.1;
        
        % Desvio Padrao da medida
        kf_desv_y = 0.3;
        
        % Matriz de medicao
        kf_H_n = eye(4);
        
        % MAtriz de cov da medicao
        kf_R_n = eye(4);
        
        %% Parametros do filtro de Kalman Extendido
        
        % Desvio Padrao da medida
        ekf_desv_y = 0.2;
        
        % Matriz R_n
        ekf_R_n = 0;
        
        
        %% Parametros do filtro de Kalman Unscented
        ukf_lambda = [];
        ukf_desv_x = [];
        ukf_desv_y = [];
        ukf_R_t = [];
        ukf_Q_t = [];
        
        %% Parametros do filtro de PARTICULAS
        
        % Covariancia no sistema
        fp_desv_x = 0.5;
        
        % Covariancia na medida
        fp_desv_y = 0.5;
        
        % Numero de particulas
        fp_M = 100;
        
        % Variancia do estado inicial
        fp_V = 3;
        
        % Matriz de covariancia do sistema
        fp_R_t = eye(3);        
        
        %% Parametros do filtro de Bayes discreto
        
        % Numero de pontos aproximados
        fb_N = 20;
        
        % Desvio padrao da medicao
        fb_desvBayes = 0.5;
                
        % Resolucao do Mapa
        fb_resMapa = 0.2;
        
        % Mapa discreto
        fb_Sx = [];
        fb_Sy = [];
        
        % Matriz de covariancia para encontrar a posicao no plano
        fb_K = [];
    end
    
    %% =========== Definicao dos metodos =========================
    methods
        
        %% --- Construtor ---
        function obj = classeFiltrosAutonomos(obj, estado_inicial, matriz_cov)
            
            %% Inicializacoes do Filtro de Kalman Simples
            % Matriz de cov da medicao
            obj.kf_R_n = obj.kf_desv_y * eye(4);
            
            %% Inicializacoes do filtro de Kalman extendido
            
            % Matriz de covariancia da medicao
            obj.ekf_R_n = obj.ekf_desv_y*eye(3);
            
            %% Inicializacoes do filtro de Kalman UNSCENTED
            
            % Fator de escala (o quao longe sigmaPoint fica da media
            obj.ukf_lambda = 1.8;
            
            % Desvio na imprevisibilidade do sistema
            obj.ukf_desv_x = 0.2;
            obj.ukf_desv_y = obj.ekf_desv_y;
            
            % Matriz de incerteza do sistema
            obj.ukf_R_t = obj.ukf_desv_x*eye(4);
            
            % Matriz de incerteza da medicao
            obj.ukf_Q_t = obj.ukf_desv_y * eye(3);
            
            %% Inicializacoes do filtro de Particulas
            
            % Matriz de incerteza do sistema
            obj.fp_R_t = obj.fp_desv_x * eye(3);
            
            %% Inicializacoes do filtro de Bayes
            
            % Inicializacao da matriz de covariancias
            obj.fb_K = eye(2) * obj.fb_desvBayes^2;
           
           %-- Cria o mapa de possibilidades onde o robo possa estar
           obj.fb_Sx = [-5:obj.fb_resMapa:5];
           obj.fb_Sy = [-5:obj.fb_resMapa:5];
            
            %% Finalizacoes do construtor
            
            % Enviando uma mensagem para o usuario
            disp('>> Objeto de filtros criado.');
            
        end
        
        %% Filtro de Kalman
        function [m_n, P_n] = kf(obj, m_n1, P_n1, y_n, deltaT)
            
            F_n = [1 0 deltaT 0; 0 1 0 deltaT; 0 0 1 0; 0 0 0 1];
            Q_n = [deltaT^3/3, 0, deltaT^2/2, 0;
                0, deltaT^3/3, 0, deltaT^2/2;
                deltaT^2/2, 0, deltaT, 0;
                0, deltaT^2/2, 0, deltaT];
            
            % Predicao
            m_nn1 = F_n * m_n1;
            P_nn1 = (F_n * P_n1 * F_n.') + Q_n;
            
            % Refinamento
            S_n = (obj.kf_H_n * P_nn1 * obj.kf_H_n.') + obj.kf_R_n;
            K_n = P_nn1 * obj.kf_H_n.' / S_n;
            m_n = m_nn1 + K_n * (y_n - (obj.kf_H_n * m_nn1));
            P_n = P_nn1 - K_n * obj.kf_H_n * P_nn1;
            
        end
        
        % --- Adicionar ruido a leitura ---
        function y_n = kf_adicionarRuidoLeitura(obj, x_n)

            w_n = obj.kf_desv_y * (rand(length(x_n),1) - 0.5);
            y_n = obj.kf_H_n * x_n + w_n;
        end
        
            
        %% Filtro de Kalman Extendido
        function [m_n, P_n] = ekf(obj, m_n1, P_n1, y_n, deltaT)
            
            % Calculando a Jacobiana de F
            Ft_n = [1 0 deltaT 0; 0 1 0 deltaT; 0 0 1 0; 0 0 0 1];
            
            % Calculando a Jacobiana de H
            Ht_n = obj.ekf_jacH(m_n1);
            
            % MAtriz de cov da predicao
            Q_n = [deltaT^3/3, 0, deltaT^2/2, 0;
                0, deltaT^3/3, 0, deltaT^2/2;
                deltaT^2/2, 0, deltaT, 0;
                0, deltaT^2/2, 0, deltaT];          
                       
            % Predicao
            m_nn1 = feval(@obj.ekf_f, m_n1, deltaT);
            P_nn1 = Ft_n * P_n1 * Ft_n.' + Q_n;
           
            % Atualizacao 
            S_n = Ht_n * P_nn1 * Ht_n.' + obj.ekf_R_n;
            K_n = P_nn1 * Ht_n.' / S_n;
            m_n = m_nn1 + K_n * (y_n - feval(@obj.ekf_h,m_nn1));
            P_n = P_nn1 - K_n * Ht_n * P_nn1;
            
        end
        
        % --- Adicionar ruido a leitura ---
        function y_n = ekf_adicionarRuidoLeitura(obj, x_n)
            
            w_n = obj.ekf_desv_y * (rand(3,1)-0.5);
            y_n = feval(@obj.ekf_h, x_n) + w_n;
        end
        
        % ----- Funcao de transicao de estado para o caso do ekf
        function x_n = ekf_f(obj, x_n1, deltT)
            x_n = [x_n1(1) + x_n1(3)*deltT;
                x_n1(2) + x_n1(4)*deltT;
                x_n1(3);
                x_n1(4);
                ];
        end
        
        %-------- Funcao de medicao para o caso do ekf
        function y_n = ekf_h(obj, x_n1)            
            y_n = [  sqrt(x_n1(1)^2 + x_n1(2)^2);
                    (x_n1(1)*x_n1(3) + x_n1(2)*x_n1(4)) / sqrt(x_n1(1)^2 + x_n1(2)^2);
                    %atan(x_n1(2) / x_n1(1));
                    atan2(real(x_n1(2)),real(x_n1(1)));
                ];
        end
        
        %-------- Funcao que cria a jacobiana de H dado um estado m_n
        function H_t = ekf_jacH(obj, m_n)
            
            % Separando o vetor de estados de entrada
            x = m_n(1); y = m_n(2); xd = m_n(3); yd = m_n(4);
            
            % Criando os elementos da jacobiana
            h11 = x / sqrt(x^2 + y^2);
            h12 = y / sqrt(x^2 + y^2);
            h21 = (y * (xd*y - x*yd)) / sqrt((x^2 + y^2)^3);
            h22 = (x * (x*yd - xd*y)) / sqrt((x^2 + y^2)^3);
            h23 = h11;
            h24 = h12;
            h31 = -y / (x^2 + y^2);
            h32 =  x / (x^2 + y^2);
            
            % Montando a jacobiana
            H_t = [h11 h12 0 0; h21 h22 h23 h24; h31 h32 0 0];
            
        end
        
        %% Filtro de kalman Unscented
        function [m_n, P_n] = ukf(obj, m_n1, P_n1, y_n, deltaT)
           
            % Dimensao do sistema
            [n, ~] = size(m_n1);          
            
            % Calculando os sigma points que serao utilizados
            sigmaPoints_n1 = [m_n1, (m_n1 + obj.ukf_lambda*sqrtm(P_n1)), (m_n1 - obj.ukf_lambda *sqrtm(P_n1))];
            
            % Pesos dos Sigma Points
            % Note que o somatorio dos pesos deve ser igual a 1
            w(1) = obj.ukf_lambda / (n + obj.ukf_lambda);
            for i=2:(2*n+1)
               w(i) = 1 / (2*(n + obj.ukf_lambda)); 
            end
            
            % Predicao atraves dos sigma points
            for i=1:(2*n+1)
               sigmaPoints_nHat(1:4,i) = feval(@obj.ekf_f, sigmaPoints_n1(1:4,i),deltaT);
            end
            
            % Calculo da media e covariancia linh
            
            % -- Etapa de Predicao ---
            
            % Calculo da media predita
            m_nn1 = 0;
            for i=0:2*n
               m_nn1 = m_nn1 + (w(i+1) * sigmaPoints_nHat(:,i+1));
            end         
            
            % Calculo da Covariancia predita
            P_nn1 = 0;
            for i=0:2*n
                P_nn1 = P_nn1 + ((w(i+1) * (sigmaPoints_nHat(:,i+1) - m_nn1(:,1)) * (sigmaPoints_nHat(:,i+1) - m_nn1(:,1)).'));
            end
            
            % Soma a incerteza a covariancia
            P_nn1 = P_nn1 + obj.ukf_R_t;
            
            % Atualizando os sigmapoints com a media e matriz de
            % covariancia preditas
            sigmaPoints_n1 = [m_nn1, (m_nn1 + obj.ukf_lambda*sqrtm(P_nn1)), (m_n1 - obj.ukf_lambda*sqrtm(P_nn1))];
            
            % --- Etapa de atualizacao ----
            
            % Calculo dos pontos mensurados no espaço da medição
            for i=(1:2*n+1)
                Z(1:3,i) = feval(@obj.ekf_h, sigmaPoints_n1(1:4,i));
            end
            
            % Calculo da media no espaço de medicao
            z_hat = 0;
            for i=0:2*n
                z_hat = z_hat + (w(i+1) * Z(1:3,i+1));
            end
            
            % Covariancia no estado de medicao
            S = 0;
            for i=0:2*n
               S = S + ( (w(i+1) * (Z(1:3,i+1) - z_hat) * (Z(1:3,i+1) - z_hat).'));               
            end
            
            % Adicionando a incerteza a variancia no espaco de medicao
            S = S + obj.ukf_Q_t;
            
            % - CAlculando o ganho de Kalman
            
            % Correlacao cruzada entre os sigmapoints e o espaco de medicao
            T = 0;
            for i=0:2*n
               T = T + (  w(i+1) * (sigmaPoints_n1(1:4,i+1) - m_nn1) * (Z(1:3,i+1) - z_hat).' ); 
            end
            
            % Ganho de Kalman
            K = T * S^-1;
            
            % Calculando a media e matriz de covariancia do filtro pelo
            % ganho de Kalman
            m_n = m_nn1 + K * (y_n - z_hat);
            P_n = P_nn1 - (K * S * K.');

        end
        
        %% Filtro de particulas
        %
        % metodo_resample - 1- simples ; 2- low variance
        % function [m_n, P_n] = ukf(obj, m_n1, P_n1, y_n, deltaT)
        function xCal_n = fp(obj, xCal_n1, u_n, y_n, deltaT, metodoResample)
           	            
            % Fase de predicao
            for m=1:obj.fp_M
                %feval(@obj.ekf_f, m_n1, deltaT);
                % Atualiza o estado de cada particula baseado no modelo do
                % sistema e a incerteza
                xCal_nHat(1:4,m) = feval(@obj.ekf_f,xCal_n1(1:4,m), deltaT) + sqrt(obj.fp_desv_x)*randn(4,1);               
                
                % Atualiza a medicao para cada particula calculada
                y_nHat(1:3,m) = feval(@obj.ekf_h,xCal_nHat(1:4,m));
                
                % Calcula o peso de cada particula
                % atraves da equacao da gaussiana
                W(:,m) = (1 / sqrt(det(obj.fp_R_t)*(2*pi)^length(y_n(:,1)))) * exp(-0.5 * (y_n' - y_nHat(:,m)') * obj.fp_R_t^-1 * (y_n' - y_nHat(:,m)')');
                
            end
            
            % Normalizando os pesos
            W = W ./ sum(W);
            
            % Faz o resample baseado em um metodo pre-definido
            switch metodoResample               
                % Resample uniforme
                case 1
                    xCal_n = obj.fp_resampleUniforme(xCal_nHat, W);    
                % Resample de baixa variancia
                case 2
                    xCal_n = obj.fp_resampleLowVariance(xCal_nHat, W);
            end

            
        end
        
        % --- Geracao das particulas iniciais
        function xCal = fp_estInicial(obj, x_ini)
            
            % Gerando as particulas iniciais baseado em uma distribuicao
            % gaussiana no entorno da priori inicial
            x_estIni = [];
            for i=1:obj.fp_M
                
                % Gera uma inicializacao para cada dimensao de x_ini
                for i2 = 1:length(x_ini(:,1))
                    xCal(i2,i) = x_ini(i2,1) + sqrt(obj.fp_V) * randn;
                end
            end             
        end
        
        % --- Metodo para calcular a media do FP a partir das particulas
        % Obtencao da previsao do filtro de particulas a partir da
        % informacao das particulas utilizando de uma metrica (no caso,
        % media)
        function m_n = fp_obter_m_n(obj, xCal_n)
            
            % Calcula a media de todos os elementos das particulas
            m_n = mean(xCal_n,2);
            
        end
            
       % --- Metodo de Resample uniforme para o FP
       function xCal_n = fp_resampleUniforme(obj, xCal_nHat, W)
           
           % Faz o resample
           for m=1:obj.fp_M
                xCal_n(:,m) = xCal_nHat(:,find(rand <= cumsum(W),1));
           end
            
       end
       
       % ---- Metodo de Resample low variance para o FP
       % MEtodo extraido do Livro Probabilistic Robots, Sebastian Thrun
       % Pg. 86
       function xCal_n = fp_resampleLowVariance(obj, xCaln_nHAt, W)
           
           % Gerando um numero aleatorio no intervalo [0,M[
           r = (rand(4,1) * obj.fp_M^-1);
           
           % Amostra um dos pesos
           c = W(1);
           
           i=1;
           for m=1:obj.fp_M
               u = r + (m-1) * obj.fp_M^-1;
               
               while u > c
                   i = i+1;
                   c = c + W(i);
               end
               
               % Adicionando a particula ao conjunto final
               xCal_n(1:4,m) = xCaln_nHAt(1:4,i);            
           end 
       end
       
       %% Filtro discreto de Bayes
       % ATENCAO: ATUALMENTE O FB ESTA IMPLEMENTADO PARA FILTRAR APENAS A
       % POSICAO NO PLANO DO ROBO       
       function Pk_n = fb(obj, fb_X_n)
           
           % Inicializacao do mapa
           % Inicializa o mapa com uma uniforme
           L = length(obj.fb_Sx);
           Pk_n1 = ones(L,L);
           
           % Normaliza Pk_n para transformar em uma pdf valida
           Pk_n1 = Pk_n1/sum(sum(Pk_n1));
           
           % Salva Pkn_1 em uma variavel iterativa auxiliar
           Pr = Pk_n1;
           
           for n=2:length(fb_X_n)
                              
               % Inicializando o mapa discreto
               mapa = 0 * Pr;
               
               % Itera o mapa para ajustar as probabilidades
               for i=1:length(Pr)
                   for j=1:length(Pr)
                       
                       % Acessa uma posicao do mapa
                       mapa_ik = [obj.fb_Sx(i); obj.fb_Sy(j)];
                       
                       % Calcula a probabilidade de estar em cada ponto
                       mapa(i,j) = 1/sqrt((2*pi)^2*det(obj.fb_K)) * exp(-(fb_X_n(:,n) - mapa_ik)' * inv(obj.fb_K) * (fb_X_n(:,n) - mapa_ik)/2);
                       
                       % Combina a probabilidade com a priori
                       mapa(i,j) = mapa(i,j) * Pr(i,j);
                   end
               end
               
               % Gera o mapa a posteriori
               Pk_n = mapa / sum(sum(mapa));
               
               % ==== DESCOMENTAR PARA MOSTRAR AS ETAPAS DE BAYES ====
%                pause(0.1);
%                mesh(Pk_n);
               
               % Salva o mapa a posteriori em um Priori auxiliar iterativo
               Pr = Pk_n;            
           end

       end
       
       % -- Metodo que encontra a predicao do filtro Bayes baseado nas
       % probabilidades do mapa discreto
       function m_n = fb_obter_m_n(obj, Pk_n)
           
            % Descobre os indices onde Pk_n atinge seu valor maximo
           [x_best, y_best] = find(Pk_n == max(max(Pk_n)));
           
           % Resgata a melhor estimativa de onde possa estar o robo
           m_n = [obj.fb_Sx(x_best); obj.fb_Sy(y_best)];
           
           % Tira uma media de m_n, caso tenham sido obtidos mais pontos
           %m_n = mean(m_n);
              
       end
       
       %-- Metodo para gerar pontos a partir de uma leitura para o FB
       function fb_xP = fb_gerarPontos(obj, y_n)
                  
           % Criando um vetor de pontos aleatorios uniformes
           fb_nPoints = obj.fb_desvBayes * randn(2,obj.fb_N);
           
           % Inicializando o vetor de priori
           fb_xP = zeros(2, obj.fb_N);
           
           % Centraliza a gaussiana a priori onde a leitura esta
           for i=1:obj.fb_N
               fb_xP(:,i) = y_n + fb_nPoints(:,i);
           end
           
       end
       
       
       
       
       
        %% ======= Funcao para criar a TH a partir dos angulos de Euler ============
        function a = toTH(b)
            
            %
            a = b;
            %Adaptando os angulos de euler para rotacoes em ZYX
            %euler(1) = eul_ang(3);
            %euler(2) = eul_ang(2);
            %euler(3) = eul_ang(1);
            
            %Convertendo a orientacao de quaternion para matriz de rotacao
            %R_mundo_robo = eul2rotm(euler);
            
            % Criando a TH do robo com respeito ao mundo
            %T_mundo_robo = thcreator(R_mundo_robo,p_mundo_robo);
            
        end
    
    % ======= Fim da declaracao dos metodos
    end
    
%% Fim da declaracao das classes
end

