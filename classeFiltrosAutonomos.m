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
        
        % Funcao de transicao de estado para o caso do ekf
        function x_n = ekf_f(obj, x_n1, deltT)
            x_n = [x_n1(1) + x_n1(3)*deltT;
                x_n1(2) + x_n1(4)*deltT;
                x_n1(3);
                x_n1(4);
                ];
        end
        
        % Funcao de medicao para o caso do ekf
        function y_n = ekf_h(obj, x_n1)            
            y_n = [  sqrt(x_n1(1)^2 + x_n1(2)^2);
                    (x_n1(1)*x_n1(3) + x_n1(2)*x_n1(4)) / sqrt(x_n1(1)^2 + x_n1(2)^2);
                    %atan(x_n1(2) / x_n1(1));
                    atan2(real(x_n1(2)),real(x_n1(1)));
                ];
        end
        
        % Funcao que cria a jacobiana de H dado um estado m_n
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
            aux = 0;
            for i=0:2*n
               aux = aux + (w(i+1) * sigmaPoints_nHat(:,i+1));
            end         
            m_nn1 = aux;
            
            % Calculo da Covariancia predita
            aux = 0;
            for i=0:2*n
                aux = aux + ((w(i+1) * (sigmaPoints_nHat(:,i+1) - m_nn1(:,1)) * (sigmaPoints_nHat(:,i+1) - m_nn1(:,1)).'));
            end
            P_nn1 = aux + obj.ukf_R_t;
            
            % Atualizando os sigmapoints com a media e matriz de
            % covariancia preditas
            %sigmaPoints_n1 = [m_nn1, (m_nn1 + obj.ukf_lambda*sqrt(abs(P_nn1))), (m_n1 - obj.ukf_lambda*sqrt(abs(P_nn1)))];
            
            % --- Etapa de atualizacao ----
            
            % Calculo dos pontos mensurados no espaço da medição
            for i=(1:2*n+1)
                Z(1:3,i) = feval(@obj.ekf_h, sigmaPoints_n1(1:4,i));
            end
            
            % Calculo da media no espaço de medicao
            aux = 0;
            for i=0:2*n
                aux = aux + (w(i+1) * Z(1:3,i+1));
            end
            z_hat = aux;
            
            % Covariancia no estado de medicao
            aux = 0;
            for i=0:2*n
               aux = aux + ( (w(i+1) * (Z(1:3,i+1) - z_hat) * (Z(1:3,i+1) - z_hat).'));               
            end
            S = aux + obj.ukf_Q_t;
            
            % --- CAlculando o ganho de Kalman
            
            % Correlacao cruzada entre os sigmapoints e o espaco de medicao
            aux = 0;
            for i=0:2*n
               aux = aux + (  w(i+1) * (sigmaPoints_n1(1:4,i+1) - m_nn1) * (Z(1:3,i+1) - z_hat).' ) ; 
            end
            T = aux;
            
            % Ganho de Kalman
            K = T * S^-1;
            
            % Calculando a media e matriz de covariancia calculadas
            m_n = m_nn1 + K * (y_n - z_hat);
            P_n = P_nn1 - (K * S * K.');

        end
        
        %% Filtro de particulas
        function a = particulas(obj, b)
            b;
        end
        
        %% Funcao para criar a TH a partir dos angulos de Euler
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

