classdef class_vrep_manipulation
    % Classe que implementa diretamente as interacoes com o VREP
    
    %% Propriedades da Classe
    properties
        
        % Id do Cliente do VREP
        clientID = [];
        
        % Manipulador da API do VREP
        vrep = [];
        
        handle_motor = zeros(1,4);
        handle_robo = 0;
    end
    
    %% Metodos da classe
    methods
        
        %% Metodo construtor
        function obj = class_vrep_manipulation(obj)
            
            % --Aviso para o usuario
            disp('--------------------------------------------');
            disp('- Codigo 01 para conexao com o VREP e LIZA -');
            disp('--------------------------------------------');
            
            % Iniciando a conexao com o VREP
            obj.vrep=remApi('remoteApi');  %Usando o arquivo prototipo remoteApi
            obj.vrep.simxFinish(-1);  % Fecha todas as conexoes que possivelmente possam estar abertas
            obj.clientID=obj.vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % Inicia a conexao
            
            %Testa se a conexao foi bem sucedida
            if obj.clientID <= -1
                disp('A conexao com o VREP nao pode ser estabelecida');
                disp('Verifique se a simulacao esta funcionando e o remoteAPI foi inicializado corretamente no modelo');
                disp(' ');
                error('Interrompendo a execucao: V-REP não encontrado.');
                return;
            else
                disp('V-REP: >> Conexao realizada com sucesso!');
            end
            
            % Recebendo os handlers dos necessarios
            for i=1:4
                [resp,obj.handle_motor(i)] = obj.vrep.simxGetObjectHandle(obj.clientID,strcat('motRoda',int2str(i)),obj.vrep.simx_opmode_blocking);
            end
            
            % Recebendo handler do robo
            [resp,obj.handle_robo] = obj.vrep.simxGetObjectHandle(obj.clientID,'liza',obj.vrep.simx_opmode_blocking);
            
            
            % --Iniciando o streaming das variaveis de interesse
            
            % Orientacao do robo
            obj.vrep.simxGetObjectOrientation(obj.clientID,obj.handle_robo,-1,obj.vrep.simx_opmode_streaming);
            
            % Posicao do robo
            obj.vrep.simxGetObjectPosition(obj.clientID,obj.handle_robo,-1,obj.vrep.simx_opmode_streaming);
            
            % Velocidades linear e angular do robo
            obj.vrep.simxGetObjectVelocity(obj.clientID,obj.handle_robo,obj.vrep.simx_opmode_streaming);
            
            % Pausa para garantir inicializacao
            pause(0.5);
            
            % Mensagem para o usuario
            disp('V-REP: Handlers devidamente inicializados');
            
        end
        
        %% Funcao que obtem de uma vez todos os dados necessarios da simulacao
        function [eul_ang, p_mundo_robo, vel, tempo_atual] = obterDadosSim(obj)
            % Orientacao em angulos de euler
            [resp,eul_ang] = obj.vrep.simxGetObjectOrientation(obj.clientID,obj.handle_robo,-1,obj.vrep.simx_opmode_buffer);
            
            % Vetor de translacao
            [resp,pos] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.handle_robo,-1,obj.vrep.simx_opmode_buffer);
            
            %Extraindo a translação do robo
            p_mundo_robo = [pos(1); pos(2); pos(3)];
            
            % Vetor de velocidades do corpo
            [res, vel] = obj.vrep.simxGetObjectVelocity(obj.clientID,obj.handle_robo,obj.vrep.simx_opmode_buffer);
            
            % Recebe o tempo de simulacao do ultimo comando enviado. Divide
            % por 1000 pois esta em ms.
            tempo_atual = obj.vrep.simxGetLastCmdTime(obj.clientID)/1000;
        end
        
        
    end
    
end

