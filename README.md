# arcaboucoTrabalhosAutonomosCOPPE

Repositório contendo o arcabouço para desenvolvimento dos trabalhos da disciplina Tópicos Especiais em Sistemas Autônomos.

(COE841 - COPPE - UFRJ). Professor Ramon Romankevicius

autor: Filipe Rocha

## Descrição dos arquivos contidos neste repositório

- **cenaSimVREP_autonomos.ttt** - Cena de simulação a ser aberta no V-REP.
- **classeFiltrosAutonomos.m**  - Classe que define os filtros e parâmetros. A implementaço dos códigos dos filtros deve ser adicionada aqui.
- **classeManipulacaoVREP.m**   - Classe que realiza a comunicação específica das necessidades deste arcabouço com o V-REP. Disponibiliza métodos de interaço com o simulador.
- **remApi.so, remApi.dll e remoteApi.dylib**  - Bibliotecas da API V-REP<->MATLAB para os sistemas Linux, Windows e MAC, respectivamente.
- **remApi.m** e remoteApiProto.m - implementação da API externa do V-REP no MATLAB.

## Utilização Rápida

1 - Inicie o simulador, abra o arquivo **cenaSimVREP_autonomos.ttt** e inicie a simulação clicando no ícone de *Play*.
  - Para pilotar o robô, clique sobre a tela de simulação e utilize os direcionais para enviar comandos. A tecla **num 0** freia o robô.
  
2 - Abra o arquivo **main.m** no *Matlab* e rode este algoritmo.

## Disclaimer

Este pacote está em atualização, portanto tome cuidado ao realizar **pull** no repositório e ter o seu desenvolvimento perdido. Dê um **fork** neste repositório para fazer seu próprio desenvolvimento e, de preferência, trabalhe em **branches** secundárias. Além disto, **pull requests** neste repositório sero muito bem vindas ;)

## Links Úteis

### Git

Caso não esteja familiarizado com a utilizaço do Git, você pode encontrar informações sobre seus princípios [aqui](https://git-scm.com/book/pt-br/v1/Primeiros-passos-No%C3%A7%C3%B5es-B%C3%A1sicas-de-Git). Além disto, um guia rápido dos comandos mais utilizados pode ser encontrado [aqui](http://rogerdudler.github.io/git-guide/index.pt_BR.html).

### Simulador V-REP

O simulador V-REP é um software comercial, porém disponibiliza uma versão educacional gratuita que pode ser encontrada [aqui](http://www.coppeliarobotics.com/downloads.html).

Já adicionei as bibliotecas para comunicação do V-REP com o MATLAB com os principais sistemas operacionais. Se ainda sim tiver tendo algum problema, dê uma olhada [nestas instruções](http://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm).

Caso queira se aventurar e fazer mais interações com o V-REP a partir do MATLAB, você pode encontrar a lista de funções disponíveis [nesta página](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm). Esta interação deve ser codificada na classe **classeManipulacaoVREP.m**.

## Contato

Caso tenha alguma dúvida, pode entrar em contato no f.rocha41@gmail.com

Espero que este arcabouço contribua no aprendizado dos conceitos da disciplina. :)


