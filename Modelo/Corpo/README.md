## Experimento das Corpo

#### Experimento
Para realizar o experimento do corpo, no código da pasta [ExperimentoArduinoIDE](https://github.com/JessicaLuana1377/diciclo_autonomo/tree/main/Modelo/ExperimentoArduinoIDE) descomente as linhas referentes ao f0, f1 e MAX_PWM do experimento do corpo e carregue o código no ESP32. Clique no botão ‘En’ do ESP32 e, quando uma luz azul piscar, clique no outro botão (‘Boot’). O experimento deve ser realizado segurando as rodas do diciclo e deixando-o pendurado, livre para que o corpo possa balançar. Mantenha-o estável durante todo o experimento e sem o cabo para não prejudicar a captura de dados. Se, em algum momento, o corpo do diciclo bater em algum lugar, diminua a variável MAX\_PWM e recomece o experimento. Espere o experimento terminar. Com o ESP32 conectado no computador, abra a ferramenta ‘CoolTerm.exe’ disponível na internet e na pasta [Programas auxiliares](https://github.com/JessicaLuana1377/diciclo_autonomo/tree/main/Progamas%20auxiliares) (não é necessário instalar o programa, apenas descompactar o arquivo e executar a ferramenta). 

#### Configuração do CoolTerm
Em ‘Options’, no campo ‘Baudrate’, selecione a velocidade da serial configurada no ESP32; caso não tenha alterado, é 115200. Para salvar os dados em um arquivo, vá em ‘Connection’, ‘Capture to text/binary file’, ‘Start’, escolha o diretório do arquivo sendo a pasta ‘Rodas’ e coloque o nome ‘exp_rodas_bruto’. Para conectar a ferramenta com o ESP32, confira se o Serial Monitor de todas as abas do Arduino IDE está fechado, clique em ‘Connect’ e espere o ESP32 parar de piscar a luz azul.

#### Leitura dos dados
Ao clicar em ‘l’, serão listados todos os arquivos presentes na flash do ESP32. A cada nova simulação, é criado um novo arquivo, começando com o nome ‘Arquivo0’, ‘Arquivo1’, etc. A letra ‘f’ apaga todos os arquivos. A letra ‘r’ lê e, em seguida, apaga todos os arquivos e ‘R’ apenas lê, mantendo todos na memória.
Realize a leitura com ‘r’ ou ‘R’ e, quando terminar a leitura, pare a gravação do arquivo em ‘Connection’, ‘Capture to text/binary file’, ‘Stop’. Vá no arquivo ‘exp_rodas_bruto.txt’ e apague as linhas desnecessárias, deixando apenas o cabeçalho e os dados (vá ao final do arquivo e apague todas as linhas em branco).

#### Tratamento e escolha de dados
No arquivo ‘TratarDados.mlx’ descomente as linhas referentes ao experimento do corpo presentes no início e no final do arquivo e o execute. Nele estão dispostos gráficos para verificação dos dados, alguns pontos devem ser conferidos:
    • A frequência inicial deve ser pequena o suficiente para que o início do gráfico de velocidade apresente um ganho constante;
    • A frequência final não deve ser muito alta para não capturar muito ruído;
    • O tempo da chirp deve ser grande o suficiente para dar tempo do motor reagir à entrada;
    • Repita os experimentos com novos valores para as variáveis, se necessário.
Ao final do arquivo "TratarDados.mlx" é gerado um arquivo de dados com extensão do MATLAB que será usado em "ExperimentoCorpo.mlx". Execute-o e serão apresentados 6 modelos: dois baseados na posição fornecida pelo encoder de cada roda, dois baseados na velocidade de cada roda, 1 baseado na posição da MPU e 1 baseado na velocidade da MPU. Escolha o que melhor corresponde ao experimento e substitua no final. Execute o arquivo novamente e serão salvos os dados do modelo correto para serem usados no modelo do pêndulo.