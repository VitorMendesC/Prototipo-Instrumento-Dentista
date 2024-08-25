# Prototipo de um instrumento para dentista
Protótipo de um instrumento para dentistas que consiste em um Motor CC controlado por um micro STM32F103 através de uma ponte H com acionamento via pedal.
No projeto em bancada o pedal é simulado através do uso de um potênciomentro.

## Requisitos
- Operação:
    - Rotacionar suavemenete o potênciometro da posição de repouso (resistência mínima) deve atualizar a velocidade do motor proporcionalmente a posição do potênciometro, permitindo ao operador selecionar uma velocidade desejada.
    - A velocidade selecionada deve ser mantida ao retornar o potênciometro.
    - A velocidade só deve ser alterada caso o potênciometro volte a posição de repouso e seja novamente incrementado, selecionando uma nova velocidade.
    - Acionar o pedal (potênciometro) até o fim de curso duas vezes consecutivas deve inverter o sentido de giro.
    - A velocidade e sentido de giro do motor devem ser exibidas no display, assim como alertas de proteção.
- Segurança:
    - Situações de carga acima do limite (motor travado) devem ser identificas pelo microcontrolador que deve desligar o motor.
    - Quando o pedal for pressionado rapidamente e por curta extensão o motor deve ser desligado.
    - O motor pode ser re-acionado retornando o potênciometro ao repouso.
 
## Planta do motor CC
A Figura 1 exibe a planta do motor CC utilizado. Não foi utilizado a planta como um todo, somente o sistema de acionamento que recebe um sinal PWM do microcontrolador, e o encoder.
Ao lado vemos o controlador seguido pela protoboard com a ponte H, potênciometro e divisor de tensão para leitura do encoder.

<p align="center">
Figura 1 - Planta do motor CC.
</p>

![](img/planta.jpg)

</br></br></br>

## Acionamento - Ponte H
Para controlar o motor CC, como é necessário inverter o sentido de giro, fez-se uso do circuito ponte H, exibido pela Figura 2.

<p align="center">
Figura 2 - Circuito da ponte H.
</p>

![](img/circuito.PNG)

Devido a limitações do microcontrolador, a tensão fornecida não é suficiente para acionar os MOSFETs de potência diretamente, dessa forma fez-se uso de BJTs para acionar os MOSFETS. Uma grande vantagem dessa configuração é a baixa corrente drenada do microcontrolador, devido ao alto ganho de corrente fornecido pelos BJTs.

Os resistores foram dimensionados de forma a garantir um rápido transitório (levando em consideração que os resistores e a porta dos MOSFETs formam um circuito RC) para que os MOSFETs sejam operados como chave e fiquem o menor tempo possivél na zona ôhmica, o que também garante que os mesmos não esquentem de forma exagera durante a operação.

A ponte H implementada em bancada é exibida pela Figura 3.

<p align="center">
Figura 3 - Ponte H implementada em bancada.
</p>

![](img/ponte.jpg)

</br></br></br>

## Display
Atendendo os requisitos do a velocidade, o sentido de giro (H) para sentido Horário e (A) para Anti-horário são exibidos como mostra Figura 4. Como um extra e para propósitos de debugging o ciclo de trabalho do PWM que o microcontrolador fornece ao sistema de acionamento também é exibido.

<p align="center">
Figura 5 - Display em operação nominal.
</p>

![](img/disp1.jpg)

Agora, caso ocorre algum problema na operação o motor é desligado e é emitido um alerta do problema no display, como exibido pela Figura 5.

<p align="center">
Figura 5 - Display quando há um problema identificado.
</p>

![](img/disp2.jpg)

</br></br></br>

## Programa embarcado
Toda a lógica necesária para implementar cada um dos requisitos pode ser vista no código fonte presente na <a href="CubeIDE Workspace/Core/Src/main.c" class="image fit">main</a>.

</br></br></br>

## Video demonstração
Um video que exibe o funcionamento do instrumento, atendendo todos os requisitos, pode ser visto <a href="https://youtu.be/rYY9jYwxjhg" class="image fit">aqui</a>.
