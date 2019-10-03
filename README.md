# VSSS Controller Simulator

Pequeno modelo de simulação e controle de um robô da categoria _Very Small Size Soccer_

## Modelo de movimentação

Os eixos que consideramos a movimentação do robô são dados pela figura:

<p align="center">
  <img src="img/coordenadas.png" width="600">
</p>

As velocidades do robô em cada eixo, relacionadas com as velocidades dos motores _*me*_ e _*md*_
em cm/s (ou m/s) são modeladas por:


<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;\left\{\begin{matrix}&space;\dot{x}&space;=&space;\frac{m_e&space;&plus;&space;m_d}{2}&space;cos\varphi&space;\\&space;\dot{y}&space;=&space;\frac{m_e&space;&plus;&space;m_d}{2}&space;sin\varphi\\&space;\dot{\varphi}&space;=&space;\frac{-m_e&space;&plus;&space;m_d}{l}&space;\end{matrix}\right." target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\left\{\begin{matrix}&space;\dot{x}&space;=&space;\frac{m_e&space;&plus;&space;m_d}{2}&space;cos\varphi&space;\\&space;\dot{y}&space;=&space;\frac{m_e&space;&plus;&space;m_d}{2}&space;sin\varphi\\&space;\dot{\varphi}&space;=&space;\frac{-m_e&space;&plus;&space;m_d}{l}&space;\end{matrix}\right." title="\left\{\begin{matrix} \dot{x} = \frac{m_e + m_d}{2} cos\varphi \\ \dot{y} = \frac{m_e + m_d}{2} sin\varphi\\ \dot{\varphi} = \frac{-m_e + m_d}{l} \end{matrix}\right." /></a>

No entanto, temos dois pontos a serem levados em conta nesta equação. Ela não é linear, isto é, 
o estado das variáveis depende do seno e do cosseno do ângulo com a horizontal. Além disso, no
processamento de imagem do jogo, o que nos é retornado é o vetor de direção û, de módulo unitário.
Sua relação com o ângulo é precisamente:

<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;\left\{\begin{matrix}&space;u_x&space;=&space;cos&space;\varphi&space;\\&space;u_y&space;=&space;sin&space;\varphi&space;\end{matrix}\right." target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\left\{\begin{matrix}&space;u_x&space;=&space;cos&space;\varphi&space;\\&space;u_y&space;=&space;sin&space;\varphi&space;\end{matrix}\right." title="\left\{\begin{matrix} u_x = cos \varphi \\ u_y = sin \varphi \end{matrix}\right." /></a>

de modo com que as velocidades (derivada com o tempo) desses _eixos_ seja associada por:

<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{120}&space;\left\{\begin{matrix}&space;\dot{u_x}&space;=&space;-sin&space;\varphi&space;\cdot&space;\dot{\varphi}&space;=&space;-u_y&space;\&space;\dot{\varphi}&space;\\&space;\dot{u_y}&space;=&space;cos&space;\varphi&space;\cdot&space;\dot{\varphi}&space;=&space;u_x&space;\&space;\dot{\varphi}&space;\end{matrix}\right." target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\left\{\begin{matrix}&space;\dot{u_x}&space;=&space;-sin&space;\varphi&space;\cdot&space;\dot{\varphi}&space;=&space;-u_y&space;\&space;\dot{\varphi}&space;\\&space;\dot{u_y}&space;=&space;cos&space;\varphi&space;\cdot&space;\dot{\varphi}&space;=&space;u_x&space;\&space;\dot{\varphi}&space;\end{matrix}\right." title="\left\{\begin{matrix} \dot{u_x} = -sin \varphi \cdot \dot{\varphi} = -u_y \ \dot{\varphi} \\ \dot{u_y} = cos \varphi \cdot \dot{\varphi} = u_x \ \dot{\varphi} \end{matrix}\right." /></a>

Por fim, associando com o valor de velocidade angular da primeira equação, 
temos a equação de estados que modela o sistema:

<a href="https://www.codecogs.com/eqnedit.php?latex=\dpi{150}&space;\left\{\begin{matrix}&space;\dot{x}&space;=&space;\frac{m_e&plus;m_d}{2}&space;\&space;u_x&space;\\&space;\dot{y}&space;=&space;\frac{m_e&plus;m_d}{2}&space;\&space;u_y&space;\\&space;\dot{u_x}&space;=&space;\frac{m_e-m_d}{l}&space;\&space;u_y&space;\\&space;\dot{u_y}&space;=&space;\frac{-m_e&plus;m_d}{l}&space;\&space;u_x&space;\end{matrix}\right." target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dpi{150}&space;\left\{\begin{matrix}&space;\dot{x}&space;=&space;\frac{m_e&plus;m_d}{2}&space;\&space;u_x&space;\\&space;\dot{y}&space;=&space;\frac{m_e&plus;m_d}{2}&space;\&space;u_y&space;\\&space;\dot{u_x}&space;=&space;\frac{m_e-m_d}{l}&space;\&space;u_y&space;\\&space;\dot{u_y}&space;=&space;\frac{-m_e&plus;m_d}{l}&space;\&space;u_x&space;\end{matrix}\right." title="\left\{\begin{matrix} \dot{x} = \frac{m_e+m_d}{2} \ u_x \\ \dot{y} = \frac{m_e+m_d}{2} \ u_y \\ \dot{u_x} = \frac{m_e-m_d}{l} \ u_y \\ \dot{u_y} = \frac{-m_e+m_d}{l} \ u_x \end{matrix}\right." /></a>

## Modelo de controle

Para controlar os motores _*me*_ e _*md*_, utilizamos um ponto do campo como referência (_setpoint_), que no jogo
pode ser a bola, por exemplo. 

<p align="center">
  <img src="img/controle.png" width="600">
</p>

Consideramos a função erro, que depende da distância do robô até o _setpoint_ e do
ângulo que û faz com a reta (ou vetor) que liga o centro do robô ao ponto de referência. O objetivo do controle é
zerar esse erro.

## Possíveis melhorias

 - Possívelmente adotar no modelo de movimentação a ideia de aceleração. Deste modo, poderíamos controlar a velocidade com que
o robô chega ao _setpoint_
 - Controlar, além da posição do _setpoint_, o ângulo com que chega ao objetivo.