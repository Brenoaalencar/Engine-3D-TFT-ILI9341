# Engine-3D-TFT-ILI9341
Engine gráfica para exibição de objetos 3D em um display ILI9341 2.4" usando Nucleo F401RE

Agora que já vimos técnicas para animações em 2D, adicionaremos a terceira dimensão em nossas animações. 

<div align="center">
<img src ="https://github.com/Brenoaalencar/DAC_R2R_8bits/assets/72100554/0d7df9f6-69b6-436a-a22a-1febbf2ff41c.gif" width="400px"/>
</div>

A chave para entendermos como transformarmos elementos tridimensionais em bidimensionais está na projeção de perspectiva, pois, na ilusão criada para comportar a terceira dimensão, tudo deve ser projetado para o plano do anteparo visual, a tela, de acordo com a perspectiva do observador.

Dessa forma, criaremos vértices no espaço que conterão a forma daquilo que queremos representar e utilizaremos transformações para movermos, rotacionarmos e projetarmos esses vértices que serão interligados por uma malha de triângulos. Em nosso exemplo, criaremos um cubo, mas, com algumas modificações, uma grande variedade de formas pode ser implementada para exibição.

A seguir, descrição detalhada do código para realizar a animação.

```C++

// Bibliotecas
#include "Arduino.h"
#include "mbed.h"
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include <cmath>
#include <vector>
```

Iniciamos as bibliotecas necessárias e criamos uma instância (objeto) da classe 'MCUFRIEND_kbv' com o nome 'tft'. As duas últimas inclusões são bibliotecas padrão de C++ para operações matemáticas e uso de vetores.

Depois definimos a tabela de cores do display no formato RGB565.

```C++
// Cores do display
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define PURPLE 0x780F /* 128,   0, 128 */
#define ORANGE 0xFDA0 /* 255, 180,   0 */
```
A seguir, três estruturas são criadas:

```C++

struct vec3d {
  float x, y, z;
};

struct triangle {
  vec3d p[3];
};

struct mesh {
  vector<triangle> tris;
};
```

A primeira 'vec3d' representa um ponto no espaço tridimensional, composto por coordenadas 'x', 'y' e 'z'. A segunda 'triangle' contém um array (vetor) chamado 'p' que pode armazenar três elementos do tipo 'vec3d'. Ou seja, 'triangle' está armazenando três pontos tridimensionais que, quando conectados, formam um triângulo no espaço 3D. Cada elemento do array 'p' representa um vértice desse triângulo.
A terceira 'mesh' contém uma variável 'tris' que é um vetor de elementos do tipo triângulo. O tipo 'vector' é um contêiner dinâmico em C++, que pode crescer ou diminuir durante a execução do programa. Ou seja, cada elementos do vetor 'tris' é um triângulo definido pela estrutura anterior. Sendo assim, 'mesh' é uma coleção de triângulos que, quando combinados, formam um modelo 3D mais complexo.

A seguir, criaremos uma estrutura de matriz 4x4 para realizarmos as transformações homogêneas. 

```C++
struct mat4x4 {
  float m[4][4];

  mat4x4() {
    // Inicialização no construtor
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        m[i][j] = 0.0f;
      }
    }
  }
};
```

Criaremos a seguir a classe Engine 3D

```C++
class Engine3D {
private:
  mesh meshCube;
  mat4x4 matProj;
  float fTheta;

  void MultiplyMatrixxVector(const vec3d &i, vec3d &o,const mat4x4 &m) {
    // Adicionaremos a função para multiplicação de matriz por vetor
  }

public:
  bool OnUserCreate() {
    // Método para inicialização da classe com parâmetros fixos
  }

  bool OnUserUpdate(float fElapsedTime) {
    // Método para atualização do jogo de acordo com o ElapsedTime
  }
};
```

Em ```private``` criamos a malha do cubo ```meshCube```, a matriz de projeção ```matProj```, o ângulo para rotação ```fTheta``` e implementamos a função de multiplicação de matriz por vetor ```MultiplyMatrixxVector```.

```C++
void MultiplyMatrixxVector(const vec3d &i, vec3d &o,const mat4x4 &m) {
    o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
    o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
    o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
    float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

    if (w != 0.0f) {
      o.x /= w;
      o.y /= w;
      o.z /= w;
    }
  }
```

  A função acima recebe o vetor de entrada, o vetor de saída e a matriz que será multiplicada pelo vetor de entrada. O resultado de cada elemento do vetor de saída vem do produto matricial e cria-se um termo 'w' para que o produto possa ser feito, pois a matriz é 4x4 e os vetores possuem três elementos. Se esse novo termo calculado for diferente de zero, divide-se os elementos de saída por ele.

Seguindo para o método ```OnUserCreate```, criaremos os triângulos do cubo seguindo uma lógica para os pontos.

<div align="center">
<img src ="https://github.com/Brenoaalencar/DAC_R2R_8bits/assets/72100554/e5c051b1-cd89-4983-8516-0cc1ac2693e8.png" width="450px"/>
</div>

  Como possuímos dois triângulos por face, precisaremos criar doze triângulos e adicioná-los no vetor tris da malha meshCube criada, sempre em ordem horária. Então criaremos triângulos auxiliares e o inseriremos na malha por meio do ```push_back()```. Existem formas mais otimizadas de inserir no vetor esses pontos, porém o compilador do Keil Studio não permitiu adicionar sem criar esses triângulos auxiliares. 

```C++
  // South
    triangle tri1 = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f};
    meshCube.tris.push_back(tri1);

    triangle tri2 = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    meshCube.tris.push_back(tri2);

    // East
    triangle tri3 = {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f};
    meshCube.tris.push_back(tri3);

    triangle tri4 = {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f};
    meshCube.tris.push_back(tri4);

    // North
    triangle tri5 = {1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f};
    meshCube.tris.push_back(tri5);

    triangle tri6 = {1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f};
    meshCube.tris.push_back(tri6);

    // West
    triangle tri7 = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f};
    meshCube.tris.push_back(tri7);

    triangle tri8 = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    meshCube.tris.push_back(tri8);

    // Top
    triangle tri9 = {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    meshCube.tris.push_back(tri9);

    triangle tri10 = {0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f};
    meshCube.tris.push_back(tri10);

    // Bottom
    triangle tri11 = {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    meshCube.tris.push_back(tri11);

    triangle tri12 = {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    meshCube.tris.push_back(tri12);

```  

Para finalizarmos o método, precisaremos criar as variáveis e a matriz responsável pela projeção.

```C++
// Projection Matrix
    float fNear = 0.01f;
    float fFar = 200.f;
    float fFov = 0.8f;
    fTheta = 0.0f; // ou outro valor inicial desejado
    float fAspectRatio = 320.0f / 240.0f;
    float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);

    matProj.m[0][0] = fAspectRatio * fFovRad;
    matProj.m[1][1] = fFovRad;
    matProj.m[2][2] = fFar / (fFar - fNear);
    matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    matProj.m[2][3] = 1.0f;
    matProj.m[3][3] = 0.0f;

```

Dessa forma, o método completo fica da seguinte forma:

```C++
bool OnUserCreate() {
    // South
    triangle tri1 = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f};
    meshCube.tris.push_back(tri1);

    triangle tri2 = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    meshCube.tris.push_back(tri2);

    // East
    triangle tri3 = {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f};
    meshCube.tris.push_back(tri3);

    triangle tri4 = {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f};
    meshCube.tris.push_back(tri4);

    // North
    triangle tri5 = {1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f};
    meshCube.tris.push_back(tri5);

    triangle tri6 = {1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f};
    meshCube.tris.push_back(tri6);

    // West
    triangle tri7 = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f};
    meshCube.tris.push_back(tri7);

    triangle tri8 = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    meshCube.tris.push_back(tri8);

    // Top
    triangle tri9 = {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    meshCube.tris.push_back(tri9);

    triangle tri10 = {0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f};
    meshCube.tris.push_back(tri10);

    // Bottom
    triangle tri11 = {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    meshCube.tris.push_back(tri11);

    triangle tri12 = {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    meshCube.tris.push_back(tri12);

    // Projection Matrix
    float fNear = 0.01f;
    float fFar = 200.f;
    float fFov = 0.8f;
    fTheta = 0.0f; // ou outro valor inicial desejado
    float fAspectRatio = 320.0f / 240.0f;
    float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);

    matProj.m[0][0] = fAspectRatio * fFovRad;
    matProj.m[1][1] = fFovRad;
    matProj.m[2][2] = fFar / (fFar - fNear);
    matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    matProj.m[2][3] = 1.0f;
    matProj.m[3][3] = 0.0f;

    return true;
}

```

Agora que possuímos a malha criada e a matriz de projeção definida, precisamos do método que atualiza a posição dos vértices conforme a rotação, translação e projeção e desenha os triângulos ao fim do processo.
Em nosso exemplo, utilizaremos rotações em X e em Z, então criamos as matrizes de rotação no espaço e o ângulo theta de rotação.
A matriz de rotação é responsável por girar os pontos em torno de algum eixo estabelecido. Sem translações do objeto, podemos realizar o seguinte produto matricial. Se houver translação, os pontos deverão passar por uma transformação homogênea que realize a essa translação.

```C++
// Set up rotation matrices
    mat4x4 matRotZ, matRotX;
    fTheta += 1.0f * fElapsedTime;

    // Rotation Z
    matRotZ.m[0][0] = cosf(fTheta);
    matRotZ.m[0][1] = sinf(fTheta);
    matRotZ.m[1][0] = -sinf(fTheta);
    matRotZ.m[1][1] = cosf(fTheta);
    matRotZ.m[2][2] = 1;
    matRotZ.m[3][3] = 1;

    // Rotation X
    matRotX.m[0][0] = 1;
    matRotX.m[1][1] = cosf(fTheta * 0.5f);
    matRotX.m[1][2] = sinf(fTheta * 0.5f);
    matRotX.m[2][1] = -sinf(fTheta * 0.5f);
    matRotX.m[2][2] = cosf(fTheta * 0.5f);
    matRotX.m[3][3] = 1;

```

Em seguida temos o loop for para desenho dos triângulos da malha do cubo. Em cada iteração do loop, acessa-se um triângulo no vetor ```meshCube.tris``` por meio de um iterador usando uma referência constante ```tri``` para trabalhar com esse triângulo dentro no loop.
Em seguida são criados elementos do tipo ```triangle``` para projeção, translação e rotações. Logo em seguida são aplicadas as rotações em Z e X, o offset para a tela e uma projeção dos triângulos do 3D para o 2D. 
Por fim, os triângulos são escalados e desenhados na tela.
Para reduzir as 'piscadas' (flickering) da tela, repetimos o mesmo for, mas agora instruímos que os triângulos sejam pintados (após um pequeno delay) com a cor do fundo, dando a ilusão de continuidade na animação, assim como nas animações em 2D que fizemos.
Para melhores resultados, busque implementar a técnica de double buffering. 

```C++
// Draw Triangles
   for (std::vector<triangle>::const_iterator it = meshCube.tris.begin(); it != meshCube.tris.end(); ++it) {
    const triangle& tri = *it;
    triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

    // Rotate in Z-Axis
    MultiplyMatrixxVector(tri.p[0], triRotatedZ.p[0], matRotZ);
    MultiplyMatrixxVector(tri.p[1], triRotatedZ.p[1], matRotZ);
    MultiplyMatrixxVector(tri.p[2], triRotatedZ.p[2], matRotZ);

    // Rotate in X-Axis
    MultiplyMatrixxVector(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
    MultiplyMatrixxVector(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
    MultiplyMatrixxVector(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

    // Offset into the screen
    triTranslated = triRotatedZX;
    triTranslated.p[0].z = triRotatedZX.p[0].z + 3.0f;
    triTranslated.p[1].z = triRotatedZX.p[1].z + 3.0f;
    triTranslated.p[2].z = triRotatedZX.p[2].z + 3.0f;

    // Project triangles from 3D --> 2D
    MultiplyMatrixxVector(triTranslated.p[0], triProjected.p[0], matProj);
    MultiplyMatrixxVector(triTranslated.p[1], triProjected.p[1], matProj);
    MultiplyMatrixxVector(triTranslated.p[2], triProjected.p[2], matProj);

    // Scale into view
    triProjected.p[0].x += 1.0f;
    triProjected.p[0].y += 1.0f;
    triProjected.p[1].x += 1.0f;
    triProjected.p[1].y += 1.0f;
    triProjected.p[2].x += 1.0f;
    triProjected.p[2].y += 1.0f;

    triProjected.p[0].x += 0.5f * 320.0f;
    triProjected.p[0].y += 0.5f * 240.0f;
    triProjected.p[1].x += 0.5f * 320.0f;
    triProjected.p[1].y += 0.5f * 240.0f;
    triProjected.p[2].x += 0.5f * 320.0f;
    triProjected.p[2].y += 0.5f * 240.0f;


    tft.drawTriangle(triProjected.p[0].x, triProjected.p[0].y,
                     triProjected.p[1].x, triProjected.p[1].y,
                     triProjected.p[2].x, triProjected.p[2].y, PURPLE);
    
  }

```
  
Por fim, o método completo fica dessa forma:

```C++
  bool OnUserUpdate(float fElapsedTime) {
    
    // Set up rotation matrices
    mat4x4 matRotZ, matRotX;
    fTheta += 1.0f * fElapsedTime;

    // Rotation Z
    matRotZ.m[0][0] = cosf(fTheta);
    matRotZ.m[0][1] = sinf(fTheta);
    matRotZ.m[1][0] = -sinf(fTheta);
    matRotZ.m[1][1] = cosf(fTheta);
    matRotZ.m[2][2] = 1;
    matRotZ.m[3][3] = 1;

    // Rotation X
    matRotX.m[0][0] = 1;
    matRotX.m[1][1] = cosf(fTheta * 0.5f);
    matRotX.m[1][2] = sinf(fTheta * 0.5f);
    matRotX.m[2][1] = -sinf(fTheta * 0.5f);
    matRotX.m[2][2] = cosf(fTheta * 0.5f);
    matRotX.m[3][3] = 1;

    // Draw Triangles
   for (std::vector<triangle>::const_iterator it = meshCube.tris.begin(); it != meshCube.tris.end(); ++it) {
    const triangle& tri = *it;
    triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

    // Rotate in Z-Axis
    MultiplyMatrixxVector(tri.p[0], triRotatedZ.p[0], matRotZ);
    MultiplyMatrixxVector(tri.p[1], triRotatedZ.p[1], matRotZ);
    MultiplyMatrixxVector(tri.p[2], triRotatedZ.p[2], matRotZ);

    // Rotate in X-Axis
    MultiplyMatrixxVector(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
    MultiplyMatrixxVector(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
    MultiplyMatrixxVector(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

    // Offset into the screen
    triTranslated = triRotatedZX;
    triTranslated.p[0].z = triRotatedZX.p[0].z + 3.0f;
    triTranslated.p[1].z = triRotatedZX.p[1].z + 3.0f;
    triTranslated.p[2].z = triRotatedZX.p[2].z + 3.0f;

    // Project triangles from 3D --> 2D
    MultiplyMatrixxVector(triTranslated.p[0], triProjected.p[0], matProj);
    MultiplyMatrixxVector(triTranslated.p[1], triProjected.p[1], matProj);
    MultiplyMatrixxVector(triTranslated.p[2], triProjected.p[2], matProj);

    // Scale into view
    triProjected.p[0].x += 1.0f;
    triProjected.p[0].y += 1.0f;
    triProjected.p[1].x += 1.0f;
    triProjected.p[1].y += 1.0f;
    triProjected.p[2].x += 1.0f;
    triProjected.p[2].y += 1.0f;

    triProjected.p[0].x += 0.5f * 320.0f;
    triProjected.p[0].y += 0.5f * 240.0f;
    triProjected.p[1].x += 0.5f * 320.0f;
    triProjected.p[1].y += 0.5f * 240.0f;
    triProjected.p[2].x += 0.5f * 320.0f;
    triProjected.p[2].y += 0.5f * 240.0f;


    tft.drawTriangle(triProjected.p[0].x, triProjected.p[0].y,
                     triProjected.p[1].x, triProjected.p[1].y,
                     triProjected.p[2].x, triProjected.p[2].y, PURPLE);
    
  }
//delay(8);
for (std::vector<triangle>::const_iterator it = meshCube.tris.begin(); it != meshCube.tris.end(); ++it) {
    const triangle& tri = *it;
    triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

    // Rotate in Z-Axis
    MultiplyMatrixxVector(tri.p[0], triRotatedZ.p[0], matRotZ);
    MultiplyMatrixxVector(tri.p[1], triRotatedZ.p[1], matRotZ);
    MultiplyMatrixxVector(tri.p[2], triRotatedZ.p[2], matRotZ);

    // Rotate in X-Axis
    MultiplyMatrixxVector(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
    MultiplyMatrixxVector(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
    MultiplyMatrixxVector(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

    // Offset into the screen
    triTranslated = triRotatedZX;
    triTranslated.p[0].z = triRotatedZX.p[0].z + 3.0f;
    triTranslated.p[1].z = triRotatedZX.p[1].z + 3.0f;
    triTranslated.p[2].z = triRotatedZX.p[2].z + 3.0f;

    // Project triangles from 3D --> 2D
    MultiplyMatrixxVector(triTranslated.p[0], triProjected.p[0], matProj);
    MultiplyMatrixxVector(triTranslated.p[1], triProjected.p[1], matProj);
    MultiplyMatrixxVector(triTranslated.p[2], triProjected.p[2], matProj);

    // Scale into view
    triProjected.p[0].x += 1.0f;
    triProjected.p[0].y += 1.0f;
    triProjected.p[1].x += 1.0f;
    triProjected.p[1].y += 1.0f;
    triProjected.p[2].x += 1.0f;
    triProjected.p[2].y += 1.0f;

    triProjected.p[0].x += 0.5f * 320.0f;
    triProjected.p[0].y += 0.5f * 240.0f;
    triProjected.p[1].x += 0.5f * 320.0f;
    triProjected.p[1].y += 0.5f * 240.0f;
    triProjected.p[2].x += 0.5f * 320.0f;
    triProjected.p[2].y += 0.5f * 240.0f;


    tft.drawTriangle(triProjected.p[0].x, triProjected.p[0].y,
                     triProjected.p[1].x, triProjected.p[1].y,
                     triProjected.p[2].x, triProjected.p[2].y, BLACK);
    
  }
    return true;
  }

```

Criamos um objeto 'game' para a classe que acabamos de finalizar e a iniciamos no setup. No loop deixamos para atualizar o valor de theta e do tempo transcorrido. 

```C++
// Orientação do Display

uint8_t Orientation = 1; // Paisagem

// Engine

Engine3D game;

void setup(void) {
  tft.reset();
  tft.begin();
  tft.setRotation(Orientation);
  tft.fillScreen(BLACK); // Fundo do Display
  game.OnUserCreate();
}

void loop() {
  // Calcula o tempo decorrido, se necessário
  float fElapsedTime = 0.016f; // Exemplo, você precisa calcular o tempo real para melhores resultados
fElapsedTime += 0.1;
  // Atualiza o jogo
  game.OnUserUpdate(fElapsedTime);

  // Aguarda um curto período (opcional)
  wait_ms(2); // ou use delay(), dependendo do contexto e bibliotecas
}

```

Esse programa é a base para aplicações mais sofisticadas de uma engine gráfica simples. Essa malha cúbica pode ser expandida para uma extensa variedade de formas. Com ajuda de softwares de modelagem 3D, como Blender, você pode criar ou importar os pontos, com algumas modificações nesse código, de modelos tridimensionais mais complexos. Ele também pode ser enriquecido com iluminação, adição de câmera e texturas e muito mais.  

Referências:
Sobre as transformações homogêneas:
https://slideplayer.com.br/slide/358122/

Extremamente recomendado conferir:
https://www.youtube.com/watch?v=ih20l3pJoeU&list=PLrOv9FMX8xJE8NgepZR1etrsU63fDDGxO&index=25&ab_channel=javidx

Slides de suporte:
https://slideplayer.com.br/amp/358089/








