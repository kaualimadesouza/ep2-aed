/*********************************************************************/
/**   ACH2024 - Algoritmos e Estruturas de Dados II                 **/
/**   EACH-USP - Primeiro Semestre de 2025                          **/
/**   Prof. Luciano Digiampietri                                    **/
/**                                                                 **/
/**   Segundo Exercicio-Programa                                    **/
/**                                                                 **/
/**   <Kauã Lima de Souza>                   <15674702>             **/
/**                                                                 **/
/*********************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define true 1
#define false 0

#define INFINITO 999999

typedef int bool;

/* Estrutura para representar nosso grafo usando matriz binaria de adjacencias */
typedef struct {
    int numVertices;
    int numArestas;
    bool** matriz;
} Grafo;


/* Funcao auxiliar para o sistema de correcao automatica (nao mexer) */
void printf123(){
    // Funcao usada pelo sistema de correcao automatica (nao mexer)
}



/* Estrutura para representar elementos de uma lista ligada de vertices */
typedef struct aux{
  int vertice;
  struct aux* prox;
} ElementoLista, *PONT;


/* Estrutura para representar elementos de uma lista ligada com arestas
   (pares de vertices)*/
typedef struct aux2{
  int origem;
  int destino;
  struct aux2* prox;
} ArestaLista, * PONT2;


/* Funcao que inicializa o grafo cujo endereco foi passado como parametro.
   Isto e, cria a matriz de adjacencia (preenchida com ARESTAS_INVALIDAS),
   e atribui valor para numVertices (recebendo o valor passado por parametro)
   e numArestas igual a zero.
*/
bool inicializaGrafo(Grafo* g, int vertices){
  if (g==NULL || vertices<1) return false;
  g->numVertices = vertices;
  g->numArestas = 0;
  int x, y;
  g->matriz = (bool**) malloc(sizeof(bool*)*vertices);
  for (x=0; x<vertices; x++){
    g->matriz[x] = (bool*) malloc(sizeof(bool)*vertices);
    for (y=0; y<vertices; y++){
      g->matriz[x][y] = false;
    }
  }
  return true;
}


/* Funcao que libera a memoria da matriz de adjacencia do grafo cujo endereco
   foi passado como parametro.
*/
bool liberaGrafo(Grafo* g){
  if (g==NULL) return false;
  int x;
  for (x=0; x<g->numVertices; x++)
    free(g->matriz[x]);
  free(g->matriz);
  g->numVertices = 0;
  g->numArestas = 0;
  g->matriz = NULL;
  return true;
}

/* Funcao que insere uma nova aresta no grafo.
   Se os vertices passados como parametro forem validos, ha duas possibilidades:
   - a aresta nao existia;
   - a aresta existia. 
*/
bool insereAresta(Grafo* g, int v1, int v2){
  if (!g || v1 < 0 || v2 < 0 || v1 >= g->numVertices || v2 >= g->numVertices) return false;
  if (g->matriz[v1][v2] == false){
      g->matriz[v1][v2] = true;
      g->numArestas++;
  }
  return true;
}


/* Funcao que exclui a aresta entre v1 e v2 (caso exista uma aresta valida
   entre v1 e v2) e retorna true; ou retorna false, caso contrario.
*/
bool removeAresta(Grafo* g, int v1, int v2){
  if (!g || v1 < 0 || v2 < 0 || v1 >= g->numVertices || v2 >= g->numVertices || g->matriz[v1][v2] == false) return false;
  g->matriz[v1][v2] = false;
  g->numArestas--;
  return true;
}


/* Funcao que cria um grafo com o numero de vertices e
   numero de arestas passados como parametro e retorna seu endereco.
*/
Grafo* criaGrafoAleatorio(int numVertices, int numArestas){
  int x, y, a, total;
  if (numVertices < 1 || numArestas >= numVertices*numVertices/2) {
    printf("Parametros invalidos, o grafo nao sera gerado.\n");
    return NULL;  
  }
  Grafo* g = (Grafo*)malloc(sizeof(Grafo));
  inicializaGrafo(g,numVertices);

  a = 0;
  while(a<numArestas){
    x = rand()%numVertices;
    y = rand()%numVertices;
    if (g->matriz[x][y] == false){
      g->matriz[x][y] = true;
      a++;
    }
  }
  g->numArestas = numArestas;
  return g;
}



/* Funcao desenvolvida para exibir um grafo, no formato de uma matriz
   de adjacencias.
*/
void exibeGrafo(Grafo* g){
  if(!g) return;
  int x, y;
  printf("\nImprimindo grafo (v=%i; a=%i)\n   ", g->numVertices, g->numArestas);
  for (x=0;x<g->numVertices;x++) printf(" %3i",x);
  printf("\n");
  for (x=0;x<g->numVertices;x++){
    printf("%3i",x);
    for (y=0;y<g->numVertices;y++)
      printf(" %3i",g->matriz[x][y]);  
    printf("\n");
  }
  printf("\n");
}


/* Funcao que calcula a distancia e o predecedor considerando todos os pares
   vertices (distancia de todos para todos), com base no algoritmo de 
   Floyd-Warshall.
*/
void calculaDistanciaFloydWarshall(Grafo* g, int** dist, int** pred){
  int i, j, k, n;
  n = g->numVertices;
  for(i=0;i<n;i++)
    for(j=0;j<n;j++)
      if(g->matriz[i][j] != false){
        dist[i][j] = g->matriz[i][j];
        pred[i][j] = i;
      }else{
        dist[i][j] = INFINITO;
        pred[i][j] = -1;
      }
  for(i=0;i<n;i++) {
    dist[i][i] = 0;
    pred[i][i] = i;
  }

  for(k=0;k<n;k++)
    for(i=0;i<n;i++)
      for(j=0;j<n;j++)
        if(dist[i][j] > dist[i][k] + dist[k][j]){
          dist[i][j] = dist[i][k] + dist[k][j];
          pred[i][j] = pred[k][j];
        }
}


/* Funcao que exibe uma matriz de distancias.
   Caso a distancia seja INFINITO, imprime '-'.
*/
void exibeMatrizDistancias(int** matriz, int n){
  printf("Exibindo matriz de distancias.\n");
  int x, y;
  for (x=0; x<n; x++) printf("\t%3i", x);
  printf("\n");
  for (x=0; x<n; x++){
    printf("%i", x);
    for (y=0; y<n; y++){
      if (matriz[x][y] == INFINITO) printf("\t-");
      else printf("\t%3i",matriz[x][y]);
    }
    printf("\n");
  }
  printf("\n");
}


/* Funcao que exibe uma matriz de predecessores.
   Caso a distancia seja INFINITO, imprime '-'.
*/
void exibeMatrizPredecessores(int** matriz, int n){
  printf("Exibindo matriz de predecessores.\n");
  int x, y;
  for (x=0; x<n; x++) printf("\t%3i", x);
  printf("\n");
  for (x=0; x<n; x++){
    printf("%i", x);
    for (y=0; y<n; y++){
      printf("\t%3i",matriz[x][y]);
    }
    printf("\n");
  }
  printf("\n");
}


/* Funcao que exibe os valores de um arranjo de numeros reais */
void exibeArranjoReais(double* arranjo, int n){
  int x;
  for (x=0; x<n; x++) printf("  v%i\t", x);
  printf("\n");
  for (x=0; x<n; x++){
    printf("%4.3f\t",arranjo[x]);
  }
  printf("\n\n");
}




/* FUNCOES QUE DEVEM SER COMPLETADAS PARA RESOLVER O EP.
   A DESCRICAO DE CADA FUNCAO ESTA NO ENUNCIADO DO EP.
   www.each.usp.br/digiampietri/ACH2024/ep2/ep2.pdf        */


/* Funcao que calcula a Centralidade de Grau de todos os vertices. */
void centralidadeDeGrau(Grafo* g, double* valores) {

  /* COMPLETE/IMPLEMENTE ESTA FUNCAO */

  /*
  -> A função percorre a matriz pelas colunas, da primeira coluna até a ultima
  -> Nesse caso a coluna representa quais vertices apontam para um determinado vertice
  -> Percorrendo a lista podemos verificar se há um relacionamento e iterar em um contador
  -> No final de cada ciclo do primeiro "for" conseguimos calcular a Centralidade de grau do determiando vertice
  */

  int i, j;
  int contGrauEntrada;
  for(i = 0; i < g->numVertices; i++) {
    contGrauEntrada = 0; // Contador do grau de entrada
    for(j = 0; j < g->numVertices; j++) {
      if(g->matriz[j][i] && j != i) {
        contGrauEntrada++;
      }
    }
    valores[i] = contGrauEntrada / (g->numVertices-1.0); // Operação para calculo da Centralidade de Grau
  }
}


/* Funcao que calcula a Centralidade de Proximidade de todos os vertices. */
void centralidadeDeProximidade(Grafo* g, double* valores) {

  /* COMPLETE/IMPLEMENTE ESTA FUNCAO */

  /*
  -> Função que basicamente para cada valor de 'valores[]' precisamos inserir numero de vertices - 1 e dividir pela soma da distancia de um vertice em relação a todos
  -> Primeiro precisamos da matriz de distancias
  -> Então se tivermos essa matriz de distancia: 
            0       1       2       3       4
  0         0       1       1       2       2
  1         2       0       1       1       1
  2         3       4       0       1       2
  3         2       3       3       0       1
  4         1       2       2       3       0

  -> Temos que o numero de vertice - 1 seja igual a 4 nesse caso e que a soma de todas as distancias seja 6
  -> Então teremos 4/6 que dará 0.666 aproximadamente */

  int verticesMenosUm = g->numVertices -1;
  int i, j;

  int **dist = (int**)malloc(sizeof(int*) * g->numVertices); // inicialização da matriz de distancias
  int **pred = (int**)malloc(sizeof(int*) * g->numVertices); // inicialização da matriz de predecessores

  // Iniciar os valores dentro das linhas
  for (i = 0; i < g->numVertices; i++) {
      dist[i] = (int*)malloc(sizeof(int) * g->numVertices);
      pred[i] = (int*)malloc(sizeof(int) * g->numVertices);
  }

  // Alimenta nossa matriz de distancias e matriz de predecessores
  calculaDistanciaFloydWarshall(g, dist, pred);

  // Loop para percorrer a matriz de distancias e inserir a centralidade de proximidade no indice certo da lista 'valores[]' 
  for(i = 0; i < g->numVertices; i++) {
    float distanciaVertice = 0.0;
    for(j = 0; j < g->numVertices; j++) {
      distanciaVertice+=dist[i][j]; 
    }
    valores[i] = verticesMenosUm/distanciaVertice;
  }

  // Libera memória das matrizes geradas
  free(dist);
  free(pred);
}


/* Funcao que calcula a Centralidade de Intermediacao de todos os vertices. */
void centralidadeDeIntermediacao(Grafo* g, double* valores) {

  /* COMPLETE/IMPLEMENTE ESTA FUNCAO */

  /*
  -> Função que conta quantas intermediações os vertices de um grafo possuem
  -> ou seja, se ele está no caminho entre 2 outros vertices podemos iterar dentro de valores[]
  -> Aqui usamos FloydWarshall para obter a matriz de predecessores e verificar quais vertices possuem dentro de um caminho qualquer
  -> Se houver um vertice no caminho de outros 2 vertices podemos iterar no indice desse vertice
  */

  // Variavel de normalização
  double normalizacaoDenominador = (g->numVertices-1)*(g->numVertices-2);
  int i, j, k;

  int **dist = (int**)malloc(sizeof(int*) * g->numVertices); // inicialização da matriz de distancias
  int **pred = (int**)malloc(sizeof(int*) * g->numVertices); // inicialização da matriz de predecessores

  // Iniciar os valores em valores[]
  for(i = 0; i < g->numVertices; i++) {
    valores[i] = 0;
  }

  // Iniciar os valores dentro das linhas
  for (i = 0; i < g->numVertices; i++) {
      dist[i] = (int*)malloc(sizeof(int) * g->numVertices);
      pred[i] = (int*)malloc(sizeof(int) * g->numVertices);
  }

  // Alimenta nossa matriz de distancias e matriz de predecessores
  calculaDistanciaFloydWarshall(g, dist, pred);

  // Percorre a matriz para ver todos os caminhos possiveis
  for(i = 0; i < g->numVertices; i++) {
    for(j = 0; j < g->numVertices; j++) {
      if (i == j) continue; // Verificar se o caminho é em relação ao proprio vertice
      
      int atual = pred[i][j]; // Pega o predecessor do vertice J

      // Loop que passa por todos os predecessores existentes no meio do caminho de i para j
      // Verifica se o predecessor não é o vertice de origem, quando for o predecessor de origem sai do loop, ou se for um caminho direto nem entra no loop
      while (atual != i) {
          valores[atual] += 1.0;
          atual = pred[i][atual];
      }
    }
  }

  // Normalização de cada intermediação
  for(i = 0; i < g->numVertices; i++) {
    valores[i] /= normalizacaoDenominador;
  }

  // Libera memória das matrizes geradas
  free(dist);
  free(pred);
}


/* Função que calcula a Centralidade Page Rank de todos os vertices. */
void centralidadePageRank(Grafo* g, double* valores, int iteracoes) {

  /* COMPLETE/IMPLEMENTE ESTA FUNCAO */

  /*
  -> Essa função usa como base o algorimo do google de ranquear paginas
  -> Aqui usamos para grafos, no caso que avaliamos e pontuamos os vertices que tem os seguidores mais influentes
  -> Está é a função de origem: 

  PR(x, 0) = 1 / N

  PR(x, t+1) = (1 - d) / N + d * Σ [ PR(y, t) / Saida(y) ]
                          y ∈ Seguem(x)

  -> A função separa entre a parte constante e o somatório
  */

  // Variaveis
  double d = 0.85;
  int N = g->numVertices;
  double primeiraParteFuncao = (1.0 - d) / N;
  int i, j, k;
  double* pageRankAtual = (double*)malloc(N * sizeof(double));
  double* pageRankProximo = (double*)malloc(N * sizeof(double));
  double valor_inicial = 1.0 / N;


  // Inicializando os PR | Se os iterações for 0 só executa esse bloco e nao executa o resto
  for (int i = 0; i < N; ++i) {
      pageRankAtual[i] = valor_inicial;
      pageRankProximo[i] = valor_inicial;
  }

  // Inicializa 'SaidasY' usado na formula
  int* saidasY = (int*)malloc(N * sizeof(int));
  for(i = 0; i < g->numVertices; i++) {
    saidasY[i] = 0;
  }

  // Calcula o numero de vertices que saem de cada Vertice | Basicamente conta o grau de saida dos vertices
  for (i = 0; i < g->numVertices; i++) {
    for (j = 0; j < g->numVertices; j++) {
      if(g->matriz[i][j] && i != j) {
        saidasY[i]++;
      }
    }
  }

  // Loops que fazer o coração do algorimo, realiza o somatório com os seguidores de cada vertice e calculam um novo PageRank
  for(i = 0; i < iteracoes; i++) { // Itera sobre o numero de 'iteracoes'
    for(j = 0; j < g->numVertices; j++) {
      double somatorio = 0.0; // Inicializa um novo somatorio a cada iteracao
      for(k = 0; k < g->numVertices; k++) {
        // Verifica somente os vertices de chegada no vertice 'k'
        if(g->matriz[k][j] && k != j) {
          double pr = pageRankAtual[k];
          somatorio+=pr/saidasY[k];
        }
      }
      // Insere no array pageRankProximo os valores de PR de iteração+1
      pageRankProximo[j] = primeiraParteFuncao + d * somatorio; 
    }

    // Atualiza o atual para a proxima iteração (Se ouver)
    for (j = 0; j < N; j++) {
      pageRankAtual[j] = pageRankProximo[j];
    }
  }

  // Passa o resultado final para 'valores[]'
  for (i = 0; i < N; i++) {
    valores[i] = pageRankAtual[i];
  }

  // Libera memoria
  free(pageRankAtual);
  free(pageRankProximo);
  free(saidasY);
}


/* FIM DAS FUNCOES QUE DEVEM SER COMPLETADAS */


/* Funcao que invoca e exibe os valores de cada uma das funcoes que voces
   devem implementar neste EP.
*/
void testaFuncoes(Grafo* g, int n){

  double* valoresReais = (double*)malloc(sizeof(double)*n);

  printf("Centralidade de Grau:\n");
  centralidadeDeGrau(g, valoresReais);
  exibeArranjoReais(valoresReais, n);

  printf("Centralidade de Proximidade:\n");
  centralidadeDeProximidade(g, valoresReais);
  exibeArranjoReais(valoresReais, n);

  printf("Centralidade de Intermediacao:\n");
  centralidadeDeIntermediacao(g, valoresReais);
  exibeArranjoReais(valoresReais, n);

  printf("Centralidade Page Rank: 0 iteracoes\n");
  centralidadePageRank(g, valoresReais, 0);
  exibeArranjoReais(valoresReais, n);

  printf("Centralidade Page Rank: 1 iteracao\n");
  centralidadePageRank(g, valoresReais, 1);
  exibeArranjoReais(valoresReais, n);

  printf("Centralidade Page Rank: 10 iteracoes\n");
  centralidadePageRank(g, valoresReais, 10);
  exibeArranjoReais(valoresReais, n);

  printf("Centralidade Page Rank: 100 iteracoes\n");
  centralidadePageRank(g, valoresReais, 100);
  exibeArranjoReais(valoresReais, n);

  free(valoresReais);
}


/* Funcao main para testar as funcoes implementadas neste EP.
   Esta parte do codigo nao sera usada na correcao do EP e nao contempla
   todos os testes possiveis.
   Fique a vontade para realizar testes adicionais.
*/
int main() {

  int n = 5;
  double* valoresReais = (double*)malloc(sizeof(double)*n);
  
  Grafo g1;

  printf("PRIMEIRO EXEMPLO\n");

  inicializaGrafo(&g1, n);
  insereAresta(&g1,0,1);
  insereAresta(&g1,1,2);
  insereAresta(&g1,2,3);
  insereAresta(&g1,3,4);
  insereAresta(&g1,4,0);
  insereAresta(&g1,0,2);
  insereAresta(&g1,1,4);
  insereAresta(&g1,1,3);

  exibeGrafo(&g1);

  testaFuncoes(&g1, n);


  printf("\n\nSEGUNDO EXEMPLO [auto-lacos]\n");
  insereAresta(&g1,0,0);
  insereAresta(&g1,2,2);

  exibeGrafo(&g1);

  testaFuncoes(&g1, n);


  printf("\n\nTERCEIRO EXEMPLO\n");
  Grafo g2;
  inicializaGrafo(&g2, n);
  insereAresta(&g2,0,2);
  insereAresta(&g2,1,2);
  insereAresta(&g2,3,2);
  insereAresta(&g2,4,2);
  insereAresta(&g2,2,0);
  insereAresta(&g2,2,1);
  insereAresta(&g2,2,3);
  insereAresta(&g2,2,4);

  exibeGrafo(&g2);

  testaFuncoes(&g2, n);


  printf("\n\nQUARTO EXEMPLO (grafo aleatorio)\n\n");
  n = 10;
  int arestas = 40;
  
  Grafo* g3 = criaGrafoAleatorio(n,arestas);

  exibeGrafo(g3);
 
  testaFuncoes(g3, n);

  return 0;  
}