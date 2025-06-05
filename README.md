
# Centralidade em Redes Sociais com Grafos

**Disciplina:** ACH2024 - Algoritmos e Estruturas de Dados II  
**Instituição:** EACH-USP  
**Semestre:** 1º Semestre de 2025  
**Professor:** Luciano Digiampietri  
**Aluno:** Kauã Lima de Souza – NUSP: 15674702  

## Descrição

Este repositório contém a implementação do segundo exercício-programa da disciplina ACH2024, cujo objetivo é implementar e analisar diferentes medidas de **centralidade em redes sociais**, representadas como **grafos direcionados não ponderados**, utilizando matriz de adjacência.

## Medidas de Centralidade

As funções implementadas devem calcular as seguintes medidas de centralidade para todos os vértices de um grafo:

1. **Centralidade de Grau (`centralidadeDeGrau`)**
   - Mede a quantidade de conexões recebidas (grau de entrada), normalizada.
2. **Centralidade de Proximidade (`centralidadeDeProximidade`)**
   - Mede o quão próximo um vértice está de todos os outros, com base nas menores distâncias.
3. **Centralidade de Intermediação (`centralidadeDeIntermediacao`)**
   - Mede quantas vezes um vértice aparece nos caminhos mais curtos entre pares de vértices.
4. **Centralidade PageRank (`centralidadePageRank`)**
   - Avalia a importância de um vértice com base na importância de quem aponta para ele.

## Algoritmos Utilizados

- **Floyd-Warshall**: Para calcular as distâncias e predecessores entre todos os pares de vértices, essencial para as medidas de proximidade e intermediação.
- **PageRank (iterativo)**: Aplica uma fórmula baseada em um fator de amortecimento (0.85) para simular a importância de cada vértice.

## Estrutura do Projeto

- `Grafo`: Estrutura principal representando o grafo com matriz de adjacência.
- Funções auxiliares:
  - Inicialização, inserção/remoção de arestas, exibição do grafo.
- Funções a serem completadas:
  - `centralidadeDeGrau`
  - `centralidadeDeProximidade`
  - `centralidadeDeIntermediacao`
  - `centralidadePageRank`

## Execução

```bash
gcc -o ep2 ep2.c
./ep2
```

## Entrega

A entrega deve ser realizada no ambiente e-Disciplinas, com o arquivo `.c` nomeado com o número USP do aluno (exemplo: `15674702.c`).
