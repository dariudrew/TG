#include <iostream>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>

const int INFINITO = std::numeric_limits<int>::max();

struct Aresta {
    int destino, peso;
    Aresta(int destino, int peso) : destino(destino), peso(peso) {}
};

using Grafo = std::vector<std::vector<Aresta>>;

class HeapPrioridade {
public:
    std::vector<int> custo;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

    HeapPrioridade(const std::vector<int>& vertices, const std::vector<int>& custoInicial) : custo(custoInicial) {
        for (int v : vertices) {
            pq.push({custoInicial[v], v});
        }
    }

    bool vazio() const {
        return pq.empty();
    }

    int pop() {
        int vertice = pq.top().second;
        pq.pop();
        return vertice;
    }

    void decrease_priority(int u, int novoCusto) {
        custo[u] = novoCusto;
        pq.push({novoCusto, u});
    }
};

void Prim(const Grafo& G, int v0) {
    int numVertices = G.size();
    std::vector<int> custo(numVertices, INFINITO);
    std::vector<int> prev(numVertices, -1);

    // Inicialização: custo[v] = infinito e prev[v] = -1 para todos os vértices
    for (int v = 0; v < numVertices; ++v) {
        custo[v] = INFINITO;
        prev[v] = -1;
    }

    // Vértice inicial
    custo[v0] = 0;

    // Inicializar heap de prioridade com os custos iniciais
    std::vector<int> vertices(numVertices);
    for (int i = 0; i < numVertices; ++i) {
        vertices[i] = i;
    }
    HeapPrioridade H(vertices, custo);

    // Algoritmo de Prim
    while (!H.vazio()) {
        int v = H.pop();

        // Para cada vizinho u de v
        for (const Aresta& aresta : G[v]) {
            int u = aresta.destino;
            int peso = aresta.peso;

            // Se o custo de u for maior que o peso da aresta (v, u)
            if (custo[u] > peso) {
                custo[u] = peso;
                prev[u] = v;
                H.decrease_priority(u, custo[u]);
            }
        }
    }

    // Exibir resultados
    int custoTotal = 0;
    std::cout << "Arestas da AGM e seus pesos:\n";
    for (int u = 0; u < numVertices; ++u) {
        if (prev[u] != -1) {
            std::cout << "(" << prev[u] << ", " << u << ") peso: " << custo[u] << "\n";
            custoTotal += custo[u];
        }
    }
    std::cout << "Custo total da AGM: " << custoTotal << "\n";
}

Grafo lerGrafo(const std::string& nomeArquivo, int& verticeInicial) {
    std::ifstream arquivo(nomeArquivo);
    if (!arquivo) {
        std::cerr << "Erro ao abrir o arquivo de entrada.\n";
        exit(1);
    }

    int numVertices, numArestas;
    arquivo >> numVertices >> numArestas;

    Grafo G(numVertices);
    int u, v, peso;
    for (int i = 0; i < numArestas; ++i) {
        arquivo >> u >> v >> peso;
        u--;  // Ajuste para índice 0
        v--;  // Ajuste para índice 0
        G[u].emplace_back(v, peso);
        G[v].emplace_back(u, peso);  // Grafo não-direcionado
    }

    arquivo.close();

    // Solicita ao usuário o vértice inicial
    std::cout << "Digite o vértice inicial (de 1 a " << numVertices << "): ";
    std::cin >> verticeInicial;
    verticeInicial--;  // Ajuste para índice 0

    return G;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Uso: " << argv[0] << " <arquivo_entrada>\n";
        return 1;
    }

    std::string nomeArquivo = argv[1];
    int verticeInicial;

    // Ler grafo do arquivo e obter vértice inicial
    Grafo G = lerGrafo(nomeArquivo, verticeInicial);

    // Executar o algoritmo de Prim
    Prim(G, verticeInicial);

    return 0;
}
