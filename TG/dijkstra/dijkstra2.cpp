#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <limits>
#include <cstring>
#include <sstream>

using namespace std;

struct Aresta {
    int v1, v2, peso;
};

struct Grafo {
    int n, m;
    vector<vector<pair<int, int>>> adj; // Lista de adjacência
};

// Função de leitura do grafo a partir de arquivo
void lerGrafoDeArquivo(const string& nomeArquivo, Grafo& grafo) {
    ifstream arquivo(nomeArquivo);
    if (!arquivo) {
        cerr << "Erro ao abrir o arquivo de entrada." << endl;
        exit(1);
    }

    arquivo >> grafo.n >> grafo.m;
    grafo.adj.resize(grafo.n);

    int v1, v2, peso;
    for (int i = 0; i < grafo.m; ++i) {
        arquivo >> v1 >> v2 >> peso;
        grafo.adj[v1 - 1].emplace_back(v2 - 1, peso);
        grafo.adj[v2 - 1].emplace_back(v1 - 1, peso); // grafo não-direcionado
    }
    arquivo.close();
}

// Algoritmo de Dijkstra usando priority_queue
void dijkstra(const Grafo& grafo, int* dist, int* prev, int s) {
    // Inicialização
    for (int i = 0; i < grafo.n; i++) {
        dist[i] = numeric_limits<int>::max();
        prev[i] = -1;
    }
    dist[s] = 0;

    // Priority queue para min-heap de pares (distância, vértice)
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.emplace(0, s);

    while (!pq.empty()) {
        int d = pq.top().first;
        int v = pq.top().second;
        pq.pop();

        // Ignora se a distância for maior que a registrada (vértice já processado)
        if (d > dist[v]) continue;

        for (const auto& vizinho : grafo.adj[v]) {
            int w = vizinho.first;
            int peso = vizinho.second;

            if (dist[w] > dist[v] + peso) {
                dist[w] = dist[v] + peso;
                prev[w] = v;
                pq.emplace(dist[w], w); // Atualiza a fila de prioridade
            }
        }
    }
}

// Função principal com argumentos de linha de comando
int main(int argc, char* argv[]) {
    string nomeArquivoEntrada, nomeArquivoSaida;
    int verticeInicial = -1;
    bool mostrarAjuda = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0) {
            mostrarAjuda = true;
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            nomeArquivoEntrada = argv[i + 1];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            nomeArquivoSaida = argv[i + 1];
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            verticeInicial = stoi(argv[i + 1]) - 1; // Vértice inicial em 0-based
        }
    }

    if (mostrarAjuda || nomeArquivoEntrada.empty() || verticeInicial == -1) {
        cout << "Uso: ./dijkstra -f <arquivo-entrada> -i <vertice-inicial> [-o <arquivo-saida>] [-h]" << endl;
        return 0;
    }

    // Leitura e processamento do grafo
    Grafo grafo;
    lerGrafoDeArquivo(nomeArquivoEntrada, grafo);
    int* dist = new int[grafo.n];
    int* prev = new int[grafo.n];
    dijkstra(grafo, dist, prev, verticeInicial);

    // Gerar a string de saída
    ostringstream resultado;
    for (int i = 0; i < grafo.n; ++i) {
        resultado << i + 1 << ":" << (dist[i] == numeric_limits<int>::max() ? -1 : dist[i]) << " ";
    }

    if (!nomeArquivoSaida.empty()) {
        ofstream arquivoOut(nomeArquivoSaida);
        if (arquivoOut) {
            arquivoOut << resultado.str() << endl;
        } else {
            cerr << "Erro ao criar o arquivo de saída." << endl;
        }
    } else {
        cout << resultado.str() << endl;
    }

    delete[] dist;
    delete[] prev;
    return 0;
}
