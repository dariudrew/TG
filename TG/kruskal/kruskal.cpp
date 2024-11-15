#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstring>
using namespace std;

struct Aresta {
    int v1, v2, peso;
};
struct Grafo {
    int n, m;
    vector<int> vertices;
    vector<int> rank;
};
void imprimirArvore(const string& arvore) {
    cout << arvore << endl;
}

void salvarEmArquivo(const string& arvore, const string& nomeArquivo) {
    ofstream arquivoOut(nomeArquivo);
    if (arquivoOut) {
        arquivoOut << arvore << endl;
    } else {
        cerr << "Erro ao criar o arquivo de saída." << endl;
    }
}

void menu(){
    cout<<"============= MENU ==============="<<endl;
    cout<<"-h: Mostra os parametros de ajuda"<<endl;
    cout<<"-f: Indica o arquivo de entrada"<<endl;
    cout<<"-o: Redireciona a saida para um arquivo"<<endl;
    cout<<"-s: Exibe a árvore gerada"<<endl;
}

void ordenarArestasPorPeso(vector<Aresta> &arestas) {
    sort(arestas.begin(), arestas.end(), [](const Aresta &a, const Aresta &b) {
        return a.peso < b.peso;
    });
}

void unionFind(Grafo &grafo, int x, int y){
    if(grafo.rank[x-1] >= grafo.rank[y-1]){
        grafo.vertices[y-1] = x;
        if(grafo.rank[x-1] == grafo.rank[y-1]){
            grafo.rank[x-1]++;
        }
    }
    else{
        grafo.vertices[x-1] = y;
    }
}

int find(Grafo &grafo, int x){
    if(grafo.vertices[x-1] != x){
        grafo.vertices[x-1] = find(grafo, grafo.vertices[x-1]);
    }
    return grafo.vertices[x-1];
}

string kruskal(Grafo &grafo, vector<Aresta> &arestas){
    string T = "";
    int custoTotal = 0;
    ordenarArestasPorPeso(arestas);
    for(int i = 0; i < grafo.m; i++){
        int u = find(grafo, arestas[i].v1);
        int v = find(grafo, arestas[i].v2);
        if(u != v){
            if (!T.empty()) {
                T += ", ";
            }
            custoTotal+= arestas[i].peso;
            T += "(" + to_string(arestas[i].v1) + "," + to_string(arestas[i].v2) + ")";
            unionFind(grafo, u, v);
        }
    }
    cout << custoTotal << endl;
    return T;
}

// Função para ler o grafo a partir de um arquivo e armazenar as arestas em um vetor
void lerGrafo(ifstream &arquivo, vector<Aresta> &arestas, Grafo &grafo) {
    if (!arquivo) {
        cerr << "Erro ao abrir o arquivo." << endl;
        return;
    }
    
    // Lê o número de vértices e arestas
    arquivo >> grafo.n >> grafo.m;
    grafo.vertices.resize(grafo.n); // Inicializa o vetor de vértices com tamanho `n`
    grafo.rank.resize(grafo.n);
    grafo.rank = vector<int>(grafo.n, 0);

    // Lendo as arestas e os vértices
    for (int i = 0; i < grafo.m; ++i) {
        Aresta aresta;
        arquivo >> aresta.v1 >> aresta.v2 >> aresta.peso;
        arestas.push_back(aresta);
        grafo.vertices[aresta.v1 - 1] = aresta.v1;
        grafo.vertices[aresta.v2 - 1] = aresta.v2;
    }
}

int main(int argc, char *argv[]) {
    string nomeArquivo = "";
    bool mostrarSaida = false;
    bool arquivoSaida = false;

    for (int i = 1; i < argc; i++) {
        if (string(argv[i]) == "-f" && i + 1 < argc) {
            nomeArquivo = argv[i + 1];
        } else if (strcmp(argv[i], "-h") == 0) {
            menu();
            return 0;
        } else if (strcmp(argv[i], "-s") == 0) {
            mostrarSaida = true;
        } else if (strcmp(argv[i], "-o") == 0) {
            arquivoSaida = true;
        }
    }

    if (nomeArquivo.empty()) {
        cerr << "Erro ao tentar abrir o arquivo." << endl
             << "Utilize o parâmetro -f para indicar um arquivo ou -h para pedir ajuda." << endl;
        return 1;
    }

    ifstream arquivo(nomeArquivo);
    if (!arquivo) {
        cerr << "Erro ao abrir o arquivo." << endl;
        return 1;
    }

    vector<Aresta> arestas;
    Grafo grafo;

    lerGrafo(arquivo, arestas, grafo);
   string arvoreGerada = kruskal(grafo, arestas);

    if (arquivoSaida) {
        salvarEmArquivo(arvoreGerada, "arquivo-saida.dat");
    } else if (mostrarSaida) {
        imprimirArvore(arvoreGerada);
    }

    arquivo.close();
    return 0;
}

