#include <iostream>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <climits>
#include <stack>
#include <unordered_map>

using namespace std;

struct Graph {
    vector<unordered_set<int>> adjList;

    Graph(int n) : adjList(n) {}

    void addEdge(int from, int to) {
      adjList[from].insert(to);
    }
};

// Perform a topological sort of the graph
void topologicalSort(const vector<unordered_set<int>>& graph, vector<int>& sortedNodes, unordered_set<int>& visited, int node) {
  visited.insert(node);
  for (int successor : graph[node]) {
    if (visited.find(successor) == visited.end()) {
      topologicalSort(graph, sortedNodes, visited, successor);
    }
  }
  sortedNodes.push_back(node);
}

// Helper function for the first pass of Kosaraju's algorithm
void kosarajuDFS(const vector<unordered_set<int>>& graph, int node, vector<bool>& visited, stack<int>& finishedStack) {
  visited[node] = true;
  for (int successor : graph[node]) {
    if (!visited[successor]) {
      kosarajuDFS(graph, successor, visited, finishedStack);
    }
  }
  finishedStack.push(node);
}

// Helper function for the second pass of Kosaraju's algorithm
void kosarajuDFSReverse(const vector<unordered_set<int>>& reversedGraph, int node, vector<bool>& visited, vector<int>& scc, int sccNumber) {
  visited[node] = true;
  scc[node] = sccNumber;
  for (int successor : reversedGraph[node]) {
    if (!visited[successor]) {
      kosarajuDFSReverse(reversedGraph, successor, visited, scc, sccNumber);
    }
  }
}

// Find the strongly connected components using Kosaraju's algorithm
vector<vector<int>> kosaraju(const vector<unordered_set<int>>& graph, vector<unordered_set<int>>& reversedGraph) {
  int n = graph.size();
  vector<bool> visited(n, false);
  stack<int> finishedStack;

  // First pass to fill the stack
  for (int i = 0; i < n; ++i) {
    if (!visited[i]) {
      kosarajuDFS(graph, i, visited, finishedStack);
    }
  }

  // Reset visited array
  visited.assign(n, false);

  vector<vector<int>> stronglyConnectedComponents;
  int sccNumber = 0;

  // Second pass to find SCCs
  while (!finishedStack.empty()) {
    int node = finishedStack.top();
    finishedStack.pop();

    if (!visited[node]) {
      vector<int> scc(n, -1);
      kosarajuDFSReverse(reversedGraph, node, visited, scc, sccNumber);
      stronglyConnectedComponents.push_back(scc);
      ++sccNumber;
    }
  }


  return stronglyConnectedComponents;
}

// Perform the transformation of the graph to SCC graph
Graph graphToSccGraph(const Graph& graph) {
  int n = graph.adjList.size();
  vector<unordered_set<int>> reversedGraph(n);
  for (int i = 0; i < n; ++i) {
    for (int successor : graph.adjList[i]) {
      reversedGraph[successor].insert(i);
    }
  }

  vector<vector<int>> stronglyConnectedComponents = kosaraju(graph.adjList, reversedGraph);

  Graph sccGraph(stronglyConnectedComponents.size());

  unordered_map<int, int> vertexToScc;
  unordered_map<int, int> sccToRep;

  // Add a representative for each SCC
  for (int i = 0; i < (int) stronglyConnectedComponents.size(); ++i) {
    int rep = i;
    sccGraph.addEdge(rep, rep);  // Add self-loop

    sccToRep[i] = rep;
    for (int vertex : stronglyConnectedComponents[i]) {
      vertexToScc[vertex] = i;
    }
  }

  // Add edge representatives
  for (int i = 0; i < n; ++i) {
    int sourceRep = sccToRep[vertexToScc[i]];
    for (int successor : graph.adjList[i]) {
      int destRep = sccToRep[vertexToScc[successor]];
      if (sourceRep != destRep) {
        sccGraph.addEdge(sourceRep, destRep);
      }
    }
  }

  return sccGraph;
}

// Find the longest path in a directed acyclic graph
int longestPath(const vector<unordered_set<int>>& graph) {
  int n = graph.size();
  vector<int> sortedNodes;
  unordered_set<int> visited;

  // Perform topological sort
  for (int i = 0; i < n; ++i) {
    if (visited.find(i) == visited.end()) {
      topologicalSort(graph, sortedNodes, visited, i);
    }
  }

  // Initialize distances
  vector<int> dist(n, INT_MIN);

  // Set the distance for the start node to 0
  dist[sortedNodes.back()] = 0;

  // Dynamic Programming for Longest Path
  for (int i = sortedNodes.size() - 1; i >= 0; --i) {
    int node = sortedNodes[i];
    for (int successor : graph[node]) {
      if(successor != node){
        dist[successor] = max(dist[successor], dist[node] + 1);
      }
    }
  }

  // Find the Maximum Distance
  return *max_element(dist.begin(), dist.end());
}

int main() {
  std::ios::sync_with_stdio(0); // disable sync with c libs
  std::cin.tie(0); // discard cin buffer after each line of input

  int n, m;
  cin >> n >> m;

  if(n < 2 || m <= 0){
    cout << 0 << endl;
    return 0;
  }

  Graph graph(n);

  // Assuming edges are unweighted (weight = 1)
  for (int i = 0; i < m; ++i) {
    int x, y;
    cin >> x >> y;
    graph.addEdge(x - 1, y - 1);  // Adjust indices to start from 0
  }

  Graph sccGraph = graphToSccGraph(graph);

  int result = longestPath(sccGraph.adjList);
  cout << result << endl;

  return 0;
}