#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <fstream>
#include <vector>
#include <set>
#include <cmath>
#include <algorithm>

struct Node{
    uint32_t id;
    int latitude;
    int longitude;
};

bool operator< (const Node &lhs, const Node &rhs){
    return lhs.id < rhs.id;
}

struct Edge{
    uint32_t from;
    uint32_t to;
};

bool operator< (Edge &lhs, Edge &rhs){
    if (lhs.from == rhs.from){
        return lhs.to < rhs.to;
    }
    return lhs.from < rhs.from;
}

void LoadNodes(const std::string& nodesFileName, std::vector<Node> &Nodes){
    std::ifstream nodesFile(nodesFileName);
    Node currentNode;
    double latitudeFloat, longitudeFloat;
    while(nodesFile >> currentNode.id >> latitudeFloat >> longitudeFloat){
        if (latitudeFloat * 1000000)
        currentNode.latitude = std::lround(latitudeFloat * 1000000);
        currentNode.longitude = std::lround(longitudeFloat * 1000000);
        Nodes.push_back(currentNode);
    }
    std::sort(Nodes.begin(), Nodes.end());
    nodesFile.close();
}

void LoadEdges(std::string edgesFileName, std::vector<Edge> &Edges){
    std::ifstream edgesFile(edgesFileName);
    Edge currentEdge;
    int roadLength;
    while (!edgesFile.eof()){
        edgesFile >> roadLength;
        edgesFile >> currentEdge.from;
        for (uint32_t i = 0; i < roadLength - 1; ++i){
            edgesFile >> currentEdge.to;
            Edges.push_back(currentEdge);
            std::swap(currentEdge.from, currentEdge.to);
            Edges.push_back(currentEdge);
        }
    }
    Edges.pop_back();
    Edges.pop_back();
    std::sort(Edges.begin(), Edges.end());
    edgesFile.close();
}



const double R = 6371000;
const double PI = 3.1415926535897932384626433832795;

double Distance(Node v, Node u){
    if (std::isnan(acos(sin(PI / 180 * ((double)v.latitude / 1000000)) * sin(PI / 180 * ((double)u.latitude / 1000000)) +
                        cos(PI / 180 * ((double)v.latitude / 1000000)) * cos(PI / 180 * ((double)u.latitude / 1000000)) *
                        cos(PI / 180 * ((double)u.longitude - (double)v.longitude) / 1000000)))){
        return 0;
    } else {
    return R * acos(sin(PI / 180 * ((double)v.latitude / 1000000)) * sin(PI / 180 * ((double)u.latitude / 1000000)) +
                    cos(PI / 180 * ((double)v.latitude / 1000000)) * cos(PI / 180 * ((double)u.latitude / 1000000)) *
                    cos(PI / 180 * ((double)u.longitude - (double)v.longitude) / 1000000));
    }
}

std::vector<uint32_t> ids;
std::vector<uint32_t> shifts;
std::vector<uint32_t> parents;
std::vector<double> g;
std::vector<double> h;
std::vector<bool> closed;
Node start;
Node finish;

struct Candidate{
    uint32_t id;
    uint32_t index;
};

bool operator< (const Candidate &lhs, const Candidate &rhs){
    return g[lhs.index] + h[lhs.index] < g[rhs.index] + h[rhs.index];
}

void AddNeighbors(std::ifstream &graphFile, Candidate activeCandidate, std::set<Candidate> &openList){
    Node activeNode;
    activeNode.id = activeCandidate.id;
    graphFile.seekg(sizeof(size_t) + activeCandidate.index * (2 * sizeof(uint32_t) + 2 * sizeof(int)) + sizeof(uint32_t), std::ios_base::beg);
    graphFile.read((char*)&activeNode.latitude, sizeof(int));
    graphFile.read((char*)&activeNode.longitude, sizeof(int));

    uint32_t activeIndex = std::lower_bound(ids.begin(), ids.end(), activeCandidate.id) - ids.begin();
    std::vector<Node> neighbors(shifts[activeIndex + 1] - shifts[activeIndex]);
    graphFile.seekg(sizeof(size_t) + ids.size() * (2 * sizeof(uint32_t) + 2 * sizeof(int)) + sizeof(uint32_t) + shifts[activeIndex] * sizeof(uint32_t), std::ios_base::beg);
    for (int i = 0; i < shifts[activeIndex + 1] - shifts[activeIndex]; ++i){
        graphFile.read((char*)&neighbors[i].id, sizeof(uint32_t));
    }

    for (int i = 0; i < neighbors.size(); ++i){
        uint32_t neighborIndex = std::lower_bound(ids.begin(), ids.end(), neighbors[i].id) - ids.begin();
        graphFile.seekg(sizeof(size_t) + neighborIndex * (2 * sizeof(uint32_t) + 2 * sizeof(int)) + sizeof(uint32_t), std::ios_base::beg);
        graphFile.read((char*)&neighbors[i].latitude, sizeof(int));
        graphFile.read((char*)&neighbors[i].longitude, sizeof(int));
        Candidate tmp{neighbors[i].id, neighborIndex};
        if (!closed[neighborIndex]){
            openList.erase(tmp);
            if (h[neighborIndex] < 0){
                h[neighborIndex] = Distance(neighbors[i], finish);
            }
            if (g[neighborIndex] < 0 || g[neighborIndex] > g[activeIndex] + Distance(activeNode, neighbors[i])){
                g[neighborIndex] = g[activeIndex] + Distance(activeNode, neighbors[i]);
                parents[neighborIndex] = activeIndex;
            }
            openList.insert(tmp);
        }
    }
}

void AStar(uint32_t startId, uint32_t finishId, size_t numberOfNodes, std::ifstream &graphFile, double &ans, std::vector<uint32_t> &path){
    g.assign(numberOfNodes, -1);
    h.assign(numberOfNodes, -1);
    parents.assign(numberOfNodes, -1);
    closed.assign(numberOfNodes, false);

    start.id = startId;
    uint32_t startIndex = std::lower_bound(ids.begin(), ids.end(), start.id) - ids.begin();
    graphFile.seekg(sizeof(size_t) + startIndex * (sizeof(uint32_t) + 2 * sizeof(int) + sizeof(uint32_t)) + sizeof(uint32_t), std::ios_base::beg);
    graphFile.read((char*)&start.latitude, sizeof(int));
    graphFile.read((char*)&start.longitude, sizeof(int));
    g[std::lower_bound(ids.begin(), ids.end(), start.id) - ids.begin()] = 0;

    finish.id = finishId;
    int finishIndex = std::lower_bound(ids.begin(), ids.end(), finish.id) - ids.begin();
    graphFile.seekg(sizeof(size_t) + finishIndex * (sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(uint32_t)) + sizeof(uint32_t), std::ios_base::beg);
    graphFile.read((char*)&finish.latitude, sizeof(int));
    graphFile.read((char*)&finish.longitude, sizeof(int));

    std::set<Candidate> openList;
    Candidate startCandidate{start.id, startIndex};
    openList.insert(startCandidate);
    while(!openList.empty()){
        Candidate activeCandidate = *openList.begin();
        openList.erase(activeCandidate);
        closed[activeCandidate.index] = true;
        if (activeCandidate.id == finish.id){
            uint32_t ind = std::lower_bound(ids.begin(), ids.end(), finish.id) - ids.begin();
            ans = g[ind];
            path.push_back(finishId);
            while(parents[ind] != -1){
                path.push_back(ids[parents[ind]]);
                ind = parents[ind];
            }
            std::reverse(path.begin(), path.end());
            return;
        }
        AddNeighbors(graphFile, activeCandidate, openList);
    }
    
    ans = -1;
}

int main(int argc, char* argv[]) {
    if (strcmp(argv[1],"preprocess") == 0){
        std::string nodesFileName;
        std::string edgesFileName;
        std::string graphFileName;
        for (int i = 2; i < argc; ++i){
            if (strcmp(argv[i], "--nodes") == 0){
                nodesFileName = argv[i + 1];
            }
            if (strcmp(argv[i], "--edges") == 0){
                edgesFileName = argv[i + 1];
            }
            if (strcmp(argv[i], "--output") == 0){
                graphFileName = argv[i + 1];
            }
        }
        std::vector<Node> Nodes;
        std::vector<Edge> Edges;
        LoadNodes(nodesFileName, Nodes);
        LoadEdges(edgesFileName, Edges);

        std::vector<uint32_t> shifts;
        int currentNodeIndex = 0;
        shifts.push_back(0);
        for (uint32_t i = 0; i < Edges.size(); ++i){
            if (Edges[i].from != Nodes[currentNodeIndex].id){
                while(Edges[i].from != Nodes[currentNodeIndex].id){
                    ++currentNodeIndex;
                    shifts.push_back(i);
                }
            }
        }
        while(shifts.size() != Nodes.size() + 1){
            shifts.push_back(Edges.size());
        }

        std::ofstream graphFile (graphFileName, std::fstream::binary);
        char* numberOfNodes = (char*) new size_t(Nodes.size());
        graphFile.write(numberOfNodes, sizeof(size_t));
        for (uint32_t i = 0; i < Nodes.size(); ++i){
            char* nodeIdPtr = (char*)&Nodes[i].id;
            char* nodeLatitude = (char*)&Nodes[i].latitude;
            char* nodelongitude = (char*)&Nodes[i].longitude;
            char* nodeShift = (char*)&shifts[i];
            graphFile.write(nodeIdPtr, sizeof(uint32_t));
            graphFile.write(nodeLatitude, sizeof(int));
            graphFile.write(nodelongitude, sizeof(int));
            graphFile.write(nodeShift, sizeof(uint32_t));
        }
        char* nodeShift = (char*)&shifts[shifts.size() - 1];
        graphFile.write(nodeShift, sizeof(uint32_t));

        int j = 0;
        for (uint32_t i = 0; i < Edges.size(); ++i){
            char* edgeTo = (char*)&Edges[i].to;
            graphFile.write(edgeTo, sizeof(uint32_t));
            if (i == shifts[j]){
                ++j;
            }
        }
        graphFile.close();
    }
    if (strcmp(argv[1], "search") == 0){
        bool isFullOutput = false;
        std::string graphFileName;
        std::string inputFileName;
        std::string outputFileName;
        for (int i = 2; i < argc; ++i){
            if (strcmp(argv[i], "--graph") == 0){
                graphFileName = argv[i + 1];
            }
            if (strcmp(argv[i], "--input") == 0){
                inputFileName = argv[i + 1];
            }
            if (strcmp(argv[i], "--output") == 0){
                outputFileName = argv[i + 1];
            }
            if (strcmp(argv[i], "--full-output") == 0){
                isFullOutput = true;
            }
        }

        std::ifstream inputFile(inputFileName);

        std::ifstream graphFile(graphFileName, std::fstream::binary);
        size_t numberOfNodes;
        graphFile.read((char*)&numberOfNodes, sizeof(size_t));
        ids.resize(numberOfNodes);
        shifts.assign(numberOfNodes + 1, -1);
        for (uint32_t i = 0; i < numberOfNodes; ++i){
            graphFile.read((char*)&ids[i], sizeof(uint32_t));
            graphFile.seekg(2 * sizeof(int), std::ios_base::cur);
            graphFile.read((char*)&shifts[i], sizeof(uint32_t));
        }
        graphFile.read((char*)&shifts[shifts.size() - 1], sizeof(uint32_t));

        std::ofstream outputFile(outputFileName);

        uint32_t startId, finishId;
        while(inputFile >> startId >> finishId){
            double ans = -1;
            std::vector<uint32_t> path;
            AStar(startId, finishId, numberOfNodes, graphFile, ans, path);
            if (ans < 0){
                outputFile << -1 << ' ' << path.size() << '\n';
            } else {
                outputFile.setf(std::ios::fixed, std::ios::floatfield);
                outputFile << ans << ' ';
                if (isFullOutput){
                    outputFile << path.size();
                    for (int i = 0; i < path.size(); ++i){
                        outputFile << ' ' << path[i];
                    }
                }
                outputFile << '\n';
            }
        }
        outputFile.close();
        graphFile.close();
    }
    return 0;
}
