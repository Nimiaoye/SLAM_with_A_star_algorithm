#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>
using namespace std;

int mapHeight = 680;
int mapWidth = 602;
vector<vector<int>> mapA(mapWidth, vector<int>(mapHeight));

int main() {
    ifstream mapFile("map.txt");
    
    if (!mapFile.is_open()) {
        cerr << "Unable to open file" << endl;
        return 1;
    }

    string line;
    int j = mapHeight - 1;
    while (getline(mapFile, line) && j >= 0) {
        stringstream ss(line);
        int number;
        int i = 0;
        while (ss >> number && i < mapWidth) {
            mapA[i][j] = number;
            i++;
        }
        j--;
    }
    mapFile.close();

    for (int i = 0; i < mapWidth; i++) {
        cout << mapA[i][286] << " ";
    }
    cout << endl;
    
    return 0;
}
