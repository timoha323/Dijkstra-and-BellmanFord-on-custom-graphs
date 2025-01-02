#include "menu.h"
#include "tests.h"
#include <vector>
#include <fstream>

int main() {
    testDijkstra();
    testBellmanFord();
    std::vector<std::pair<std::pair<long long, long long>, long long>> tests = {
        {testPerformance(750, 10'000), 7'500'000},
        {testPerformance(500, 10'000), 5'000'000},
        {testPerformance(250, 10'000), 2'500'000},
        {testPerformance(1'000, 1'000), 1'000'000},
        {testPerformance(100, 1'000), 100000}
    };

    std::ofstream outFile("test_data.txt");
    if (!outFile) {
        return 1;
    }
    for (const auto& test : tests) {
        outFile << test.first.first << " "
                << test.first.second << " "
                << test.second << "\n";
    }
    outFile.close();
    system("python3 ../print_graphics.py");
    menu();
    
    return 0;
}
