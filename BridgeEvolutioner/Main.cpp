#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>

#include "Bridge.h"

#include <stdlib.h>  
#include <crtdbg.h>

std::vector<double> best_deflection_gen;
std::vector<double> best_fitness_gen;
std::vector<double> average_deflection_gen;
std::vector<double> average_fitness_gen;
std::shared_ptr<Bridge> best_bridge;
int generations;

double fitness_equation(const std::shared_ptr<Bridge>& bridge)
{
    if (!bridge->validate())
    {
        return 100000000;
    }
    double weight = bridge->weight();
    double deflection = bridge->vertical_deflection();
    double cost = 0;

    if (weight < 120)
    {
        cost = deflection * 3250000;
    }
    else if (weight > 200)
    {
        cost = ((weight - 184) * 25000) + (deflection * 3250000);
    }
    else
    {
        cost = ((weight - 120) * 5000) + (deflection * 3250000);
    }
    if (deflection > 2)
        cost += 10000000;
    return cost;
}

void run_generations(std::vector<std::shared_ptr<Bridge>>& bridges)
{
    for (int i = 0; i < generations; i++)
    {
        std::cout << "Processing Generation " << i+1 << std::endl;
        for (std::shared_ptr<Bridge> bridge : bridges)
        {
            bridge->fitness = fitness_equation(bridge);
        }
        std::sort(bridges.begin(), bridges.end(), [](std::shared_ptr<Bridge> b1, std::shared_ptr<Bridge> b2) {
            return b1->fitness < b2->fitness;
        });

        double average_fitness = 0, average_deflection = 0;
        for (std::shared_ptr<Bridge> bridge : bridges)
        {
            average_fitness += bridge->fitness;
            average_deflection += bridge->deflection;
        }
        average_fitness /= bridges.size();
        average_deflection /= bridges.size();

        best_deflection_gen.push_back(bridges[0]->deflection);
        best_fitness_gen.push_back(bridges[0]->fitness);
        average_deflection_gen.push_back(average_deflection);
        average_fitness_gen.push_back(average_fitness);

        int random;

        for (int i = 1; i < 34; i++)
        {
            random = rand() % 10;
            if (random == 0)
            {
                random = rand() % 34;
                if (random != i)
                    bridges[i]->mutate(bridges[random]);
            }
        }
        for (int i = 34; i < 67; i++)
        {
            random = rand() % 10;
            if (random < 3)
            {
                random = rand() % 34;
                if (random != i)
                    bridges[i]->mutate(bridges[random]);
            }
        }
        for (int i = 67; i < 100; i++)
        {
            random = rand() % 10;
            if (random < 9)
            {
                random = rand() % 34;
                if (random != i)
                    bridges[i]->mutate(bridges[random]);
            }
        }
    }
}

void run1()
{
    srand((unsigned int)time(NULL));
    std::vector<std::shared_ptr<Bridge>> bridges;
    for (unsigned int i = 0; i < 100; i++)
    {
        Joint A = Joint(0, 0);
        Joint B = Joint(10.5, 50);
        Joint C = Joint(21, 0);
        Joint D = Joint(31.5, 50);
        Joint E = Joint(42, 0);
        Joint F = Joint(54, 50);
        Joint G = Joint(66, 0);
        Joint H = Joint(78, 50);
        Joint I = Joint(90, 0);
        Joint J = Joint(102, 50);
        Joint K = Joint(114, 0);
        Joint L = Joint(126, 50);
        Joint M = Joint(138, 0);
        Joint N = Joint(150, 50);
        Joint O = Joint(162, 0);
        Joint P = Joint(176, 50);
        Joint Q = Joint(186, 0);
        Joint R = Joint(198, 50);
        Joint S = Joint(210, 0);
        Joint T = Joint(222, 50);
        Joint U = Joint(234, 0);
        Joint V = Joint(244.5, 50);
        Joint W = Joint(255, 0);
        Joint X = Joint(265.5, 50);
        Joint Y = Joint(276, 0);

        std::shared_ptr<Bridge> bridge = std::make_shared<Bridge>();

        bridge->add_joint(A);
        bridge->add_joint(B);
        bridge->add_joint(C);
        bridge->add_joint(D);
        bridge->add_joint(E);
        bridge->add_joint(F);
        bridge->add_joint(G);
        bridge->add_joint(H);
        bridge->add_joint(I);
        bridge->add_joint(J);
        bridge->add_joint(K);
        bridge->add_joint(L);
        bridge->add_joint(M);
        bridge->add_joint(N);
        bridge->add_joint(O);
        bridge->add_joint(P);
        bridge->add_joint(Q);
        bridge->add_joint(R);
        bridge->add_joint(S);
        bridge->add_joint(T);
        bridge->add_joint(U);
        bridge->add_joint(V);
        bridge->add_joint(W);
        bridge->add_joint(X);
        bridge->add_joint(Y);

        bridge->add_member(A, C);
        bridge->add_member(C, E);
        bridge->add_member(E, G);
        bridge->add_member(G, I);
        bridge->add_member(I, K);
        bridge->add_member(K, M);
        bridge->add_member(M, O);
        bridge->add_member(O, Q);
        bridge->add_member(S, U);
        bridge->add_member(U, W);
        bridge->add_member(W, Y);
        bridge->add_member(B, D);
        bridge->add_member(D, F);
        bridge->add_member(F, H);
        bridge->add_member(H, J);
        bridge->add_member(J, L);
        bridge->add_member(L, N);
        bridge->add_member(N, P);
        bridge->add_member(P, R);
        bridge->add_member(R, T);
        bridge->add_member(T, V);
        bridge->add_member(V, X);
        bridge->add_member(A, B);
        bridge->add_member(B, C);
        bridge->add_member(C, D);
        bridge->add_member(D, E);
        bridge->add_member(E, F);
        bridge->add_member(F, G);
        bridge->add_member(G, H);
        bridge->add_member(H, I);
        bridge->add_member(I, J);
        bridge->add_member(J, K);
        bridge->add_member(K, L);
        bridge->add_member(L, M);
        bridge->add_member(M, N);
        bridge->add_member(N, O);
        bridge->add_member(O, P);
        bridge->add_member(P, Q);
        bridge->add_member(Q, R);
        bridge->add_member(R, S);
        bridge->add_member(S, T);
        bridge->add_member(T, U);
        bridge->add_member(U, V);
        bridge->add_member(V, W);
        bridge->add_member(W, X);
        bridge->add_member(X, Y);

        bridges.push_back(bridge);
    }

    run_generations(bridges);

    best_bridge = bridges[0];

    for (unsigned int i = 1; i < bridges.size(); i++)
    {
        bridges[i].reset();
    }

    bridges.clear();
}

int main()
{
    int i = 1;
    std::cout << "Enter bridge version: ";
    std::cin >> i;
    std::cout << "Enter number of generations: ";
    std::cin >> generations;
    switch (i)
    {
    default:
        run1();
    }

    std::ofstream output_file;
    output_file.open("output.txt");
    std::ofstream data_file;
    data_file.open("data.csv");
    std::ofstream best_bridge_file;
    best_bridge_file.open("best_bridge.txt");

    data_file << "Generation," << "Best_Deflection," << "Best_Fitness," << "Avg_Deflection," << "Avg_Fitness\n";

    for (unsigned int i = 0; i < best_fitness_gen.size(); i++)
    {
        output_file << "BEST\n";
        output_file << "Deflection(in): " << best_deflection_gen[i];
        output_file << "\nFitness value($): " << std::fixed << best_fitness_gen[i];
        output_file << "\nAVERAGE\n";
        output_file << "\nDeflection(in): " << average_deflection_gen[i];
        output_file << "\nFitness value($): " << std::fixed << average_fitness_gen[i] << "\n\n\n";

        data_file << i+1 << ',' << best_deflection_gen[i] << ',' << std::fixed << best_fitness_gen[i] / 1000000 << ',' <<
            average_deflection_gen[i] << ',' << std::fixed << average_fitness_gen[i] / 1000000 << '\n';
    }

    best_bridge_file << best_bridge->print();

    output_file.close();
    data_file.close();
    best_bridge_file.close();
    best_bridge.reset();

    _CrtDumpMemoryLeaks();
    std::cout << "Hit <Enter> to continue.....";
    std::cin.clear(); std::cin.ignore(INT_MAX, '\n');
    std::cin.get();
    return 0;
}