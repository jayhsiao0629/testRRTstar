#include "RRTstarApp.h"
#include "RRTstar.h"
#include "BMP.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <algorithm>
#include <vector>

int runRRTstar(int argc, char **argv)
{
    RRTSTAR* rrtstar;
    int width = 500, height = 500;
    std::string output_path = "";
    std::chrono::duration<double> total_planner_time(0);
#ifdef TEST_TIME
    int ITER = 10;
    std::chrono::duration<double> total_findNearest_time(0);
    std::chrono::duration<double> total_findNearNeighbors_time(0);
    std::chrono::duration<double> total_findParent_time(0);
    for (int iter = 0; iter < ITER; iter++) {
#endif
    if (argc == 1) // use default setting
    {
        Point start_pos(10,490);
        Point end_pos(490, 10);
        float rrt_radius = 25;
        float end_thresh = 10;
        rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh);
        rrtstar->world->setWorldWidth(width);
        rrtstar->world->setWorldHeight(height);
        rrtstar->setMaxIterations(10000);
        rrtstar->setStepSize(10.0);
        for (int j = 0; j < 6; j++)
        for (int i = 5-j; i >= 0; i--)
        {
            Point ob1(i*75 + 50, j*75 + 50);
            Point ob2(i*75 + 100, j*75 + 100);
            rrtstar->world->addObstacle(ob1, ob2);
        }
        rrtstar->world->saveObsToFile("Mfiles//Obstacles.txt");
    } else if (argc == 2 || argc == 3) {
        if (argc == 3)
            output_path = argv[2];
        float x1, y1, x2, y2;
        std::ifstream ifs(argv[1]);
        if (!ifs.is_open()) {
            std::cerr << "Failed to open file.\n";
            exit(-1);
        }
        ifs >> width >> height;
        ifs >> x1 >> y1 >> x2 >> y2;
        Point start_pos(x1,y1);
        Point end_pos(x2, y2);
        float rrt_radius;
        float end_thresh;
        ifs >> rrt_radius >> end_thresh;
        rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh);
        rrtstar->world->setWorldWidth(width);
        rrtstar->world->setWorldHeight(height);
        int MAX_ITER, StepSize;
        ifs >> MAX_ITER >> StepSize;
        rrtstar->setMaxIterations(MAX_ITER);
        rrtstar->setStepSize(StepSize);
        while (ifs >> x1 >> y1 >> x2 >> y2) {
            Point ob1(std::min(x1, x2), std::max(y1, y2));
            Point ob2(std::max(x1, x2), std::min(y1, y2));
            rrtstar->world->addObstacle(ob1, ob2);
        }
        rrtstar->world->saveObsToFile("Mfiles//Obstacles.txt");
    } else {
        fprintf(stderr, "Usage: %s {Map file}\n", argv[0]);
        exit(-1);
    }
    Graph g(width, height, output_path);
    g.SetUpBackGround();
    rrtstar->world->SetAllObs(g);
    rrtstar->savePlanToFile({}, "Mfiles//first_viable_path.txt", {});
    rrtstar->savePlanToFile({}, "Mfiles//Path_after_MAX_ITER.txt", {});
    std::cout << "Starting RRT* Algorithm..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Point> initial_solution =rrtstar->planner(g);
    auto end_time = std::chrono::high_resolution_clock::now();
    total_planner_time += (end_time - start_time);
    rrtstar->savePlanToFile(initial_solution, "Mfiles//first_viable_path.txt", "First viable solution . This file contains vector of points of the generated path.");
    if (!initial_solution.empty()) {
        std::cout << "First Viable Solution Obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->lastnode->cost << std::endl;
        std::cout << "Number of Node: " << rrtstar->GetNumNodes() << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
    int X, Y, old_X = -1, old_Y = -1;
    for (size_t i = 0; i < initial_solution.size(); i++) {
        X = round(initial_solution[i].m_x);
        Y = round(initial_solution[i].m_y);
        int pixel = 3;
        g.DrawRect(std::max(X - pixel, 0), std::max(Y - pixel, 0), std::min(X + pixel, width - 1), std::min(Y + pixel, height - 1), {255, 0, 64});
        if (old_X >= 0 && old_Y >= 0)
            g.DrawLine(X, Y, old_X, old_Y, {255, 0, 64});
        old_X = X;
        old_Y = Y;
    }
    g.WriteImage();
    g.CheckPoint();
    std::vector<Point> optimized_solution;
    while (rrtstar->getCurrentIterations() < rrtstar->getMaxIterations() && !initial_solution.empty())
    {
        std::cout << "=========================================================================" << std::endl;
        std::cout << "The algorithm continues iterating on the current plan to improve the plan" << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();
        optimized_solution = rrtstar->planner(g);
        auto end_time = std::chrono::high_resolution_clock::now();
        total_planner_time += (end_time - start_time);
        std::cout << "More optimal solution has obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->m_cost_bestpath << std::endl;
        std::cout << "Number of Node: " << rrtstar->GetNumNodes() << std::endl;
#ifdef UPDATE_ONCE
    }
#endif
        g.Restore();
        old_X = old_Y = -1;
        for (size_t i = 0; i < optimized_solution.size(); i++) {
            X = round(optimized_solution[i].m_x);
            Y = round(optimized_solution[i].m_y);
            int pixel = 3;
            g.DrawStar(X, Y, pixel, {64, 0, 255});
            if (old_X >= 0 && old_Y >= 0)
                g.DrawLine(X, Y, old_X, old_Y, {64, 0, 255});
            old_X = X;
            old_Y = Y;
        }
        g.WriteImage();
#ifndef UPDATE_ONCE
    }
#endif
    std::cout << "Total time spent in planner : " << total_planner_time.count() << " seconds" << std::endl;
    std::cout << "[findNearest()]: " << rrtstar->findNearest_time.count() << " seconds (" << (rrtstar->findNearest_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findNearNeighbors_time()]: " << rrtstar->findNearNeighbors_time.count() << " seconds (" << (rrtstar->findNearNeighbors_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findParent_time()]: " << rrtstar->findParent_time.count() << " seconds (" << (rrtstar->findParent_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    rrtstar->savePlanToFile(optimized_solution, "Mfiles//Path_after_MAX_ITER.txt", " Optimized Solution after maximum provided iteration.This file contains vector of points of the generated path.");
    if (!optimized_solution.empty()) {
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
#ifdef TEST_TIME
    total_findNearest_time += rrtstar->findNearest_time;
    total_findNearNeighbors_time += rrtstar->findNearNeighbors_time;
    total_findParent_time += rrtstar->findParent_time;
    }
    std::cout << "Total time spent in planner : " << (total_planner_time.count() / ITER) << " seconds" << std::endl;
    std::cout << "[findNearest()]             : " << (total_findNearest_time.count() / ITER) << " seconds (" << (total_findNearest_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findNearNeighbors_time()]  : " << (total_findNearNeighbors_time.count() / ITER) << " seconds (" << (total_findNearNeighbors_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findParent_time()]         : " << (total_findParent_time.count() / ITER) << " seconds (" << (total_findParent_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
#endif
    delete rrtstar;
    return 0;
}
