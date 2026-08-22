#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include "dynamic_game_planner.h"
//#include "recorder.h"

#if (ENABLE_QUANTIZATION == 1)
    std::string quantization_status = "_withQuanti";
#else
    std::string quantization_status = "_withoutQuanti";
#endif

/** Identifies the arithmetic format in output filenames */
#if USE_POSIT
    std::string format_tag = "posit" + std::to_string(REAL_BITS) + "es" + std::to_string(POSIT_ES);
#else
    std::string format_tag = std::to_string(REAL_BITS);
#endif

void save_lanes_to_csv(const std::vector<VehicleState>& traffic, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    // Write CSV header
    file << "lane_type,x,y,s\n";

    for (const auto& vehicle : traffic) {
        std::vector<std::pair<std::string, Lane>> lanes = {
            {"center", vehicle.centerlane},
            {"left", vehicle.leftlane},
            {"right", vehicle.rightlane}
        };

        for (const auto& [lane_type, lane] : lanes) {
            if (lane.present) {  // Only save if the lane exists
                int num_samples = 20;  // Number of points along the lane
                for (int i = 0; i < num_samples; i++) {
                    double s = i * (lane.s_max / num_samples);
                    double x = lane.spline_x(s);
                    double y = lane.spline_y(s);

                    file << lane_type << "," << x << "," << y << "," << s << "\n";
                }
            }
        }
    }

    file.close();
    std::cout << "Lanes saved to " << filename << std::endl;
}

void save_trajectories_to_csv(const std::vector<VehicleState>& traffic, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    file << "vehicle_id,x,y,psi,s,l,time\n";
    
    for (size_t i = 0; i < traffic.size(); i++) {
        file << i << "," << traffic[i].x << "," << traffic[i].y << "," << traffic[i].psi << "," << 0 << "," << 0 << 0 << "\n";
        for (const auto& point : traffic[i].predicted_trajectory) {
            file << i << "," << point.x << "," << point.y << "," << point.psi << "," << point.s << "," << point.l << "," << point.t_start << "\n";
        }
    }

    file.close();
    std::cout << "Trajectories saved to " << filename << std::endl;
}

int main() {
    
    // Generate center lanes for each vehicle
    int centerlane_length = 50;
    

    //---------------------------------------- INTERSECTION --------------------------------------------------------
    DynamicGamePlanner planner_intersection;
    std::cerr<<"------------------------ Intersection Scenario -----------------------------"<<"\n";

    // Define traffic participants (3 vehicles approaching an intersection)
    TrafficParticipants traffic_intersection = {
        // x, y, v, psi, beta, a, v_target
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 10.0},  // Vehicle 1 (moving along X)
        {10.0, -10.0, 0.0, 1.57, 0.0, 1.0, 10.0}, // Vehicle 2 (coming from bottom Y)
        {-10.0, 10.0, 0.0, -1.57, 0.0, 1.0, 10.0}, // Vehicle 3 (coming from top Y)
        {-10.0, -10.0, 0.0, 0.0, 0.0, 1.0, 10.0}, // Vehicle 4 (moving along X)
        {0.0, 10.0, 0.0, 0.0, 0.0, 1.0, 10.0},  // Vehicle 5 (moving along X) before was {0.0, 10.0, 0.0, 0.0, 0.0, 1.0, 10.0},
        {20.0, 20.0, 0.0, -1.57, 0.0, 1.0, 10.0}, // Vehicle 6 (coming from top Y)
        {10.0, 10.0, 0.0, 3.14, 0.0, 1.0, 10.0}, // Vehicle 7 (moving backward X) before was {10.0, 10.0, 0.0, 3.14, 0.0, 1.0, 10.0}
        {0.0, -10.0, 0.0, 1.57, 0.0, 1.0, 10.0}, // Vehicle 8 (coming from bottom Y)
    };
    
    for (size_t i = 0; i < traffic_intersection.size(); i++) {
        std::vector<double> x_vals, y_vals, s_vals;

        for (int j = 0; j < centerlane_length; j++) {
            if (i == 0) { 
                x_vals.push_back(traffic_intersection[i].x + j * 5.0); // Move forward in X
                y_vals.push_back(traffic_intersection[i].y);
            } else if (i == 1) { 
                x_vals.push_back(traffic_intersection[i].x);
                y_vals.push_back(traffic_intersection[i].y + j * 5.0); // Move forward in Y
            } else if (i == 2) {
                x_vals.push_back(traffic_intersection[i].x);
                y_vals.push_back(traffic_intersection[i].y - j * 5.0); // Move downward in Y
            } else if (i == 3) {
                x_vals.push_back(traffic_intersection[i].x + j * 5.0); // Move forward in X
                y_vals.push_back(traffic_intersection[i].y); 
            } else if (i == 4) { 
                x_vals.push_back(traffic_intersection[i].x + j * 5.0); // Move forward in X
                y_vals.push_back(traffic_intersection[i].y);
            } else if (i == 5) { 
                x_vals.push_back(traffic_intersection[i].x);
                y_vals.push_back(traffic_intersection[i].y - j * 5.0); // Move downward in Y
            } else if (i == 6) {
                x_vals.push_back(traffic_intersection[i].x - j * 5.0); // Move backward in X
                y_vals.push_back(traffic_intersection[i].y); 
            } else if (i == 7) {
                x_vals.push_back(traffic_intersection[i].x);
                y_vals.push_back(traffic_intersection[i].y + j * 5.0); // Move downward in Y
            }
            s_vals.push_back(j * 5.0);
        }

        traffic_intersection[i].centerlane.initialize_spline(x_vals, y_vals, s_vals);
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    planner_intersection.run(traffic_intersection);
    #ifdef USE_RECORDER
        // write recorded data to csv file
        Recorder::getInstance()->writeDataToCSV();
    #endif

    auto end_time = std::chrono::high_resolution_clock::now();

    // Compute duration in milliseconds
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Execution Time for run(): " << elapsed_time.count() << " ms" << std::endl;
    std::cout << "Excution Time for integrate(): " << planner_intersection.getRuntimeForIntegrate_ms()<< " ms" << std::endl;

    traffic_intersection = planner_intersection.traffic;

    // Save trajectories to a CSV file
    std::string output_filename = "trajectories_intersection_" + format_tag + quantization_status + ".csv";
    save_trajectories_to_csv(traffic_intersection, output_filename);
    save_lanes_to_csv(traffic_intersection, "lanes_intersection_" + format_tag + quantization_status + ".csv");

    // Plot this run into media/ (run from build/, so the script is one level up)
    char eps_str[32];
    std::snprintf(eps_str, sizeof(eps_str), "%g", planner_intersection.param.eps);
    std::string plot_cmd = "python3 ../plot_run.py \"" + output_filename + "\""
                         + " --eps " + eps_str;
#if USE_POSIT
    plot_cmd += " --es " + std::to_string(POSIT_ES);
#endif
    if (std::system(plot_cmd.c_str()) != 0) {
        std::cerr << "Warning: plotting failed (" << plot_cmd << ")" << std::endl;
    }

    return 0;
}
