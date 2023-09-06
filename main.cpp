//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Trajectory_Planner.h"
#include <iostream>
#include <string>


using namespace fzi::top_uav;

////////////////////////////////////////////////
/// EXAMPLE FROM MEYER and GLOCK 2023 //////////
////////////////////////////////////////////////
int main() {

    double v_max = 4.0;
    double a_max = 1.0;

    Trajectory_Planner traj_planner_sota(v_max, a_max, "sota");
    Trajectory_Planner traj_planner_basic(v_max, a_max, "basic");
    Trajectory_Planner traj_planner_improved(v_max, a_max, "improved");

    constexpr auto args = std::make_tuple(
        StartPoint{0.1, 2.0, 4.3},
        EndPoint{3.6, 0.4, 2.6},
        StartVelocity{0.1, -1.9, -0.4},
        EndVelocity{.1, -1.8, 0.6});

    Solution t_opt_basic = traj_planner_basic.calc_opt_time(args);
    Solution t_opt_improved = traj_planner_improved.calc_opt_time(args);
    Solution t_opt_sota = traj_planner_sota.calc_opt_time(args);

    std::cout << "Optimal trajectory duration (SOTA): "
              << std::to_string(t_opt_sota.time_optimal_trajectory_duration) << std::endl;
    std::cout << "____________________________________________" << std::endl;
    std::cout << "Optimal trajectory duration (TOP-UAV-basic): "
              << std::to_string(t_opt_basic.time_optimal_trajectory_duration) << std::endl;
    std::cout << "Times x: [" <<
              std::to_string(t_opt_basic.acceleration_profiles.x.time_durations_segments[0]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.x.time_durations_segments[1]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.x.time_durations_segments[2]) << "]" <<
              " / Accelerations x: [" <<
              std::to_string(t_opt_basic.acceleration_profiles.x.accelerations_segments[0]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.x.accelerations_segments[1]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.x.accelerations_segments[2]) << "]"
              << std::endl;

    std::cout << "Times y: [" <<
              std::to_string(t_opt_basic.acceleration_profiles.y.time_durations_segments[0]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.y.time_durations_segments[1]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.y.time_durations_segments[2]) << "]" <<
              " / Accelerations y: [" <<
              std::to_string(t_opt_basic.acceleration_profiles.y.accelerations_segments[0]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.y.accelerations_segments[1]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.y.accelerations_segments[2]) << "]"
              << std::endl;
    std::cout << "Times z: [" <<
              std::to_string(t_opt_basic.acceleration_profiles.z.time_durations_segments[0]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.z.time_durations_segments[1]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.z.time_durations_segments[2]) << "]" <<
              " / Accelerations z: [" <<
              std::to_string(t_opt_basic.acceleration_profiles.z.accelerations_segments[0]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.z.accelerations_segments[1]) << "," <<
              std::to_string(t_opt_basic.acceleration_profiles.z.accelerations_segments[2]) << "]"
              << std::endl;


    std::cout << "____________________________________________" << std::endl;
    std::cout << "Optimal trajectory duration (TOP-UAV-improved): "
              << std::to_string(t_opt_improved.time_optimal_trajectory_duration) << std::endl;
    std::cout << "Times x: [" <<
              std::to_string(t_opt_improved.acceleration_profiles.x.time_durations_segments[0]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.x.time_durations_segments[1]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.x.time_durations_segments[2]) << "]" <<
              " / Accelerations x: [" <<
              std::to_string(t_opt_improved.acceleration_profiles.x.accelerations_segments[0]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.x.accelerations_segments[1]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.x.accelerations_segments[2]) << "]"
              << std::endl;

    std::cout << "Times y: [" <<
              std::to_string(t_opt_improved.acceleration_profiles.y.time_durations_segments[0]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.y.time_durations_segments[1]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.y.time_durations_segments[2]) << "]" <<
              " / Accelerations y: [" <<
              std::to_string(t_opt_improved.acceleration_profiles.y.accelerations_segments[0]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.y.accelerations_segments[1]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.y.accelerations_segments[2]) << "]"
              << std::endl;
    std::cout << "Times z: [" <<
              std::to_string(t_opt_improved.acceleration_profiles.z.time_durations_segments[0]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.z.time_durations_segments[1]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.z.time_durations_segments[2]) << "]" <<
              " / Accelerations z: [" <<
              std::to_string(t_opt_improved.acceleration_profiles.z.accelerations_segments[0]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.z.accelerations_segments[1]) << "," <<
              std::to_string(t_opt_improved.acceleration_profiles.z.accelerations_segments[2]) << "]"
              << std::endl;
}
