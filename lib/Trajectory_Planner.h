//
// Created by Fabian Meyer on 2023-02-13.
//

#pragma once
#ifndef TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
#define TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
#include "Trajectory_Planner_Single_Axis.h"
#include "Config.h"
#include "utils.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <optional>
#include <array>

namespace fzi {
    namespace top_uav {
        static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

        template<typename T>
        struct Vec3D {
            T x;
            T y;
            T z;
        };

        struct AccelerationProfile1D {
            std::array<double, 3> accelerations_segments = {nan, nan, nan};
            std::array<double, 3> time_durations_segments = {nan, nan, nan};
        };

        using AccelerationProfile3D = Vec3D<AccelerationProfile1D>;

        struct Solution
        {
            Vec3D<AccelerationProfile1D> acceleration_profiles;
            double time_optimal_trajectory_duration = nan;
        };

        class Trajectory_Planner
        {
        public:
            Trajectory_Planner(double v_max, double a_max, std::string version);
            Solution calc_opt_time(const double& x_s, const double& x_e, const double& y_s, const double& y_e, const double& z_s, const double& z_e, const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze);
            std::optional<AccelerationProfile3D> synchronization_possible_3d(double t_opt, double x_s, double x_e, double y_s, double y_e, double z_s, double z_e, double v_xs, double v_xe, double v_ys, double v_ye, double v_zs, double v_ze, double v_min_x, double v_max_x, double a_min_x, double a_max_x, double v_min_y, double v_max_y, double a_min_y, double a_max_y, double v_min_z, double v_max_z, double a_min_z, double a_max_z);
            std::optional<AccelerationProfile1D> synchronization_possible(const double& t_opt, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);

        private:
            Configs configs;
            const std::string version;
            bool check_inputs(const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze, const Config& config);

            std::optional<AccelerationProfile1D> sync_possible_pattern1(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            std::optional<AccelerationProfile1D> sync_possible_pattern2(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            std::optional<AccelerationProfile1D> sync_possible_pattern3(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            std::optional<AccelerationProfile1D> sync_possible_pattern4(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);

            std::vector<double> determine_candidate_times(const double& t_opt, const double& x_s, const double& x_e, const double& y_s, const double& y_e, const double& z_s, const double& z_e, const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze, const Config& config);

            void sync_pattern1(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_max, const double& a_max, std::vector<double>& candidates);
            void sync_pattern2(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& a_min, std::vector<double>& candidates);
            void sync_pattern3(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& a_max, std::vector<double>& candidates);
            void sync_pattern4(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& a_min, std::vector<double>& candidates);
        };

    } // fzi
} // top_uav

#endif //TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
