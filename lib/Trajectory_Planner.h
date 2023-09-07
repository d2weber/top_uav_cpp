//
// Created by Fabian Meyer on 2023-02-13.
//

#pragma once
#ifndef TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
#define TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
#include "Trajectory_Planner_Single_Axis.h"
#include "Points.h"
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
            const Solution calc_opt_time(const Points&);
            std::optional<AccelerationProfile3D> synchronization_possible_3d(double t_opt, const Points& p, double v_min_x, double v_max_x, double a_min_x, double a_max_x, double v_min_y, double v_max_y, double a_min_y, double a_max_y, double v_min_z, double v_max_z, double a_min_z, double a_max_z);
            std::optional<AccelerationProfile1D> synchronization_possible(const double& t_opt, const PointsSingleDim& p, const double& v_min, const double& v_max, const double& a_min, const double& a_max);

        private:
            Configs configs;
            const std::string version;
            bool check_inputs(const StartVelocity& v_s, const EndVelocity& v_e, const Config& config);

            std::optional<AccelerationProfile1D> sync_possible_pattern1(const double& t_sync, const PointsSingleDim& p, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            std::optional<AccelerationProfile1D> sync_possible_pattern2(const double& t_sync, const PointsSingleDim& p, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            std::optional<AccelerationProfile1D> sync_possible_pattern3(const double& t_sync, const PointsSingleDim& p, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            std::optional<AccelerationProfile1D> sync_possible_pattern4(const double& t_sync, const PointsSingleDim& p, const double& v_min, const double& v_max, const double& a_min, const double& a_max);

            std::vector<double> determine_candidate_times(const double& t_opt, const Points& p, const Config& config);

            void sync_pattern1(const PointsSingleDim&p, const double& v_max, const double& a_max, std::vector<double>& candidates);
            void sync_pattern2(const PointsSingleDim&p, const double& v_min, const double& a_min, std::vector<double>& candidates);
            void sync_pattern3(const PointsSingleDim&p, const double& a_max, std::vector<double>& candidates);
            void sync_pattern4(const PointsSingleDim&p, const double& a_min, std::vector<double>& candidates);
        };

    } // fzi
} // top_uav

#endif //TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
