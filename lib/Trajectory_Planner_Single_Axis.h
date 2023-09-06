//
// Created by Fabian Meyer on 2023-02-13.
//
#pragma once
#ifndef TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_SINGLE_AXIS_H
#define TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_SINGLE_AXIS_H

#include "Points.h"
#include "utils.h"
#include <cmath>
#include <algorithm>



namespace fzi {
    namespace top_uav {

        class Trajectory_Planner_Single_Axis
        {
        public:
            Trajectory_Planner_Single_Axis() = default;
            static double calc_opt_time(const PointsSingleDim& p, const double v_min, const double v_max, const double a_min, const double a_max);

        private:
            /*double v_min, v_max, a_min, a_max;*/
            static double case1(const PointsSingleDim& p, const double v_min, const double v_max, const double a_min, const double a_max);
            static double case2(const PointsSingleDim& p, const double v_min, const double v_max, const double a_min, const double a_max);
            static double case3(const PointsSingleDim& p, const double v_min, const double v_max, const double a_min, const double a_max);
            static double case4(const PointsSingleDim& p, const double v_min, const double v_max, const double a_min, const double a_max);

        };

    } // fzi
} // top_uav



#endif //TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_SINGLE_AXIS_H
