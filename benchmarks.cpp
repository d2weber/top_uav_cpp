#include "Trajectory_Planner.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

using namespace fzi::top_uav;

TEST_CASE("Calculate opt_time for example")
{
    double v_max = 4.0;
    double a_max = 1.0;

    constexpr auto args = std::make_tuple(
        StartPoint{0.1, 2.0, 4.3},
        EndPoint{3.6, 0.4, 2.6},
        StartVelocity{0.1, -1.9, -0.4},
        EndVelocity{.1, -1.8, 0.6});


    Trajectory_Planner sota(v_max, a_max, "sota");
    BENCHMARK("example opt_sota")
    {
        return sota.calc_opt_time(args);
    };

    Trajectory_Planner basic(v_max, a_max, "basic");
    BENCHMARK("example basic")
    {
        return basic.calc_opt_time(args);
    };

    Trajectory_Planner improved(v_max, a_max, "improved");
    BENCHMARK("example improved")
    {
        return improved.calc_opt_time(args);
    };
}
