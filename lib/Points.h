#pragma once

#include <tuple>

namespace fzi
{
    namespace top_uav
    {
        template <typename T>
        struct Vec3D
        {
            T x, y, z;
        };

        struct DoubleNewType
        {
            double value;
            operator double() const { return this->value; }
        };

        struct StartPointCoord : public DoubleNewType
        {
        };

        struct EndPointCoord : public DoubleNewType
        {
        };

        struct StartVelocityCoord : public DoubleNewType
        {
        };

        struct EndVelocityCoord : public DoubleNewType
        {
        };


        template <typename... Ts>
        constexpr std::tuple<Ts...> get_x(const std::tuple<Vec3D<Ts>...>& tup)
        {
            return std::apply([](auto const &...args)
                              { return std::make_tuple(args.x...); },
                              tup);
        }
        template <typename... Ts>
        constexpr std::tuple<Ts...> get_y(const std::tuple<Vec3D<Ts>...>& tup)
        {
            return std::apply([](auto const &...args)
                              { return std::make_tuple(args.y...); },
                              tup);
        }

        template <typename... Ts>
        constexpr std::tuple<Ts...> get_z(const std::tuple<Vec3D<Ts>...>& tup)
        {
            return std::apply([](auto const &...args)
                              { return std::make_tuple(args.z...); },
                              tup);
        }
        using StartPoint = Vec3D<StartPointCoord>;
        using EndPoint = Vec3D<EndPointCoord>;
        using StartVelocity = Vec3D<StartVelocityCoord>;
        using EndVelocity = Vec3D<EndVelocityCoord>;
        using Points = std::tuple<StartPoint, EndPoint, StartVelocity, EndVelocity>;
        using PointsSingleDim = std::tuple<StartPointCoord, EndPointCoord, StartVelocityCoord, EndVelocityCoord>;

        constexpr const StartPointCoord& (*p_s)(const PointsSingleDim&p) = std::get<StartPointCoord>;
        constexpr const EndPointCoord& (*p_e)(const PointsSingleDim&p) = std::get<EndPointCoord>;
        constexpr const StartVelocityCoord& (*v_s)(const PointsSingleDim&p) = std::get<StartVelocityCoord>;
        constexpr const EndVelocityCoord& (*v_e)(const PointsSingleDim&p) = std::get<EndVelocityCoord>;
    }
}
