/******************************************************************************
 * Modifier: cuiDarchan
 * Date: 2022-02-03
 * Description: Point cloud data type and simple calculation
 *****************************************************************************/

#pragma once

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <cmath>

using Eigen::Vector3d;
using Eigen::Vector2d;

constexpr double kSinRadToDeg = 57.295779513;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD_RATIO = PI / 180.0;
constexpr double RAD_TO_DEG_RATIO = 180.0 / PI;
constexpr double EARTH_MAJOR_AXIS = 6378137.0188;
constexpr double EARTH_MINOR_AXIS = 6356752.3142;
constexpr double kUTMScaleFactor = 0.9996;

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

double Arclength0fMeridian(double phi) {
  auto n = (EARTH_MAJOR_AXIS - EARTH_MINOR_AXIS) /
           (EARTH_MAJOR_AXIS + EARTH_MINOR_AXIS);
  auto alpha = ((EARTH_MAJOR_AXIS + EARTH_MINOR_AXIS) / 2.0) *
               (1.0 + (std::pow(n, 2) / 4.0) + (std::pow(n, 4) / 64.0));
  auto beta = (-3.0 * n / 2.0) + (9.0 * std::pow(n, 3) / 16.0) +
              (-3.0 * std::pow(n, 5) / 32.0);
  auto gamma = (15.0 * std::pow(n, 2) / 16.0) + (-15.0 * std::pow(n, 4) / 32.0);
  auto delta =
      (-35.0 * std::pow(n, 3) / 48.0) + (105.0 * std::pow(n, 5) / 256.0);
  auto epsilon = (315.0 * std::pow(n, 4) / 512.0);

  return alpha *
         (phi + (beta * std::sin(2.0 * phi)) + (gamma * std::sin(4.0 * phi)) +
          (delta * std::sin(6.0 * phi)) + (epsilon * std::sin(8.0 * phi)));
}

Vector2d MapLatLonToXY(double phi, double lambda, double lambda0) {
  auto ep2 = (std::pow(EARTH_MAJOR_AXIS, 2) - std::pow(EARTH_MINOR_AXIS, 2)) /
             std::pow(EARTH_MINOR_AXIS, 2);
  auto nu2 = ep2 * std::pow(std::cos(phi), 2);
  auto nn =
      std::pow(EARTH_MAJOR_AXIS, 2) / (EARTH_MINOR_AXIS * std::sqrt(1.0 + nu2));

  auto t = std::tan(phi);
  auto t2 = t * t;
  auto l3coef = 1.0 - t2 + nu2;
  auto l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
  auto l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
  auto l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
  auto l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
  auto l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

  auto l = lambda - lambda0;
  auto x = nn * std::cos(phi) * l +
           (nn / 6.0 * std::pow(std::cos(phi), 3) * l3coef * std::pow(l, 3)) +
           (nn / 120.0 * std::pow(std::cos(phi), 5) * l5coef * std::pow(l, 5)) +
           (nn / 5040.0 * std::pow(std::cos(phi), 7) * l7coef * std::pow(l, 7));
  auto y =
      Arclength0fMeridian(phi) +
      (t / 2.0 * nn * std::pow(std::cos(phi), 2) * std::pow(l, 2)) +
      (t / 24.0 * nn * std::pow(std::cos(phi), 4) * l4coef * std::pow(l, 4)) +
      (t / 720.0 * nn * std::pow(std::cos(phi), 6) * l6coef * std::pow(l, 6)) +
      (t / 40320.0 * nn * std::pow(std::cos(phi), 8) * l8coef * std::pow(l, 8));

  return Vector2d(x, y);
}

double UTMCentralMeridian(int zone) {
  constexpr double kSinsDegToRad = 0.01745329252;
  return (-183.0 + (zone * 6.0)) * kSinsDegToRad;
}

std::tuple<Vector3d, int, bool> WGS84ToUTM(const Vector3d &position) {
  auto lat = position.x() * DEG_TO_RAD_RATIO;  // 纬度 28.xx
  auto lon = position.y() * DEG_TO_RAD_RATIO;

  auto zone = static_cast<int>((lon * kSinRadToDeg + 180.0) / 6.0) + 1;
  auto xy = MapLatLonToXY(lat, lon, UTMCentralMeridian(zone));

  xy.x() = xy.x() * kUTMScaleFactor + 500000.0;
  xy.y() = xy.y() * kUTMScaleFactor;
  if (xy.y() < 0.0) {
    xy.y() += 10000000.0;
  }

  return std::make_tuple(Vector3d(xy.x(), xy.y(), position.z()), zone,
                         position.y() < 0.0);
}
