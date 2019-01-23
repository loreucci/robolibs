/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "camera.h"

#include <cmath>

#include <utilities/utilities.h>

Camera::Camera(const CameraParameters& params) {

    setParameters(params);

}

void Camera::setParameters(const CameraParameters& params) {

    sx = params.sx;
    sy = params.sy;
    fx = params.fx;
    fy = params.fy;
    cx = params.cx;
    cy = params.cy;

}

MetricPoint Camera::pixel2metric(PixelPoint p) {
    return pixel2metric(p.u, p.v);
}

MetricPoint Camera::pixel2metric(double u, double v) {
    MetricPoint ret;
    ret.x = (u - cx)/fx;
    ret.y = (v - cy)/fy;
    return ret;
}

PixelPoint Camera::metric2pixel(MetricPoint p) {
    return metric2pixel(p.x, p.y);
}

PixelPoint Camera::metric2pixel(double x, double y) {
    PixelPoint ret;
    ret.u = x * fx + cx;
    ret.v = y * fy + cy;
    return ret;
}

AnglePoint Camera::metric2angle(MetricPoint p) {
    return metric2angle(p.x, p.y);
}

AnglePoint Camera::metric2angle(double x, double y) {
    AnglePoint ret;
    double rho = std::sqrt(x*x+y*y+1);
    ret.a = -std::atan(x)*180/Utils::PI; //azimuth
    ret.e = -std::asin(y/rho)*180/Utils::PI; //elevation
    return ret;
}

MetricPoint Camera::angle2metric(AnglePoint p) {
    return angle2metric(p.a, p.e);
}

MetricPoint Camera::angle2metric(double a, double e) {
    MetricPoint ret;
    ret.x = -std::tan(a*Utils::PI/180);
    ret.y = -std::tan(e*Utils::PI/180)*std::sqrt(ret.x*ret.x+1);
    return ret;
}

AnglePoint Camera::pixel2angle(PixelPoint p) {
    return pixel2angle(p.u, p.v);
}

AnglePoint Camera::pixel2angle(double u, double v) {
    return metric2angle(pixel2metric(u, v));
}

PixelPoint Camera::angle2pixel(AnglePoint p) {
    return angle2pixel(p.a, p.e);
}

PixelPoint Camera::angle2pixel(double a, double e) {
    return metric2pixel(angle2metric(a, e));
}

PixelPoint Camera::pixel2norm(PixelPoint p) {
    return pixel2norm(p.u, p.v);
}

PixelPoint Camera::pixel2norm(double u, double v) {
    PixelPoint ret;
    ret.u = 2*u/sx - 1;
    ret.v = 2*v/sy - 1;
    return ret;
}

PixelPoint Camera::norm2pixel(PixelPoint p) {
    return norm2pixel(p.u, p.v);
}

PixelPoint Camera::norm2pixel(double u, double v) {
    PixelPoint ret;
    ret.u = sx*(u+1)/2;
    ret.v = sy*(v+1)/2;
    return ret;
}

AnglePoint Camera::norm2angle(PixelPoint p) {
    return norm2angle(p.u, p.v);
}

AnglePoint Camera::norm2angle(double u, double v) {
    return pixel2angle(norm2pixel(u, v));
}

MetricPoint Camera::norm2metric(PixelPoint p) {
    return norm2metric(p.u, p.v);
}

MetricPoint Camera::norm2metric(double u, double v) {
    return pixel2metric(norm2pixel(u, v));
}

