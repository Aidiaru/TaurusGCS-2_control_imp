#pragma once

#ifndef FORMATION_H
#define FORMATION_H

#include "Swarm.h"

// Formasyon fonksiyonları
void goto_formation(std::shared_ptr<Drone> drone, double target_lat, double target_lon, float target_alt);

// Reverse V formasyonu pozisyon hesaplama
void calculate_reverse_v_position(size_t index, double ref_lat, double ref_lon, float ref_alt,
                                double lat_offset, double lon_offset,
                                double& target_lat, double& target_lon, float& target_alt);

// Normal V formasyonu pozisyon hesaplama
void calculate_v_position(size_t index, double ref_lat, double ref_lon, float ref_alt,
                         double lat_offset, double lon_offset,
                         double& target_lat, double& target_lon, float& target_alt);

// İniş için düz çizgi formasyonu hesaplama
void land_straight_line_formation(size_t index, double ref_lat, double ref_lon, double lat_offset,
                                 double& target_lat, double& target_lon);

#endif // FORMATION_H