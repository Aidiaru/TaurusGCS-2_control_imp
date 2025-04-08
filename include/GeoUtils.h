#pragma once

namespace GeoUtils {
    // 3D mesafe hesaplama (Haversine formülü + yükseklik farkı)
    double calculateDistance3D(double lat1, double lon1, float alt1, 
                              double lat2, double lon2, float alt2);
    
    // Coğrafi koordinatları kartezyen koordinatlara dönüştürme
    void geoToCartesian(double lat, double lon, float alt, 
                       double& x, double& y, double& z);
    
    // Kartezyen koordinatları coğrafi koordinatlara dönüştürme
    void cartesianToGeo(double x, double y, double z, 
                       double& lat, double& lon, float& alt);
} 