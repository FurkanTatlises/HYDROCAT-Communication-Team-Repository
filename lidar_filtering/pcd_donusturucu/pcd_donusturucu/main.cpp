#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct PointXYZIT {
    PCL_ADD_POINT4D;                  
    float intensity;                  
    double time;                      
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, time, time)
)

// main fonksiyonuna argüman alma (argc, argv) özelliği ekledik
int main(int argc, char** argv) {
    // Eğer kullanıcı eksik dosya adı girerse uyar:
    if (argc != 3) {
        std::cerr << "Kullanim hatasi! Dogru kullanim sekli:" << std::endl;
        std::cerr << "./donusturucu <giris_dosyasi.txt> <cikis_dosyasi.pcd>" << std::endl;
        return -1;
    }

    // Terminalden girilen 1. isim txt dosyamız, 2. isim oluşacak pcd dosyamız
    std::string input_file = argv[1];
    std::string output_file = argv[2];

    pcl::PointCloud<PointXYZIT>::Ptr cloud(new pcl::PointCloud<PointXYZIT>);
    
    std::ifstream file(input_file);
    if (!file.is_open()) {
        std::cerr << "Hata: " << input_file << " dosyasi acilamadi! Dosyanin burada oldugundan emin ol." << std::endl;
        return -1;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        PointXYZIT pt;
        
        if (ss >> pt.x >> pt.y >> pt.z >> pt.intensity >> pt.time) {
            cloud->push_back(pt);
        }
    }
    file.close();

    // Okunan nokta yoksa uyar
    if (cloud->empty()) {
        std::cerr << "Hata: Dosyadan hic nokta okunamadi. Icerigin formati dogru mu?" << std::endl;
        return -1;
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Kullanıcının belirlediği isimle pcd dosyasını kaydet
    pcl::io::savePCDFileBinary(output_file, *cloud);
    std::cout << "Basarili: " << input_file << " dosyasi " << output_file 
              << " olarak kaydedildi. (" << cloud->size() << " nokta)" << std::endl;

    return 0;
}