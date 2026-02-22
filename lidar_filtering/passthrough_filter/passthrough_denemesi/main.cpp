#include <iostream>
#include <pcl/io/pcd_io.h> // Dosya okuma/yazma için EKLENDİ
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// main fonksiyonuna dışarıdan parametre alabilmesi için argc ve argv ekledik
int main (int argc, char** argv) 
{
  // Eğer kullanıcı dosya adını girmeyi unutursa uyaralım:
  if (argc < 2) 
  {
    std::cerr << "Eksik parametre girdiniz!" << std::endl;
    std::cerr << "Kullanim: ./filtrele <islemek_istediginiz_dosya.pcd>" << std::endl;
    return (-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // "deneme2.pcd" yazan yeri sildik, yerine argv[1] (kullanıcının girdiği 1. parametre) yazdık
  std::cout << argv[1] << " dosyasi okunuyor..." << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) 
  {
    PCL_ERROR ("Dosya okunamadi! \n");
    return (-1);
  }
  std::cout << "Filtreleme oncesi nokta sayisi: " << cloud->points.size() << std::endl;

  // 2. ADIM: PASSTHROUGH FİLTRESİNİ UYGULA
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z"); // Yüksekliğe göre kes
  
  // Deniz aracı senaryosu: Sensörün 1 metre altından (-1.0) 
  // 3 metre yukarısına (3.0) kadar olanları tut, gerisini sil.
  // Bu değerleri kendi aracının sensör yüksekliğine göre değiştirebilirsin.
  pass.setFilterLimits (0.0, 2.0); 
  
  pass.filter (*cloud_filtered);

  std::cout << "Filtreleme sonrasi nokta sayisi: " << cloud_filtered->points.size() << std::endl;

  // 3. ADIM: SONUCU YENİ BİR DOSYA OLARAK KAYDET
  pcl::io::savePCDFileASCII ("passthrough_sonrasi.pcd", *cloud_filtered);
  std::cout << "Islem tamam! Yeni dosya kaydedildi." << std::endl;

  return (0);
}