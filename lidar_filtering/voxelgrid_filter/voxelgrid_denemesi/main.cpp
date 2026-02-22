#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
  // Kullanıcı dosya ismini girmezse uyar
  if (argc < 2) 
  {
    std::cerr << "Eksik parametre!" << std::endl;
    std::cerr << "Kullanim: ./filtrele <islemek_istediginiz_dosya.pcd>" << std::endl;
    return (-1);
  }

  // Zincirleme işlemde uyumlu olması için PCLPointCloud2 yerine PointXYZ kullanıyoruz
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // 1. ADIM: DOSYAYI OKU
  std::cout << argv[1] << " dosyasi okunuyor..." << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) 
  {
    PCL_ERROR ("Dosya okunamadi! \n");
    return (-1);
  }

  std::cout << "VoxelGrid oncesi nokta sayisi: " << cloud->points.size() << std::endl;

  // 2. ADIM: VOXELGRID FİLTRESİNİ UYGULA
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  
  // 10cm x 10cm x 10cm'lik hayali küpler. 
  // Deniz üstünde veri çok yoğun gelirse, hızı artırmak için 
  // buraları 0.2f, 0.2f, 0.2f veya daha büyük yapabilirsin.
  sor.setLeafSize (0.1f, 0.1f, 0.1f); 
  sor.filter (*cloud_filtered);

  std::cout << "VoxelGrid sonrasi nokta sayisi: " << cloud_filtered->points.size() << std::endl;

  // 3. ADIM: SONUCU KAYDET
  pcl::io::savePCDFileASCII ("voxel_sonrasi.pcd", *cloud_filtered);
  std::cout << "Islem tamam! 'voxel_sonrasi.pcd' kaydedildi." << std::endl;

  return (0);
}