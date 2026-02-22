#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main (int argc, char** argv)
{
  // Kullanıcı dosya ismini girmezse uyar
  if (argc < 2) 
  {
    std::cerr << "Eksik parametre!" << std::endl;
    std::cerr << "Kullanim: ./filtrele <islemek_istediginiz_dosya.pcd>" << std::endl;
    return (-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // 1. ADIM: DOSYAYI OKU
  std::cout << argv[1] << " dosyasi okunuyor..." << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) 
  {
    PCL_ERROR ("Dosya okunamadi! \n");
    return (-1);
  }

  std::cout << "SOR oncesi nokta sayisi: " << cloud->points.size() << std::endl;

  // 2. ADIM: İNCE TEMİZLİK (GÜRÜLTÜ SİLME) FİLTRESİNİ UYGULA
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  
  // Parametre 1: Her nokta için en yakın kaç komşuya bakılacak? (50 ideal bir başlangıçtır)
  sor.setMeanK (50); 
  
  // Parametre 2: Standart Sapma Eşiği (Ne kadar agresif sileceğiz?)
  // 1.0 standarttır. Eğer 0.1 yaparsan etrafında çok sıkı komşusu olmayan her şeyi acımadan siler.
  sor.setStddevMulThresh (1.0); 
  
  sor.filter (*cloud_filtered);

  std::cout << "SOR sonrasi nokta sayisi: " << cloud_filtered->points.size() << std::endl;

  // 3. ADIM: SONUCU KAYDET
  pcl::io::savePCDFileASCII ("sor_sonrasi.pcd", *cloud_filtered);
  std::cout << "Islem tamam! 'sor_sonrasi.pcd' kaydedildi." << std::endl;

  return (0);
}