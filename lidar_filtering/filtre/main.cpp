#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Eksik parametre girdiniz!" << std::endl;
    std::cerr << "Kullanim: ./filtrele <islemek_istediginiz_dosya.pcd>" << std::endl;
    return (-1);
  }

  // Ham bulut + her filtre asamasi icin ayri pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud          (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor      (new pcl::PointCloud<pcl::PointXYZ>);

  // =========================================================
  // 1. ADIM: DOSYAYI OKU
  // =========================================================
  std::cout << "\n[1/4] " << argv[1] << " dosyasi okunuyor..." << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    PCL_ERROR ("Dosya okunamadi! \n");
    return (-1);
  }
  std::cout << "      Ham nokta sayisi : " << cloud->points.size() << std::endl;

  // =========================================================
  // 2. ADIM: PASSTHROUGH FİLTRESİ
  // =========================================================
  std::cout << "\n[2/4] PassThrough filtresi uygulanıyor..." << std::endl;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);   // Z ekseninde 0.0 - 2.0 m arasi tut
  pass.filter (*cloud_pass);
  std::cout << "      PassThrough sonrasi: " << cloud_pass->points.size() << " nokta" << std::endl;

  // =========================================================
  // 3. ADIM: VOXEL GRID FİLTRESİ
  // =========================================================
  std::cout << "\n[3/4] VoxelGrid filtresi uygulanıyor..." << std::endl;
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud (cloud_pass);  // PassThrough ciktisini al
  voxel.setLeafSize (0.1f, 0.1f, 0.1f);  // 10cm x 10cm x 10cm voxel
  voxel.filter (*cloud_voxel);
  std::cout << "      VoxelGrid sonrasi : " << cloud_voxel->points.size() << " nokta" << std::endl;

  // =========================================================
  // 4. ADIM: STATISTICAL OUTLIER REMOVAL FİLTRESİ
  // =========================================================
  std::cout << "\n[4/4] Statistical Outlier Removal filtresi uygulanıyor..." << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_voxel);   // VoxelGrid ciktisini al
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_sor);
  std::cout << "      SOR sonrasi       : " << cloud_sor->points.size() << " nokta" << std::endl;

  // =========================================================
  // 5. ADIM: SONUCU KAYDET
  // =========================================================
  pcl::io::savePCDFileASCII ("pipeline_sonrasi.pcd", *cloud_sor);
  std::cout << "\n[TAMAM] Tum filtreler uygulandı." << std::endl;
  std::cout << "        Baslangic : " << cloud->points.size()     << " nokta" << std::endl;
  std::cout << "        Sonuc     : " << cloud_sor->points.size() << " nokta" << std::endl;
  std::cout << "        Cikti     : pipeline_sonrasi.pcd" << std::endl;

  return (0);
}