cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Projemizin adı
project(pcd_donusturucu)

# Modern PCL sürümleri için C++ standardını belirliyoruz (Hata almamak için önemli)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Sistemdeki PCL kütüphanesini bul
find_package(PCL REQUIRED)

# PCL'in başlık (header) ve kütüphane yollarını projeye dahil et
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# main.cpp dosyasından 'donusturucu' adında bir program (executable) üret
add_executable(donusturucu main.cpp)

# PCL kütüphanelerini programımıza bağla
target_link_libraries(donusturucu ${PCL_LIBRARIES})