cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(filtre)

# Bilgisayardaki PCL kütüphanesini bul
find_package(PCL 1.2 REQUIRED)

# PCL'in başlık ve kütüphane yollarını projeye bağla
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# main.cpp dosyamızdan "filtrele" adında bir program üret
add_executable(filtrele main.cpp)

# PCL yeteneklerini programımıza enjekte et
target_link_libraries(filtrele ${PCL_LIBRARIES})