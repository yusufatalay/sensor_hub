# Buildroot Tabanlı Minimal CAN Sensör Hub (BeagleBone Black)

Bu proje, bir STM32F4 kartından (MPU6050 sensörü ile) alınan ivmeölçer verilerinin CAN bus üzerinden BeagleBone Black (BBB) üzerinde çalışan minimal bir Linux sistemine iletilmesini ve işlenmesini gösterir. BBB üzerindeki sistem Buildroot ile özelleştirilmiştir.

## Amaç

* STM32F4 (MPU6050) sensör verilerini CAN bus üzerinden göndermek.
* BeagleBone Black için Buildroot kullanarak minimal ve hızlı açılan bir Linux imajı oluşturmak.
* BBB üzerinde CAN verilerini almak, işlemek ve görüntülemek için C tabanlı bir uygulama geliştirmek (SocketCAN kullanarak).
* Gömülü sistemlerde donanım haberleşmesi (I2C, CAN) ve Linux özelleştirme konularını pratik olarak uygulamak.

## Temel Donanım

* BeagleBone Black (Rev C veya benzeri)
* STM32F4 Discovery (veya MPU6050 bağlı başka bir STM32F4 kartı)
* MPU6050 İvmeölçer/Gyro Sensörü
* SN65HVD230 CAN Transceiver Modülü (veya entegresi) x 2 adet
* 120 Ohm Sonlandırma Direnci x 2 adet
* Gerekli bağlantı kabloları

## Kullanılan Teknolojiler

* **Buildroot:** Minimal Linux imajı oluşturmak için.
* **STM32CubeIDE:** STM32F4 firmware geliştirmek için.
* **C Programlama Dili:** Hem STM32 (HAL Kütüphanesi) hem de BeagleBone Black (SocketCAN) uygulamaları için.
* **CAN Bus:** Düğümler arası haberleşme için.
* **I2C:** STM32F4 ve MPU6050 arası haberleşme için.
* **Make:** BBB uygulamasını derlemek için.

## Depo Yapısı
├── stm32f4_firmware/  # STM32F4 CubeIDE projesi ve kaynak kodları
├── bbb_application/  # BeagleBone Black C uygulaması ve Makefile
├── buildroot_config/ # Buildroot .config dosyası ve notlar
└── README.md         # Bu dosya - Projeye genel bakış

