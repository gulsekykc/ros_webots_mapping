# ROS_WEBOTS_MAPPING
## Bu proje, ROS 2 Humble ve Webots ekosistemini birleştiren, yüksek hassasiyetli bir otonom haritalama çözümüdür. FastAPI entegrasyonu ile terminal karmaşasını ortadan kaldırarak robotik süreç yönetimini modern bir web arayüzüne taşır.



-Yüksek Hassasiyetli SLAM: 0.02 çözünürlük parametresiyle milimetrik detayda dijital haritalama.

-Web Kontrol Merkezi: FastAPI üzerinden sistem başlatma, sıfırlama ve harita yönetimi.

-Gerçek Zamanlı Veri Senkronizasyonu: LiDAR ve odometri verilerinin simülasyon ile ROS düğümleri arasında kesintisiz iletimi.

-Akıllı Harita Arşivleme: Kaydedilen teknik verilerin otomatik olarak PNG formatına dönüştürülmesi ve galeriye eklenmesi.

-Modüler Mimari: Farklı robot modellerine ve simülasyon dünyalarına hızlı uyarlanabilen URDF ve Launch yapısı


<img width="1915" height="863" alt="image" src="https://github.com/user-attachments/assets/207b06fa-0be7-4409-b144-2c5e76aece33" />
<img width="1205" height="874" alt="image" src="https://github.com/user-attachments/assets/c8111fc6-36fa-48fb-862e-8bfe94b2e8b3" />
<img width="1919" height="573" alt="image" src="https://github.com/user-attachments/assets/c2bd3ce7-43e6-41f8-a34f-718ad4a121f3" />


### **Sistem Gereksinimleri**

Projeyi çalıştırmadan önce sisteminizde aşağıdaki bileşenlerin kurulu olduğundan emin olun:

- **İşletim Sistemi:** Ubuntu 22.04 LTS  
- **ROS 2:** Humble Hawksbill  
- **Simülatör:** Webots (R2023b veya üzeri)  
- **Python:** 3.10+

---
### **Python Bağımlılıkları (Sanal Ortam ile)**
Proje bağımlılıklarını izole bir şekilde kurmak için `uv` kullanılması önerilir:
```bash
cd ~/gulse_robotics_ws/src/gulse_scan_driver
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt
```
## ***Gerekli ROS 2 Paketleri***

Terminalde aşağıdaki komutları çalıştırarak gerekli ROS 2 paketlerini kurun:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server ros-humble-webots-ros2
```

---

## ***Kurulum Adımları***

### ***1. Workspace Oluşturma ve Repoyu Klonlama***

```bash
mkdir -p ~/gulse_robotics_ws/src
cd ~/gulse_robotics_ws/src
git clone https://github.com/gulsekykc/ros_webots_mapping.git
```

---

### ***2. Paketi Derleme***

```bash
cd ~/gulse_robotics_ws
colcon build --packages-select gulse_scan_driver
source install/setup.bash
```

---

## ***Çalıştırma Talimatları***

Sistemi tam kapasite çalıştırmak için **3 ayrı terminal sekmesi** kullanın:

---

### ***Terminal 1 — Webots Simülasyonu***

```bash
webots ~/gulse_robotics_ws/src/gulse_scan_driver/worlds/GaziSim_Alpha.wbt
```

---

### ***Terminal 2 — ROS 2 Launch***

```bash
source ~/gulse_robotics_ws/install/setup.bash
ros2 launch gulse_scan_driver gulse_slam_launch.py
```

---

### ***Terminal 3 — Web Paneli***

```bash
cd ~/gulse_robotics_ws/src/gulse_scan_driver
python3 panel.py
```

Ardından tarayıcınızdan aşağıdaki adrese giderek sistemi yönetebilirsiniz:

```
http://localhost:8000
```
"SİSTEMİ SIFIRLA" butonunu kullandıktan sonra değişiklik hemen gelmediyse rviz2 'de reset tuşuna basmanız önerilir.
