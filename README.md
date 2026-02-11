## ***Sistem Gereksinimleri***

Projeyi çalıştırmadan önce sisteminizde aşağıdaki bileşenlerin kurulu olduğundan emin olun:

- **İşletim Sistemi:** Ubuntu 22.04 LTS  
- **ROS 2:** Humble Hawksbill  
- **Simülatör:** Webots (R2023b veya üzeri)  
- **Python:** 3.10+

---

## ***Gerekli ROS 2 Paketleri***

Terminalde aşağıdaki komutları çalıştırarak gerekli ROS 2 paketlerini kurun:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server ros-humble-webots-ros2
```

---

## ***Python Bağımlılıkları***

Web arayüzü ve görüntü işleme için gereken Python kütüphanelerini yükleyin:

```bash
pip install fastapi uvicorn pillow
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
