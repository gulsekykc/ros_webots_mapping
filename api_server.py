import threading
import uvicorn
from fastapi import FastAPI
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger 

# --- 1. ROS 2 DÜĞÜMÜ ---
class RobotController(Node):
    def __init__(self):
        super().__init__('fastapi_bridge')
        # SLAM Toolbox Reset Servisi
        self.client = self.create_client(Trigger, '/slam_toolbox/reset')
        
    def reset_slam(self):
        """SLAM'i sıfırlayan fonksiyon"""
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, "SLAM Reset Servisi bulunamadı! (SLAM Toolbox açık mı?)"
        
        req = Trigger.Request()
        future = self.client.call_async(req)
        return True, "Sıfırlama emri gönderildi! Harita temizleniyor..."

# --- 2. FASTAPI UYGULAMASI ---
app = FastAPI()
ros_node = None

@app.on_event("startup")
def startup_event():
    global ros_node
    rclpy.init()
    ros_node = RobotController()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

@app.get("/")
def read_root():
    return {"Durum": "Robot Kontrol Paneli Aktif! 🤖"}

@app.post("/reset-map")
def reset_map():
    if ros_node is None:
        return {"status": "error", "message": "ROS Node başlatılamadı!"}
        
    success, message = ros_node.reset_slam()
    if success:
        return {"status": "success", "message": message}
    else:
        return {"status": "error", "message": message}

# --- 3. BAŞLATMA ---
if __name__ == "__main__":
    print("🌍 Web Sunucusu Başlatılıyor: http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)