import subprocess
import time
import signal
import os
import glob
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from contextlib import asynccontextmanager
from PIL import Image

# --- SÜREÇ YÖNETİMİ ---
slam_process = None
MAPS_DIR = os.getcwd()

def start_slam():
    """SLAM Toolbox'ı başlatır"""
    global slam_process
    print("🚀 SLAM Başlatılıyor...")
    command = [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py", 
        "use_sim_time:=False", "base_frame:=base_footprint", "odom_frame:=odom", "resolution:=0.02"
    ]
    slam_process = subprocess.Popen(command, preexec_fn=os.setsid)

def stop_proc(proc):
    """Verilen süreci güvenli bir şekilde durdurur"""
    if proc:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=2)
        except:
            pass
    return None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Uygulama başlarken ve kapanırken çalışacak işlemler"""
    start_slam()
    yield
    stop_proc(slam_process)

app = FastAPI(lifespan=lifespan)
app.mount("/map_files", StaticFiles(directory=MAPS_DIR), name="map_files")

# --- API ENDPOINTLERİ ---

@app.get("/get-maps")
def get_maps():
    """Kaydedilmiş PNG haritaları listeler"""
    png_files = glob.glob("*.png")
    png_files.sort(key=os.path.getmtime, reverse=True)
    return {"maps": [f"/map_files/{f}" for f in png_files]}

@app.post("/save-map")
def save_map():
    """Mevcut haritayı kaydeder ve PNG'ye dönüştürür"""
    try:
        timestamp = int(time.time())
        map_name = f"gulse_map_{timestamp}"
        
        # 1. PGM kaydı
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_name]
        subprocess.run(cmd, check=True)
        
        # 2. PNG dönüşümü
        time.sleep(1.5)
        with Image.open(f"{map_name}.pgm") as img:
            img.save(f"{map_name}.png")
            
        return {"status": "success", "message": "Harita Galeriye Eklendi! 💾"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.post("/reset-map")
def reset_map():
    """Sistemi ve haritayı sıfırlar"""
    global slam_process
    stop_proc(slam_process)
    time.sleep(2)
    start_slam()
    return {"status": "success", "message": "Harita ve SLAM Sıfırlandı! ♻️"}

@app.get("/", response_class=HTMLResponse)
def read_root():
    return """
    <html>
    <head>
        <title>GulseScan Hub</title>
        <style>
            body { background: #0d1117; color: white; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; padding: 20px; }
            .container { background: #161b22; padding: 30px; border-radius: 20px; border: 1px solid #30363d; display: inline-block; box-shadow: 0 10px 30px rgba(0,0,0,0.5); }
            .btn-group { margin: 20px 0; }
            .btn { padding: 15px 25px; margin: 8px; border-radius: 10px; border: none; font-weight: bold; cursor: pointer; transition: 0.3s; font-size: 14px; text-transform: uppercase; }
            .btn-save { background: #238636; color: white; }
            .btn-reset { background: #da3633; color: white; }
            .btn:hover { opacity: 0.85; transform: translateY(-2px); box-shadow: 0 5px 15px rgba(0,0,0,0.3); }
            .gallery { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 25px; padding: 30px; }
            .map-card { background: #1c2128; padding: 15px; border-radius: 15px; border: 1px solid #444; transition: 0.3s; }
            .map-card:hover { border-color: #58a6ff; }
            .map-card img { width: 100%; border-radius: 10px; background: white; border: 2px solid #30363d; }
            #status { color: #58a6ff; margin: 20px; font-weight: bold; font-size: 18px; min-height: 24px; }
            h1 { font-size: 2.5em; margin-bottom: 10px; color: #f0f6fc; }
            h2 { border-bottom: 2px solid #30363d; padding-bottom: 10px; margin-top: 50px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 GulseScan Hub</h1>
            <p style="color: #8b949e;">Otonom Haritalama ve Kontrol Merkezi</p>
            
            <div id="status">Sistem Hazır</div>

            <div class="btn-group">
                <button class="btn btn-save" onclick="action('/save-map')">Haritayı Kaydet 💾</button>
                <button class="btn btn-reset" onclick="action('/reset-map')">Sistemi Sıfırla ♻️</button>
            </div>
        </div>
        
        <h2>🖼️ Harita Arşivi</h2>
        <div id="gallery" class="gallery"></div>

        <script>
            async function action(url) {
                const status = document.getElementById('status');
                status.innerText = "⏳ İşlem yürütülüyor...";
                try {
                    const res = await fetch(url, { method: 'POST' });
                    const data = await res.json();
                    status.innerText = data.message;
                    if(url.includes('save') || url.includes('reset')) {
                        setTimeout(loadMaps, 1000);
                    }
                } catch (e) {
                    status.innerText = "❌ Bağlantı Hatası!";
                }
            }

            async function loadMaps() {
                const res = await fetch('/get-maps');
                const data = await res.json();
                const gallery = document.getElementById('gallery');
                gallery.innerHTML = data.maps.map(url => `
                    <div class="map-card">
                        <img src="${url}" alt="Map">
                        <p style="margin-top:10px; font-size:13px; color:#c9d1d9;">${url.split('/').pop()}</p>
                    </div>
                `).join('');
            }
            window.onload = loadMaps;
        </script>
    </body>
    </html>
    """

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)