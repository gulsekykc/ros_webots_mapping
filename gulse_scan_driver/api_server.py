import subprocess
import time
import signal
import os
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse

app = FastAPI()

# --- SLAM YÖNETİMİ ---
slam_process = None

def start_slam():
    """SLAM'i başlatır"""
    global slam_process
    print("🚀 SLAM Başlatılıyor...")
    # Komut: SLAM Toolbox'ı sim_time=False ile başlat
    command = ["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=False"]
    # new_session=True (setsid) diyerek bunu ana programdan ayırıyoruz
    slam_process = subprocess.Popen(command, preexec_fn=os.setsid)

def stop_slam():
    """SLAM'i öldürür"""
    global slam_process
    if slam_process:
        print("🛑 SLAM Durduruluyor...")
        try:
            os.killpg(os.getpgid(slam_process.pid), signal.SIGTERM)
            slam_process.wait()
        except:
            pass
        slam_process = None

@app.on_event("startup")
def startup_event():
    start_slam()

@app.on_event("shutdown")
def shutdown_event():
    stop_slam()

# --- API ENDPOINT (Reset İşlemi) ---
@app.post("/reset-map")
def reset_map():
    stop_slam()
    time.sleep(2) # Kapanması için nefes payı
    start_slam()
    return {"status": "success", "message": "Harita Sıfırlandı!"}

# --- ARAYÜZ (HTML/CSS) ---
@app.get("/", response_class=HTMLResponse)
def read_root():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>GulseScan Kontrol</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body {
                background-color: #0d1117;
                color: #c9d1d9;
                font-family: 'Segoe UI', sans-serif;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify_content: center;
                height: 100vh;
                margin: 0;
            }
            .container {
                text-align: center;
                background: #161b22;
                padding: 40px;
                border-radius: 20px;
                box-shadow: 0 0 30px rgba(88, 166, 255, 0.15);
                border: 1px solid #30363d;
                max-width: 90%;
            }
            h1 { color: #58a6ff; margin: 0 0 10px 0; font-size: 2rem; }
            p { color: #8b949e; margin-bottom: 30px; }
            
            .btn {
                background: linear-gradient(135deg, #da3633, #f85149);
                border: none;
                padding: 15px 40px;
                color: white;
                font-size: 1.1rem;
                font-weight: bold;
                border-radius: 50px;
                cursor: pointer;
                transition: transform 0.2s, box-shadow 0.2s;
                box-shadow: 0 4px 15px rgba(218, 54, 51, 0.4);
            }
            .btn:hover { transform: scale(1.05); box-shadow: 0 6px 20px rgba(218, 54, 51, 0.6); }
            .btn:active { transform: scale(0.95); }
            .btn:disabled { background: #484f58; cursor: not-allowed; transform: none; box-shadow: none; opacity: 0.7;}

            #status { margin-top: 20px; font-weight: bold; min-height: 24px; }
            .loader {
                border: 3px solid #30363d;
                border-top: 3px solid #58a6ff;
                border-radius: 50%;
                width: 24px;
                height: 24px;
                animation: spin 1s linear infinite;
                display: none;
                margin: 15px auto;
            }
            @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 GulseScan Hub</h1>
            <p>ROS 2 Haritalama & Navigasyon Paneli</p>
            
            <button class="btn" onclick="resetMap()" id="resetBtn">
                HARİTAYI SIFIRLA ♻️
            </button>
            
            <div class="loader" id="loader"></div>
            <div id="status"></div>
        </div>

        <script>
            async function resetMap() {
                const btn = document.getElementById('resetBtn');
                const status = document.getElementById('status');
                const loader = document.getElementById('loader');

                // UI Kilitle
                btn.disabled = true;
                btn.innerText = "SIFIRLANIYOR...";
                loader.style.display = "block";
                status.innerText = "";

                try {
                    // API'ye İstek At
                    const response = await fetch('/reset-map', { method: 'POST' });
                    const data = await response.json();
                    
                    if(data.status === 'success') {
                        status.style.color = '#3fb950';
                        status.innerText = "✨ " + data.message;
                    } else {
                        status.style.color = '#f85149';
                        status.innerText = "⚠️ " + data.message;
                    }
                } catch (error) {
                    status.style.color = '#f85149';
                    status.innerText = "❌ Sunucuya ulaşılamadı!";
                }

                // UI Eski Haline Getir
                loader.style.display = "none";
                setTimeout(() => {
                    btn.disabled = false;
                    btn.innerText = "HARİTAYI SIFIRLA ♻️";
                }, 2500);
            }
        </script>
    </body>
    </html>
    """

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)