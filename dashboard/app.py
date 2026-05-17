#!/usr/bin/env python3
"""
Dashboard web para Mini CNC.
Flujo:  Browser → Flask → HTTP(POST) → ESP32 → UART → STM32
- POST /upload  → reenvia .nc a ESP32 (http://<IP>/upload)
- POST /start   → reenvia start a ESP32 (http://<IP>/start)
- GET  /status  → SSE con progreso en tiempo real
"""

import time
import json
import threading
import requests
from flask import Flask, request, jsonify, render_template, Response

# ── Configuracion ───────────────────────────────────────────────────────────
ESP32_URL = "http://192.168.4.1"   # IP del ESP32 (AP mode por defecto)

app = Flask(__name__)

# ── Estado global ───────────────────────────────────────────────────────────
state = {
    "running":    False,
    "total":      0,
    "sent":       0,
    "current":    "",
    "response":   "",
    "log":        [],
}

state_lock = threading.Lock()

# ── Helpers ─────────────────────────────────────────────────────────────────
def add_log(msg):
    with state_lock:
        state["log"].append(msg)
        if len(state["log"]) > 200:
            state["log"] = state["log"][-200:]

# ── Rutas Flask ─────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/upload", methods=["POST"])
def upload():
    """Recibe .nc del navegador y lo reenvia al ESP32 por HTTP"""
    if "file" not in request.files:
        return jsonify({"status": "error", "msg": "No se envio archivo"}), 400

    f = request.files["file"]
    if f.filename == "":
        return jsonify({"status": "error", "msg": "Nombre vacio"}), 400

    content = f.read()
    lines = [l.strip() for l in content.decode("ascii", errors="ignore").splitlines()
             if l.strip() and not l.strip().startswith(";")
             and not l.strip().startswith("(")]

    if not lines:
        return jsonify({"status": "error", "msg": "Archivo vacio"}), 400

    try:
        # Enviar archivo raw al ESP32 (POST /upload con body = contenido .nc)
        r = requests.post(f"{ESP32_URL}/upload", data=content,
                          headers={"Content-Type": "application/octet-stream"},
                          timeout=30)
        if r.status_code != 200:
            add_log(f"[ERROR] ESP32 /upload respondio {r.status_code}")
            return jsonify({"status": "error", "msg": f"ESP32 error: {r.status_code}"}), 500

        app.config["GCODE_LINES"] = lines
        add_log(f"[OK] '{f.filename}' enviado al ESP32: {len(lines)} lineas")
        return jsonify({"status": "ok", "lines": len(lines)})

    except requests.ConnectionError:
        add_log(f"[ERROR] No se pudo conectar al ESP32 en {ESP32_URL}")
        return jsonify({"status": "error", "msg": f"ESP32 inaccesible en {ESP32_URL}"}), 500
    except Exception as e:
        add_log(f"[ERROR] {e}")
        return jsonify({"status": "error", "msg": str(e)}), 500

@app.route("/start", methods=["POST"])
def start():
    """Envia POST /start al ESP32 para iniciar el envio UART"""
    lines = app.config.get("GCODE_LINES", [])
    if not lines:
        return jsonify({"status": "error", "msg": "No hay archivo cargado"}), 400

    try:
        r = requests.post(f"{ESP32_URL}/start", timeout=5)
        if r.status_code != 200:
            return jsonify({"status": "error", "msg": f"ESP32 error: {r.status_code}"}), 500

        with state_lock:
            state["running"] = True
            state["total"]   = len(lines)
            state["sent"]    = 0

        add_log(f"[INFO] START enviado al ESP32. {len(lines)} lineas pendientes.")
        return jsonify({"status": "ok"})

    except requests.ConnectionError:
        return jsonify({"status": "error", "msg": f"ESP32 inaccesible"}), 500

@app.route("/status")
def status_route():
    """SSE: envia progreso simulado al navegador"""
    def generate():
        last_sent = 0
        while True:
            with state_lock:
                data = {
                    "running":   state["running"],
                    "sent":      state["sent"],
                    "total":     state["total"],
                }
            if state["sent"] > last_sent:
                yield f"data: {json.dumps(data)}\n\n"
                last_sent = state["sent"]
            if not state["running"] and last_sent >= state["total"]:
                yield f"data: {json.dumps(data)}\n\n"
                break
            time.sleep(0.5)
    return Response(generate(), mimetype="text/event-stream")

@app.route("/log")
def log_route():
    with state_lock:
        return jsonify(state["log"])

# ── Entry point ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"[INFO] Dashboard CNC en http://0.0.0.0:5000")
    print(f"[INFO] ESP32 destino: {ESP32_URL}")
    print("[INFO] Flujo: Browser → Flask → HTTP → ESP32 → UART → STM32")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
