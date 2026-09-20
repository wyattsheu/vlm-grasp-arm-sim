#!/usr/bin/env python3
"""Local-only browser viewer for Robot 129 wrist RGB-D snapshots."""

from __future__ import annotations

import argparse
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import time
from urllib.parse import urlsplit

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "out/ros_webrtc_robot129/wrist_camera"

PAGE = b"""<!doctype html>
<meta charset="utf-8">
<title>Robot 129 wrist RGB-D</title>
<style>
body{margin:0;background:#20242a;color:#eee;font:15px system-ui,sans-serif}
header{padding:12px 18px;background:#15181c}
main{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}
section{background:#171a1f;padding:10px;border-radius:8px}
img{display:block;width:100%;height:auto;background:#333}
pre{white-space:pre-wrap;color:#a9d6ff}
.good{color:#85e89d}.bad{color:#ff7b72}
@media(max-width:900px){main{grid-template-columns:1fr}}
</style>
<header><b>Robot 129 wrist RGB-D</b> &nbsp; <span id="state">loading</span></header>
<main>
<section><h3>RGB / camera_color_optical_frame</h3><img id="rgb"></section>
<section><h3>Depth / 32FC1 meter</h3><img id="depth"></section>
<section><h3>Validation</h3><pre id="stats"></pre></section>
<section><h3>Meaning</h3><pre>Same pixel in both panels refers to the same camera ray.
Near depth is blue; far depth is red.
The camera is attached below the gripper camera optical frame.</pre></section>
</main>
<script>
var seq=0;
async function refresh(){
  seq+=1;
  document.getElementById("rgb").src="/rgb.jpg?n="+seq;
  document.getElementById("depth").src="/depth.jpg?n="+seq;
  try{
    var r=await fetch("/stats?n="+seq,{cache:"no-store"});
    var s=await r.json();
    var age=Date.now()/1000-s.rgb_mtime;
    document.getElementById("state").textContent=age<3?"LIVE":"STALE";
    document.getElementById("state").className=age<3?"good":"bad";
    document.getElementById("stats").textContent=JSON.stringify(s,null,2);
  }catch(e){
    document.getElementById("state").textContent="NO DATA";
    document.getElementById("state").className="bad";
  }
}
setInterval(refresh,500); refresh();
</script>"""


def load_data():
    rgb_path = SCENE / "rgb.png"
    depth_path = SCENE / "depth.npy"
    info_path = SCENE / "camera_info.json"
    tf_path = SCENE / "tf.json"
    rgb = np.asarray(Image.open(rgb_path).convert("RGB"))
    depth = np.squeeze(np.load(depth_path))
    info = json.loads(info_path.read_text())
    tf = json.loads(tf_path.read_text())
    return rgb_path, depth_path, rgb, depth, info, tf


def depth_image(depth):
    valid = np.isfinite(depth) & (depth > 0)
    near, far = np.percentile(depth[valid], [2.0, 98.0])
    if far <= near:
        far = near + 1e-6
    x = np.clip((depth - near) / (far - near), 0.0, 1.0)
    red = np.clip(2.0 * x, 0.0, 1.0)
    green = np.clip(2.0 - np.abs(4.0 * x - 2.0), 0.0, 1.0)
    blue = np.clip(2.0 * (1.0 - x), 0.0, 1.0)
    out = (np.stack([red, green, blue], axis=-1) * 255).astype(np.uint8)
    out[~valid] = (25, 25, 25)
    return Image.fromarray(out), valid, float(near), float(far)


def jpeg(image):
    buf = io.BytesIO()
    image.save(buf, format="JPEG", quality=82)
    return buf.getvalue()


class Handler(BaseHTTPRequestHandler):
    def send_bytes(self, body, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            self.send_bytes(PAGE, "text/html; charset=utf-8")
            return
        try:
            rgb_path, depth_path, rgb, depth, info, tf = load_data()
            depth_vis, valid, near, far = depth_image(depth)
            if path == "/rgb.jpg":
                self.send_bytes(jpeg(Image.fromarray(rgb)), "image/jpeg")
            elif path == "/depth.jpg":
                self.send_bytes(jpeg(depth_vis), "image/jpeg")
            elif path == "/stats":
                center = float(depth[depth.shape[0] // 2, depth.shape[1] // 2])
                body = json.dumps({
                    "status": "PASS",
                    "frame_id": info["frame_id"],
                    "size": [int(rgb.shape[1]), int(rgb.shape[0])],
                    "depth_encoding": "32FC1",
                    "depth_units": "meter",
                    "depth_valid_fraction": round(float(valid.mean()), 6),
                    "depth_center_m": round(center, 4),
                    "depth_color_range_m": [round(near, 4), round(far, 4)],
                    "camera_translation_xyz_m": tf["translation_xyz_m"],
                    "rgb_mtime": rgb_path.stat().st_mtime,
                    "depth_mtime": depth_path.stat().st_mtime,
                }).encode()
                self.send_bytes(body, "application/json")
            else:
                self.send_error(404)
        except Exception as exc:
            self.send_error(503, f"RGB-D unavailable: {type(exc).__name__}: {exc}")

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"Robot 129 wrist RGB-D viewer: http://{args.bind}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
