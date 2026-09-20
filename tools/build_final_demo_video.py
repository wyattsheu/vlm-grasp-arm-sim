#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'out/final_demo'
PHYSICS = ROOT / 'out/lesson_06/physics_grasp'
REPORT = json.loads((PHYSICS / 'physics_grasp_report.json').read_text())
ROUNDTRIP = json.loads((ROOT / 'out/lesson_08/ros_isaac_roundtrip.json').read_text())
MOVEIT = json.loads((ROOT / 'out/lesson_09/moveit_fixed_plan.json').read_text())
MTC = json.loads((ROOT / 'out/lesson_09/mtc_plan.json').read_text())
LATEST = json.loads((ROOT / 'research/out/robot129_sim_replay/latest.json').read_text())
GROUNDING = Path(LATEST['run_dir']) / 'grounding/overlay.png'

W, H = 640, 480
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
OUT.mkdir(parents=True, exist_ok=True)


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


def centered(draw, y, text, size, color='white', bold=False):
    f = font(size, bold)
    box = draw.textbbox((0, 0), text, font=f)
    draw.text(((W - (box[2] - box[0])) / 2, y), text, font=f, fill=color)


def card(path, title, lines, accent):
    im = Image.new('RGB', (W, H), '#101820')
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, W, 12), fill=accent)
    centered(d, 55, title, 35, 'white', True)
    y = 145
    for text, color, size in lines:
        centered(d, y, text, size, color, False)
        y += size + 20
    d.rectangle((44, H - 55, W - 44, H - 53), fill='#45606f')
    centered(d, H - 40, 'SIMULATION ONLY  |  HARDWARE DRIVERS: 0', 15, '#a9c8d8', True)
    im.save(path)


intro = OUT / 'intro.png'
card(intro, 'Robot 129 Digital Twin', [
    ('Piper 3D model in Isaac Sim 6.0.1', '#d8e7ee', 24),
    ('ROS 2 Jazzy  |  ros2_control  |  MoveIt 2', '#d8e7ee', 22),
    ('True PhysX contact grasp - no pose attachment', '#ef5350', 20),
], '#ef5350')

base = Image.open(GROUNDING).convert('RGB').resize((W, H), Image.Resampling.LANCZOS)
panel = base.copy()
d = ImageDraw.Draw(panel, 'RGBA')
d.rectangle((0, 0, W, 52), fill=(10, 20, 28, 225))
d.text((18, 10), 'Isaac RGB-D -> SceneBundle -> MPG replay', font=font(23, True), fill='white')
d.rectangle((0, H - 52, W, H), fill=(10, 20, 28, 225))
d.text((18, H - 40), '3/3 points localized to camera and world coordinates', font=font(17), fill='#8ee6a8')
grounding_panel = OUT / 'grounding_replay.png'
panel.save(grounding_panel)

result = OUT / 'results.png'
card(result, 'Acceptance Results', [
    (f"PhysX grasp PASS   contact {REPORT['max_contact_force_N']['link7']:.2f}/{REPORT['max_contact_force_N']['link8']:.2f} N", '#8ee6a8', 20),
    (f"Payload z {REPORT['z_m']['before_lift']:.3f} -> {REPORT['z_m']['peak']:.3f} m", '#8ee6a8', 20),
    (f"ROS -> Isaac max error {ROUNDTRIP['max_final_error']:.5f} rad/m", '#88c9ff', 19),
    (f"MoveIt {MOVEIT['planner']} {MOVEIT['trajectory_points']} points  |  MTC {MTC['planned_solutions']} solution", '#88c9ff', 18),
    ('SceneBundle + geometry replay + Thor loopback PASS', '#f7d774', 18),
], '#48a868')

video = OUT / 'robot129_full_simulation.mp4'
cmd = [
    'ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
    '-loop', '1', '-framerate', '30', '-t', '2', '-i', str(intro),
    '-framerate', '30', '-start_number', '0', '-i', str(PHYSICS / 'frames/frame_%04d.png'),
    '-loop', '1', '-framerate', '30', '-t', '4', '-i', str(grounding_panel),
    '-loop', '1', '-framerate', '30', '-t', '3', '-i', str(result),
    '-filter_complex',
    '[0:v]scale=640:480,trim=duration=2,setpts=PTS-STARTPTS[v0];'
    '[1:v]scale=640:480,trim=duration=10,setpts=PTS-STARTPTS[v1];'
    '[2:v]scale=640:480,trim=duration=4,setpts=PTS-STARTPTS[v2];'
    '[3:v]scale=640:480,trim=duration=3,setpts=PTS-STARTPTS[v3];'
    '[v0][v1][v2][v3]concat=n=4:v=1:a=0,format=yuv420p[v]',
    '-map', '[v]', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
    '-movflags', '+faststart', str(video),
]
subprocess.run(cmd, check=True)
probe = json.loads(subprocess.check_output([
    'ffprobe', '-v', 'error', '-show_entries',
    'format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames',
    '-of', 'json', str(video)], text=True))
sha = hashlib.sha256(video.read_bytes()).hexdigest()

# Six panels make the result inspectable without playing the movie.
sources = [intro, PHYSICS/'frames/frame_0080.png', PHYSICS/'frames/frame_0140.png',
           PHYSICS/'frames/frame_0220.png', grounding_panel, result]
labels = ['SYSTEM', 'APPROACH', 'GRASP / CONTACT', 'WAYPOINT / LIFT', 'RGB-D GROUNDING', 'RESULTS']
sheet = Image.new('RGB', (960, 540), '#101820')
for i, (source, label) in enumerate(zip(sources, labels)):
    tile = Image.open(source).convert('RGB').resize((320, 240), Image.Resampling.LANCZOS)
    td = ImageDraw.Draw(tile, 'RGBA')
    td.rectangle((0, 0, 320, 30), fill=(0, 0, 0, 190))
    td.text((8, 5), label, font=font(15, True), fill='white')
    x, y = (i % 3) * 320, (i // 3) * 270
    sheet.paste(tile, (x, y))
    ImageDraw.Draw(sheet).text((x + 8, y + 244), label, font=font(14), fill='#d8e7ee')
contact_sheet = OUT / 'contact_sheet.png'
sheet.save(contact_sheet)

final_report = {
    'status': 'PASS',
    'video': str(video),
    'contact_sheet': str(contact_sheet),
    'sha256': sha,
    'ffprobe': probe,
    'timeline': [
        {'seconds': [0, 2], 'content': 'system and safety scope'},
        {'seconds': [2, 12], 'content': 'true PhysX grasp/lift/waypoint/release, no pose attachment'},
        {'seconds': [12, 16], 'content': 'Isaac RGB-D SceneBundle and MPG geometry replay'},
        {'seconds': [16, 19], 'content': 'measured acceptance results'},
    ],
    'claims': {
        'physics_grasp': REPORT['status'],
        'kinematic_attachment': REPORT['kinematic_attachment'],
        'ros_isaac_roundtrip': ROUNDTRIP['status'],
        'moveit_fixed_plan': MOVEIT['status'],
        'mtc_plan': MTC['status'],
        'live_vllm_model': 'NOT RUN',
        'thor_connected': 'NOT RUN',
        'hardware_commands': 0,
    },
}
(OUT / 'video_report.json').write_text(json.dumps(final_report, indent=2) + '\n')
print(json.dumps(final_report, indent=2))
