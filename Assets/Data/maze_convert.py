# save as Assets/Data/maze_convert.py
import json
from pathlib import Path

# Input / Output
src = Path("maze.json")     # 输入 JSON 文件
dst = Path("maze.txt")      # 输出 TXT 文件

# Load JSON
with src.open("r", encoding="utf-8") as f:
    data = json.load(f)

cells = data.get("cells", [])

# Write TXT: flip Z for right-hand coord system
with dst.open("w", encoding="utf-8") as f:
    for c in cells:
        px = c["px"]
        py = c["py"]
        pz = -c["pz"]   # 翻转 Z 轴

        sx = c["sx"]
        sy = c["sy"]
        sz = c["sz"]

        f.write(f"{px} {py} {pz} {sx} {sy} {sz}\n")

print(f"Converted {len(cells)} cells -> {dst}")
