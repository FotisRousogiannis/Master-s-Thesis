"""Draws the Arm Studio icon (a stylised 3-link arm) and writes PNG + ICO.

    python3 software/launcher/icons/make_icons.py
"""

from pathlib import Path

from PIL import Image, ImageDraw

SIZE = 1024  # drawn large, scaled down for smooth edges
OUT = Path(__file__).resolve().parent
BG, ARM, JOINT, ACCENT = "#1f2a44", "#f08a24", "#e8edf5", "#4fc3f7"


def draw() -> Image.Image:
    img = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([40, 40, SIZE - 40, SIZE - 40], radius=200, fill=BG)
    # base
    d.rounded_rectangle([250, 780, 560, 860], radius=30, fill=JOINT)
    # links: base -> shoulder -> elbow -> wrist
    points = [(405, 780), (405, 560), (700, 330), (790, 500)]
    for a, b in zip(points, points[1:]):
        d.line([a, b], fill=ARM, width=110)
    for x, y in points[1:]:
        d.ellipse([x - 70, y - 70, x + 70, y + 70], fill=JOINT)
        d.ellipse([x - 30, y - 30, x + 30, y + 30], fill=BG)
    # gripper
    x, y = points[-1]
    d.line([(x, y + 60), (x - 40, y + 170)], fill=ACCENT, width=45)
    d.line([(x, y + 60), (x + 60, y + 160)], fill=ACCENT, width=45)
    return img


def main():
    icon = draw()
    icon.resize((256, 256), Image.LANCZOS).save(OUT / "arm_studio.png")
    icon.resize((256, 256), Image.LANCZOS).save(
        OUT / "arm_studio.ico", sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]
    )


if __name__ == "__main__":
    main()
