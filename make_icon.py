"""プラズマCAM用アイコン生成 - 稲妻バージョン"""
from PIL import Image, ImageDraw, ImageFilter
import math
import random

SIZE = 256
random.seed(42)


def lerp(a, b, t):
    return a + (b - a) * t


def zigzag_bolt(cx, cy, angle, r_start, r_end, jitter=12, steps=7):
    """ジグザグ稲妻パスを生成"""
    points = []
    for i in range(steps + 1):
        t = i / steps
        r = lerp(r_start, r_end, t)
        # 垂直方向にランダムなずれ
        perp = angle + math.pi / 2
        offset = random.uniform(-jitter, jitter) * math.sin(t * math.pi)
        x = cx + r * math.cos(angle) + offset * math.cos(perp)
        y = cy + r * math.sin(angle) + offset * math.sin(perp)
        points.append((x, y))
    return points


def draw_glow_line(draw, points, color, width, glow_layers=4):
    """グロー付きライン描画"""
    for layer in range(glow_layers, 0, -1):
        alpha = int(color[3] * (layer / glow_layers) * 0.5)
        w = width + layer * 3
        c = (color[0], color[1], color[2], alpha)
        for i in range(len(points) - 1):
            draw.line([points[i], points[i+1]], fill=c, width=w)
    # コア（白に近い明るいライン）
    for i in range(len(points) - 1):
        draw.line([points[i], points[i+1]], fill=color, width=width)


def make_icon():
    # 高解像度で描いてからリサイズ（アンチエイリアス代わり）
    WORK = SIZE * 2
    wcx, wcy = WORK // 2, WORK // 2

    img = Image.new("RGBA", (WORK, WORK), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # ── 背景: 宇宙っぽい濃紺の円 ──
    for r in range(WORK//2 - 10, 0, -1):
        frac = r / (WORK // 2)
        b_val = int(lerp(60, 10, frac))
        g_val = int(lerp(20, 5, frac))
        draw.ellipse([wcx-r, wcy-r, wcx+r, wcy+r],
                     fill=(g_val, g_val, b_val, 255))

    # ── 外周グローリング ──
    for i in range(12, 0, -1):
        rr = WORK // 2 - 14 + i
        alpha = int(180 * (i / 12))
        draw.ellipse([wcx-rr, wcy-rr, wcx+rr, wcy+rr],
                     outline=(40, 120 + i*8, 255, alpha), width=4)

    # ── メイン稲妻（6本、放射状） ──
    bolt_angles = [math.pi * i / 3 for i in range(6)]
    for angle in bolt_angles:
        pts = zigzag_bolt(wcx, wcy, angle, 28, WORK//2 - 22, jitter=18, steps=8)
        # グロー外層（水色）
        draw_glow_line(draw, pts, (80, 200, 255, 220), width=4, glow_layers=5)
        # コア（白）
        draw_glow_line(draw, pts, (220, 240, 255, 255), width=2, glow_layers=2)

    # ── サブ稲妻（6本、細め） ──
    sub_angles = [math.pi * i / 3 + math.pi / 6 for i in range(6)]
    for angle in sub_angles:
        pts = zigzag_bolt(wcx, wcy, angle, 28, WORK//2 - 40, jitter=10, steps=6)
        draw_glow_line(draw, pts, (60, 160, 255, 160), width=3, glow_layers=3)
        draw_glow_line(draw, pts, (180, 220, 255, 200), width=1, glow_layers=1)

    # ── 中心グロー（爆発っぽく） ──
    glow_colors = [
        (255, 255, 255, 255),
        (200, 230, 255, 220),
        (80,  180, 255, 160),
        (30,  100, 255, 100),
        (10,   50, 200,  60),
    ]
    radii = [12, 24, 42, 64, 90]
    for col, rad in zip(glow_colors, radii):
        draw.ellipse([wcx-rad, wcy-rad, wcx+rad, wcy+rad], fill=col)

    # ── ブラーでグローを柔らかく ──
    img = img.filter(ImageFilter.GaussianBlur(radius=2))

    # 再描画: グロー後にシャープなコアを乗せる
    draw2 = ImageDraw.Draw(img)

    # 稲妻コアを再描画（くっきり）
    random.seed(42)
    for angle in bolt_angles:
        pts = zigzag_bolt(wcx, wcy, angle, 28, WORK//2 - 22, jitter=18, steps=8)
        for i in range(len(pts)-1):
            draw2.line([pts[i], pts[i+1]], fill=(240, 250, 255, 255), width=3)

    # 中心白点
    draw2.ellipse([wcx-10, wcy-10, wcx+10, wcy+10], fill=(255, 255, 255, 255))

    # 外枠
    rr = WORK // 2 - 10
    draw2.ellipse([wcx-rr, wcy-rr, wcx+rr, wcy+rr],
                  outline=(0, 200, 255, 230), width=6)

    # ── リサイズして保存 ──
    sizes = [16, 32, 48, 64, 128, 256]
    base = img.resize((SIZE, SIZE), Image.LANCZOS)
    icons = [base.resize((s, s), Image.LANCZOS) for s in sizes]
    icons[0].save(
        "plasma_cam.ico",
        format="ICO",
        sizes=[(s, s) for s in sizes],
        append_images=icons[1:]
    )
    print("plasma_cam.ico を生成しました（稲妻バージョン）")


if __name__ == "__main__":
    make_icon()
