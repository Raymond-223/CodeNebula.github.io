#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate figures for reinforcement-learning/index.md using Pillow."""
import os
from PIL import Image, ImageDraw, ImageFont

FONT_DIR = "/mnt/c/Windows/Fonts"
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "docs", "reinforcement-learning", "assets")
os.makedirs(OUT_DIR, exist_ok=True)

def font(size, bold=False):
    if bold:
        return ImageFont.truetype(os.path.join(FONT_DIR, "msyhbd.ttc"), size)
    return ImageFont.truetype(os.path.join(FONT_DIR, "msyh.ttc"), size)

# Color palette (flat, professional)
C_BG      = (255, 255, 255)
C_PRIMARY = (37, 99, 235)    # blue-600
C_SECOND  = (14, 165, 233)   # sky-500
C_GREEN   = (22, 163, 74)    # green-600
C_ORANGE  = (234, 88, 12)    # orange-600
C_PURPLE  = (124, 58, 237)   # violet-600
C_RED     = (220, 38, 38)    # red-600
C_DARK    = (30, 41, 59)     # slate-800
C_GRAY    = (100, 116, 139)  # slate-500
C_LIGHT   = (241, 245, 249)  # slate-100
C_BORDER  = (203, 213, 225)  # slate-300

def rrect(draw, box, radius, fill, outline=None, width=2):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)

def center_text(draw, box, text, fnt, fill):
    x0, y0, x1, y1 = box
    bbox = draw.textbbox((0, 0), text, font=fnt)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    draw.text(((x0 + x1 - w) / 2 - bbox[0], (y0 + y1 - h) / 2 - bbox[1]), text,
              font=fnt, fill=fill)

def arrow(draw, p1, p2, color, width=4, head=14):
    x1, y1 = p1
    x2, y2 = p2
    draw.line([p1, p2], fill=color, width=width)
    import math
    ang = math.atan2(y2 - y1, x2 - x1)
    for da in (math.pi - 0.35, math.pi + 0.35):
        draw.line([(x2, y2),
                   (x2 + head * math.cos(ang + da), y2 + head * math.sin(ang + da))],
                  fill=color, width=width)

# ============================================================
# Figure 1: Agent-Environment interaction loop
# ============================================================
W, H = 1280, 640
img = Image.new("RGB", (W, H), C_BG)
d = ImageDraw.Draw(img)
f_title = font(30, bold=True)
f_box   = font(24, bold=True)
f_txt   = font(19)
f_small = font(16)

d.text((40, 28), "强化学习的闭环：Agent 与 Environment 的交互", font=f_title, fill=C_DARK)

# Agent box (left)
agent_box = (60, 150, 560, 490)
rrect(d, agent_box, 18, C_LIGHT, C_PRIMARY, 3)
center_text(d, (60, 160, 560, 215), "Agent（智能体）", f_box, C_PRIMARY)

agent_items = [
    ("策略 π(a|s)", "根据状态选择动作", C_GREEN),
    ("价值函数 V(s) / Q(s,a)", "评估状态与动作的好坏", C_SECOND),
    ("学习器", "更新策略与价值（试错）", C_ORANGE),
]
y = 240
for name, desc, col in agent_items:
    rrect(d, (95, y, 525, y + 72), 10, (255, 255, 255), col, 2)
    center_text(d, (95, y, 525, y + 38), name, f_txt, col)
    center_text(d, (95, y + 38, 525, y + 72), desc, f_small, C_GRAY)
    y += 86

# Environment box (right)
env_box = (720, 150, 1220, 490)
rrect(d, env_box, 18, C_LIGHT, C_GREEN, 3)
center_text(d, (720, 160, 1220, 215), "Environment（环境）", f_box, C_GREEN)

env_items = [
    ("状态转移 P(s'|s,a)", "环境如何响应动作", C_ORANGE),
    ("奖励函数 R(s,a)", "行为的即时反馈", C_RED),
    ("观测 o", "智能体感知到的信息", C_PURPLE),
]
y = 240
for name, desc, col in env_items:
    rrect(d, (755, y, 1185, y + 72), 10, (255, 255, 255), col, 2)
    center_text(d, (755, y, 1185, y + 38), name, f_txt, col)
    center_text(d, (755, y + 38, 1185, y + 72), desc, f_small, C_GRAY)
    y += 86

# Arrows
arrow(d, (560, 250), (720, 250), C_PRIMARY, 5)
d.text((600, 215), "动作 a", font=f_txt, fill=C_PRIMARY)
d.text((600, 258), "（改变环境）", font=f_small, fill=C_GRAY)

arrow(d, (720, 420), (560, 420), C_GREEN, 5)
d.text((585, 430), "奖励 r + 新状态 s'", font=f_txt, fill=C_GREEN)

# Loop caption
rrect(d, (240, 540, 1040, 600), 12, (254, 249, 195), C_ORANGE, 2)
center_text(d, (240, 540, 1040, 600),
            "目标：最大化累积奖励  G = r₁ + γr₂ + γ²r₃ + …   （γ 为折扣因子）",
            f_txt, C_DARK)

img.save(os.path.join(OUT_DIR, "agent-environment-loop.png"))
print("saved agent-environment-loop.png")

# ============================================================
# Figure 2: RL milestone timeline
# ============================================================
W, H = 1500, 620
img = Image.new("RGB", (W, H), C_BG)
d = ImageDraw.Draw(img)
f_title = font(30, bold=True)
f_year  = font(22, bold=True)
f_txt   = font(17)
f_small = font(14)

d.text((40, 30), "强化学习经典里程碑", font=f_title, fill=C_DARK)

# Timeline axis
axis_y = 320
d.line([(80, axis_y), (1440, axis_y)], fill=C_BORDER, width=4)

milestones = [
    (150,  "1950s", "试错学习与\nMDP 理论奠基", C_DARK),
    (360,  "1989",  "Q-Learning\n诞生", C_PURPLE),
    (570,  "1992",  "TD-Gammon\n击败人类", C_SECOND),
    (780,  "2013",  "DQN\n玩转 Atari", C_PRIMARY),
    (990,  "2016",  "AlphaGo\n战胜李世石", C_GREEN),
    (1200, "2017",  "PPO\n成为默认算法", C_ORANGE),
    (1380, "2022",  "ChatGPT\nRLHF 对齐", C_RED),
]

for x, year, label, col in milestones:
    # dot
    d.ellipse([x - 8, axis_y - 8, x + 8, axis_y + 8], fill=col, outline=C_BG, width=3)
    # year above
    yb = d.textbbox((0, 0), year, font=f_year)
    d.text((x - (yb[2] - yb[0]) / 2, axis_y - 55), year, font=f_year, fill=col)
    # label below
    lines = label.split("\n")
    yy = axis_y + 20
    for ln in lines:
        lb = d.textbbox((0, 0), ln, font=f_txt)
        d.text((x - (lb[2] - lb[0]) / 2, yy), ln, font=f_txt, fill=C_DARK)
        yy += 28

img.save(os.path.join(OUT_DIR, "rl-milestones.png"))
print("saved rl-milestones.png")

# ============================================================
# Figure 3: Algorithm family tree
# ============================================================
W, H = 1500, 780
img = Image.new("RGB", (W, H), C_BG)
d = ImageDraw.Draw(img)
f_title = font(30, bold=True)
f_head  = font(22, bold=True)
f_txt   = font(18)
f_small = font(15)

d.text((40, 28), "深度强化学习算法谱系", font=f_title, fill=C_DARK)

# Roots (top)
roots = [
    (200, "Model-free（免模型）", "直接从交互经验学习，\n不需要环境模型", C_PRIMARY),
    (750, "Model-based（基于模型）", "学习环境模型后用\n规划/想象加速学习", C_GREEN),
    (1300, "模仿学习 / 离线学习", "从专家演示或固定数据集\n学习，不与环境交互", C_ORANGE),
]
for x, name, desc, col in roots:
    rrect(d, (x - 170, 100, x + 170, 230), 14, C_LIGHT, col, 3)
    center_text(d, (x - 170, 108, x + 170, 155), name, f_head, col)
    y = 165
    for ln in desc.split("\n"):
        lb = d.textbbox((0, 0), ln, font=f_small)
        d.text((x - (lb[2] - lb[0]) / 2, y), ln, font=f_small, fill=C_GRAY)
        y += 24

# Level 2: value vs policy
lv2 = [
    (110, "基于价值 Value-based", "DQN / Double DQN\nDueling DQN / Rainbow", C_PURPLE),
    (420, "基于策略 Policy-based", "REINFORCE\nTRPO / PPO", C_SECOND),
    (750, "Actor-Critic（演员-评论家）", "A2C / A3C\nDDPG / TD3 / SAC", C_PRIMARY),
    (1100, "多智能体 MARL", "MADDPG / QMIX\nMAPPO", C_RED),
    (1380, "序列决策", "Decision Transformer\n决策即序列建模", C_ORANGE),
]
for x, name, desc, col in lv2:
    rrect(d, (x - 130, 320, x + 130, 450), 12, C_LIGHT, col, 3)
    center_text(d, (x - 130, 328, x + 130, 372), name, f_txt, col)
    y = 385
    for ln in desc.split("\n"):
        lb = d.textbbox((0, 0), ln, font=f_small)
        d.text((x - (lb[2] - lb[0]) / 2, y), ln, font=f_small, fill=C_DARK)
        y += 26

# Connectors: roots -> lv2 (model-free -> value/policy/AC/MARL/DT)
for x in (110, 420, 750, 1100, 1380):
    d.line([(x, 230), (x, 320)], fill=C_BORDER, width=3)

# Level 3: RLHF & applications
rrect(d, (370, 540, 1130, 670), 14, C_LIGHT, C_RED, 3)
center_text(d, (370, 550, 1130, 596), "LLM 对齐：RLHF 三阶段", f_head, C_RED)
steps = ["SFT 监督微调", "奖励模型 RM", "PPO 策略优化"]
x = 430
for i, s in enumerate(steps):
    rrect(d, (x, 605, x + 170, 655), 8, (255, 255, 255), C_RED, 2)
    center_text(d, (x, 605, x + 170, 655), s, f_txt, C_DARK)
    if i < 2:
        arrow(d, (x + 170, 630), (x + 200, 630), C_RED, 3, head=10)
    x += 230
d.line([(750, 450), (750, 540)], fill=C_BORDER, width=3)

# Bottom caption
rrect(d, (330, 700, 1170, 750), 10, (254, 249, 195), C_ORANGE, 2)
center_text(d, (330, 700, 1170, 750),
            "学习主线：表格型 RL → 深度 RL（DQN/PPO/SAC）→ 大模型对齐（RLHF）",
            f_txt, C_DARK)

img.save(os.path.join(OUT_DIR, "rl-family-tree.png"))
print("saved rl-family-tree.png")

print("ALL DONE ->", OUT_DIR)
