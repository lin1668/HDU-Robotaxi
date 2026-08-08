#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
语音指令 + 视觉识别 → 启动小车
流程：
  1. 语音指令 → LLM 解析 → 圈数配置
  2. 摄像头拍照识别 → 用户确认后写入 alertTarget
  3. 写入圈数配置到 config.json
  4. 启动小车程序
注意：摄像头在第 2 步确认后即释放，确保不占用小车程序资源
"""

import json
import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from visual import VisualLLM, LABEL_DICT, update_alert_target

# 终端颜色
COUT_RED = "\033[91m"
COUT_GREEN = "\033[92m"
COUT_YELLOW = "\033[93m"
COUT_REST = "\033[0m"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.abspath(os.path.join(BASE_DIR, "../res/config.json"))
BUILD_DIR = os.path.join(BASE_DIR, "..", "build")
ICAR_PATH = os.path.join(BUILD_DIR, "icar")



def run_speech_flow():
    """语音指令解析流程，返回 (lapConfig, totalLaps) 或 None"""
    from speech.llm import LLM
    from speech.speech import parsedToLapConfig, normalizeTasks

    def _describe_lap(lap):
        if lap["park"]:
            if lap.get("parkSpot") == 0:
                return "停车场(穿过)"
            return f"停车场(停车位 {lap['parkSpot']})"
        if lap["busy"]:
            pos = "中间" if lap["busyStopPoint"] == 1 else "出口"
            return f"施工区({pos})"
        if lap.get("fork") or lap.get("yfork"):
            side = "左侧" if lap.get("yforkLeft") else "右侧"
            return f"岔路口({side})"
        return "未知"

    llm = LLM()

    while True:
        instruction = input("\n请输入行驶指令: ").strip()
        if not instruction:
            continue

        print(f"\n原始指令: {instruction}\n")

        result = llm.parseInstruction(instruction)
        if not result:
            print(f"{COUT_RED}解析失败！请检查 API Key 是否有效{COUT_REST}")
            continue

        print(f"LLM 解析结果: {json.dumps(result, ensure_ascii=False)}\n")

        normalizeTasks(result)
        print(f"修正后: {json.dumps(result, ensure_ascii=False)}\n")

        lapConfig = parsedToLapConfig(result)
        totalLaps = len(result["tasks"])
        print("映射为每圈配置:")
        print(json.dumps(lapConfig, ensure_ascii=False, indent=2))
        print(f"  总圈数: {totalLaps}")
        for lapNum in sorted(lapConfig.keys()):
            lap = lapConfig[lapNum]
            info = _describe_lap(lap)
            print(f"  {lapNum} → {info}")

        confirm = input("\n解析结果是否正确？(y/n): ").strip().lower()
        if confirm == "y":
            return lapConfig, totalLaps
        print("已取消，请重新输入指令。")


def run_visual_flow(vllm):
    """摄像头拍照识别循环：拍一张 → 识别 → 确认"""
    import cv2

    print("\n" + "=" * 60)
    print("    摄像头拍照识别 — 按提示确认是否写入配置")
    print("=" * 60)

    camera_device = "/dev/v4l/by-id/usb-XCX-230919-H_PC_Camera_A4-video-index0"
    cap = cv2.VideoCapture(camera_device)
    if not cap.isOpened():
        print(f"{COUT_RED}[摄像头] 无法打开摄像头{COUT_REST}")
        return None
    time.sleep(0.5)

    while True:
        # 清除摄像头驱动内部的帧缓冲区，确保读到最新画面
        for _ in range(5):
            cap.grab()
        ret, frame = cap.read()
        if not ret:
            print(f"{COUT_RED}[摄像头] 读取画面失败，尝试重新拍摄...{COUT_REST}")
            continue

        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            temp_path = tmp.name
        cv2.imwrite(temp_path, frame)

        print(f"\n{COUT_YELLOW}[摄像头] 正在识别...{COUT_REST}")
        label = vllm.recognize(temp_path)
        os.remove(temp_path)

        if label:
            name = LABEL_DICT.get(label, label)
            print(f"{COUT_GREEN}[摄像头] 识别结果: {label} ({name}){COUT_REST}")
        else:
            print(f"{COUT_RED}[摄像头] 识别失败{COUT_REST}")

        confirm = input("\n是否将此识别结果写入配置文件？(y/n): ").strip().lower()
        if confirm == "y":
            if label:
                update_alert_target(label, CONFIG_PATH)
                print(f"{COUT_GREEN}已写入配置{COUT_REST}")
            break
        print("已取消，重新拍摄识别...")

    cap.release()
    print(f"{COUT_GREEN}[摄像头] 已释放{COUT_REST}")
    return label


def start_car_program():
    """启动小车程序"""
    if not os.path.exists(ICAR_PATH):
        print(f"{COUT_RED}\n未找到小车程序: {ICAR_PATH}{COUT_REST}")
        return False

    boot_needed = input("\n是否需要先启动boot？(y/n): ").strip().lower()
    if boot_needed == "y":
        boot_cmd = [
            "gnome-terminal", "--", "export", "DISPLAY=:0.0",
            "--working-directory", BUILD_DIR, "--", "./boot"
        ]
        subprocess.Popen(boot_cmd, cwd=BUILD_DIR)
        print("已启动 boot，等待 3 秒...")
        time.sleep(3)

    print(f"\n正在启动小车程序: {ICAR_PATH}")
    log_dir = os.path.join(BUILD_DIR, "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "icar.log")
    log_file = open(log_path, "w")
    subprocess.Popen(
        ["./icar"],
        cwd=BUILD_DIR,
        stdout=log_file,
        stderr=log_file,
    )
    print(f"{COUT_GREEN}小车程序已启动，日志: {log_path}{COUT_REST}")
    return True


def main():
    print("=" * 60)
    print("    智能小车启动程序 — 语音 + 视觉")
    print("=" * 60)

    # ========== 1. 语音指令解析 ==========
    print(f"\n{COUT_YELLOW}>>> 第一步：语音指令解析{COUT_REST}")
    result = run_speech_flow()
    if result is None:
        sys.exit(1)
    lapConfig, totalLaps = result

    # ========== 2. 摄像头视觉识别 ==========
    print(f"\n{COUT_YELLOW}>>> 第二步：拍照识别场景{COUT_REST}")
    vllm = VisualLLM()
    label = run_visual_flow(vllm)
    if label:
        print(f"最终识别标签: {label}")

    # ========== 3. 写入圈数配置 ==========
    print(f"\n{COUT_YELLOW}>>> 第三步：写入圈数配置{COUT_REST}")
    from speech.speech import updateConfigJson
    updateConfigJson(lapConfig, totalLaps, CONFIG_PATH)

    # ========== 4. 启动小车 ==========
    print(f"\n{COUT_YELLOW}>>> 第四步：启动小车程序{COUT_REST}")
    start_car_program()


if __name__ == "__main__":
    main()
