#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
视觉识别 - 百度一见视觉大模型
接收图片，返回场景标签: cone | person | busy | limit | unlimit | park
"""

import json
import base64
import os
import time
from typing import Optional
from PIL import Image

import requests


# ===== 配置 =====
VISUAL_API_URL = ""
VISUAL_API_KEY = ""
# =================

# 终端颜色（兼容 Windows）
COUT_RED = "\033[91m"
COUT_GREEN = "\033[92m"
COUT_YELLOW = "\033[93m"
COUT_REST = "\033[0m"

# 场景标签映射
LABEL_DICT = {
    "cone": "锥桶",
    "person": "行人",
    "busy": "施工区",
    "limit": "限速",
    "unlimit": "解除限速",
    "park": "停车场",
}


class VisualLLM:
    """百度一见视觉大模型 - 场景识别"""

    def __init__(self, api_url: str = None, api_key: str = None):
        self.api_url = api_url or VISUAL_API_URL
        self.api_key = api_key or VISUAL_API_KEY

    def _make_image_meta(self, image_path: str) -> str:
        """读取图片并构造图片元信息 JSON 字符串"""
        with Image.open(image_path) as img:
            width, height = img.size
        with open(image_path, "rb") as f:
            b64_str = base64.b64encode(f.read()).decode("utf-8")
        meta = {
            "imageData": b64_str,
            "imageId": os.path.basename(image_path),
            "sourceId": "1",
            "imageWidth": width,
            "imageHeight": height,
            "timestamp": int(time.time() * 1000),
        }
        return json.dumps(meta, ensure_ascii=False)

    def recognize(self, image_path: str, instruction: str = None, strict_mode: bool = True) -> Optional[str]:
        """
        识别图片，返回场景标签

        Args:
            image_path: 图片文件路径
            instruction: 可选的识别提示词，帮助模型理解识别目标
            strict_mode: 启用二阶段确认，结果为"限速"时再验证一次
        Returns:
            cone | person | busy | limit | unlimit | park
            或 None（识别失败时）
        """
        if not os.path.exists(image_path):
            print(f"{COUT_RED}[VisualLLM] 图片不存在: {image_path}{COUT_REST}")
            return None

        image_meta = self._make_image_meta(image_path)

        # 百度一见 DataSet 格式
        body = {
            "sessionID": "1",
            "inputs": [
                {
                    "name": "input0",
                    "display_name": "输入",
                    "type": "DataSet",
                    "value": "",
                    "schema": [
                        {
                            "name": "image",
                            "displayName": "图片",
                            "type": "Image",
                            "schema": None,
                            "defaultValue": "",
                            "value": image_meta,
                            "valueType": "Image",
                            "description": "输入的图片",
                            "optional": True,
                            "readonly": False,
                            "constraint": None,
                            "visuals": "",
                        }
                    ],
                    "defaultValue": "",
                    "value": "",
                    "valueType": "DataSet",
                    "description": "",
                    "optional": True,
                    "readonly": False,
                    "constraint": None,
                    "visuals": "",
                }
            ],
        }
        # 携带识别指令（部分百度一见技能支持 query 上下文）
        body["query"] = instruction or (
            "识别图中模拟交通场景的标志类别，必须从以下六类中选择唯一结果。\n\n"
            "六类定义：\n"
            "1. 锥桶（cone）：红色或红白色的三角锥形路障\n"
            "2. 行人（person）：紫色或红色的玩具小人\n"
            "3. 施工区（busy）：施工围挡或工程机械\n"
            "4. 限速（limit）：圆形，有红色圆圈边框，白底上有黑色数字【重要：必须有红圈】\n"
            "5. 解除限速（unlimit）：圆形，但没有红色圆圈，有黑圈，有多条黑色斜线贯穿标志【重要：有斜杠、没有红圈】\n"
            "6. 停车场（park）：蓝色底白色P字牌\n\n"
            "【特别注意】限速(limit)和解除限速(unlimit)极易混淆，请严格按以下特征区分：\n"
            "- 看到红色圆圈 → 限速(limit)\n"
            "- 看到黑色斜线/斜杠（没有红圈） → 解除限速(unlimit)\n"
            "- 两者完全不同，不能混淆"
        )

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        try:
            resp = requests.post(self.api_url, json=body, headers=headers, timeout=60)
            if not resp.ok:
                print(f"{COUT_RED}[VisualLLM] HTTP {resp.status_code}: {resp.text[:500]}{COUT_REST}")
                return None
            result = resp.json()
            label = self._parse_result(result)

            # 二阶段确认：结果为"限速"时，再专门问一次区分限速/解除限速
            if strict_mode and label == "limit":
                confirm = self._confirm_limit_unlimit(image_path)
                if confirm == "unlimit":
                    print(f"{COUT_YELLOW}[VisualLLM] 二阶段确认纠正为: unlimit (解除限速){COUT_REST}")
                    label = "unlimit"
                elif confirm == "limit":
                    print(f"{COUT_GREEN}[VisualLLM] 二阶段确认: 确认为 limit (限速){COUT_REST}")

            return label
        except Exception as e:
            print(f"{COUT_RED}[VisualLLM] 请求失败: {e}{COUT_REST}")
            return None

    def _parse_result(self, result: dict) -> Optional[str]:
        """解析 API 返回结果，提取场景标签"""
        outputs = []
        # 兼容两种结构: result.result.outputs 或 顶层 outputs
        inner_result = result.get("result")
        if isinstance(inner_result, dict):
            outputs = inner_result.get("outputs", [])
        if not outputs:
            outputs = result.get("outputs", [])

        raw_text = None
        for out in outputs:
            if not isinstance(out, dict) or out.get("name") != "output":
                continue
            val = out.get("value", "")
            if not isinstance(val, str) or not val:
                continue

            # 尝试解析为 JSON（老格式：{"参数描述": "限速"}）
            try:
                inner = json.loads(val)
                if isinstance(inner, dict):
                    for field in ["参数描述", "description", "识别结果", "标签", "场景", "label", "result", "output"]:
                        if field in inner and isinstance(inner[field], str):
                            raw_text = inner[field]
                            break
                    if not raw_text:
                        raw_text = json.dumps(inner, ensure_ascii=False)
                elif isinstance(inner, str):
                    raw_text = inner
            except json.JSONDecodeError:
                # 纯字符串标签，如 "limit" / "cone"
                raw_text = val
            break

        # 兜底：result 字段直接是字符串
        if not raw_text and isinstance(inner_result, str):
            raw_text = inner_result

        # 最后兜底：递归搜索
        if not raw_text:
            return self._find_label(result)

        return self._match_label(raw_text)

    def _match_label(self, text: str) -> Optional[str]:
        """从文本中匹配场景标签（特殊处理「限速/解除限速」冲突）"""
        text = text.strip()

        # 直接命中英文标签
        if text in LABEL_DICT:
            return text

        # 优先匹配"解除限速"（避免被短词"限速"截胡）
        if "解除限速" in text:
            return "unlimit"

        # 额外匹配变体："限速解除" == "解除限速"
        if "限速解除" in text or "限速已解除" in text or "解除限速标志" in text:
            return "unlimit"

        # 按中文长度降序匹配（长匹配优先）
        for eng, chn in sorted(LABEL_DICT.items(), key=lambda x: -len(x[1])):
            if chn in text:
                return eng

        print(f"{COUT_YELLOW}[VisualLLM] 未能匹配标签，原始文本: {text}{COUT_REST}")
        return None

    def _find_label(self, obj) -> Optional[str]:
        """递归查找字典中是否有匹配的标签值"""
        if isinstance(obj, dict):
            for v in obj.values():
                if isinstance(v, str) and v in LABEL_DICT:
                    return v
                found = self._find_label(v)
                if found:
                    return found
        elif isinstance(obj, list):
            for item in obj:
                found = self._find_label(item)
                if found:
                    return found
        elif isinstance(obj, str):
            try:
                parsed = json.loads(obj)
                return self._find_label(parsed)
            except (json.JSONDecodeError, TypeError):
                pass
        return None

    def _confirm_limit_unlimit(self, image_path: str) -> Optional[str]:
        """二阶段确认：专门区分限速(limit)和解除限速(unlimit)"""
        image_meta = self._make_image_meta(image_path)
        body = {
            "sessionID": "2",
            "inputs": [
                {
                    "name": "input0",
                    "display_name": "输入",
                    "type": "DataSet",
                    "value": "",
                    "schema": [
                        {
                            "name": "image",
                            "displayName": "图片",
                            "type": "Image",
                            "schema": None,
                            "defaultValue": "",
                            "value": image_meta,
                            "valueType": "Image",
                            "description": "输入的图片",
                            "optional": True,
                            "readonly": False,
                            "constraint": None,
                            "visuals": "",
                        }
                    ],
                    "defaultValue": "",
                    "value": "",
                    "valueType": "DataSet",
                    "description": "",
                    "optional": True,
                    "readonly": False,
                    "constraint": None,
                    "visuals": "",
                }
            ],
            "query": (
                "图中有一个交通标志。仔细看它的特征："
                "有红色圆圈的是限速(limit)，"
                "有黑色斜线或叉号、没有红圈的是解除限速(unlimit)。"
                "只回答一个词：限速 或 解除限速"
            ),
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        try:
            resp = requests.post(self.api_url, json=body, headers=headers, timeout=60)
            if resp.ok:
                data = resp.json()
                text = json.dumps(data, ensure_ascii=False)
                # 全文搜索关键词（兼容中文描述和英文返回值）
                if "解除限速" in text or "斜线" in text or "斜杠" in text:
                    return "unlimit"
                if '"unlimit"' in text.lower():
                    return "unlimit"
                if "限速" in text:
                    return "limit"
                if '"limit"' in text.lower():
                    return "limit"
                # 兜底：走标准解析
                return self._parse_result(data)
        except Exception:
            pass
        return None


def update_alert_target(label: str, config_path: str = None):
    """将视觉识别结果写入 config.json 的 alertTarget 字段"""
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), "config.json")

    if not os.path.exists(config_path):
        print(f"{COUT_RED}[VisualLLM] config.json 不存在: {config_path}{COUT_REST}")
        return

    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    config["通用配置参数"]["alertTarget"] = label

    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=2)

    name = LABEL_DICT.get(label, label)
    print(f"{COUT_GREEN}[VisualLLM] alertTarget 已更新为: {label} ({name}){COUT_REST}")


def main():
    """VisualLLM 测试程序"""
    import sys

    print("=" * 60)
    print("    视觉识别测试（百度一见视觉大模型）")
    print("=" * 60)

    vllm = VisualLLM()

    # 支持传入图片路径；否则使用摄像头
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
        return _recognize_file(vllm, image_path)
    else:
        return _recognize_camera(vllm)


def _recognize_file(vllm, image_path):
    """从图片文件识别"""
    import os

    if not os.path.exists(image_path):
        print(f"{COUT_RED}[VisualLLM] 图片不存在: {image_path}{COUT_REST}")
        return

    print(f"识别图片: {image_path}")
    label = vllm.recognize(image_path)
    if label:
        name = LABEL_DICT.get(label, label)
        print(f"{COUT_GREEN}识别结果: {label} ({name}){COUT_REST}")
        _confirm_and_update(label)
    else:
        print(f"{COUT_RED}识别失败{COUT_REST}")


def _recognize_camera(vllm):
    """摄像头拍照识别"""
    import cv2
    import os
    import tempfile

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print(f"{COUT_RED}[VisualLLM] 无法打开摄像头{COUT_REST}")
        return
    time.sleep(0.5)

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"{COUT_RED}[VisualLLM] 读取画面失败，尝试重新拍摄...{COUT_REST}")
            continue

        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            temp_path = tmp.name
        cv2.imwrite(temp_path, frame)

        print(f"\n{COUT_YELLOW}[VisualLLM] 正在识别...{COUT_REST}")
        label = vllm.recognize(temp_path)
        os.remove(temp_path)

        if label:
            name = LABEL_DICT.get(label, label)
            print(f"{COUT_GREEN}[VisualLLM] 识别结果: {label} ({name}){COUT_REST}")
        else:
            print(f"{COUT_RED}[VisualLLM] 识别失败{COUT_REST}")

        confirm = input("\n是否将此识别结果写入配置文件？(y/n): ").strip().lower()
        if confirm == "y":
            if label:
                _confirm_and_update(label)
            break
        print("已取消，重新拍摄识别...")

    cap.release()
    print(f"{COUT_GREEN}[VisualLLM] 摄像头已释放{COUT_REST}")


def _confirm_and_update(label):
    """用户确认后写入配置"""
    import os
    # 尝试用 res/config.json，否则用默认路径
    config_path = os.path.join(os.path.dirname(__file__), "../../res/config.json")
    if os.path.exists(config_path):
        update_alert_target(label, config_path)
    else:
        update_alert_target(label)


if __name__ == "__main__":
    main()
