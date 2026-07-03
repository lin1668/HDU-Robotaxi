#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@file          :llm.py
@Description   :LLM/MLM模型接口 - 支持多提供商
                国内大模型大多兼容OpenAI API格式, 统一使用OpenAIProvider配置
@Date          :2026/05/09 13:51:16
@Autor         :Leo
@Version       :v1.0
"""

import json
import re
import time
import threading
from typing import Optional, Callable, List, Dict, Any, Iterator
from abc import ABC, abstractmethod

try:
    from .libs.utils import *
except ImportError:
    from libs.utils import *


class LLMProvider(ABC):
    """
    LLM提供商抽象基类
    Abstract Base Classes (ABC)
    """

    @abstractmethod
    def __init__(self, config: Dict[str, Any]):
        pass

    @abstractmethod
    def chat(self, messages: List[Dict[str, str]], **kwargs) -> Optional[str]:
        """
        纯文本对话
        """
        pass

    @abstractmethod
    def chatWithImg(self, messages: List[Dict[str, Any]], image_path: str, **kwargs) -> Optional[str]:
        """
        多模态对话(图像输入)
        """
        pass

    @abstractmethod
    def chatStream(self, messages: List[Dict[str, str]], **kwargs) -> Iterator[str]:
        """
        流式对话，返回迭代器
        """
        pass


class OpenAIProvider(LLMProvider):
    """
    OpenAI兼容API提供商(支持百度千帆、阿里千问、智谱AI等)
    """

    def __init__(self, config: Dict[str, Any]):
        self.api_key = config.get("api_key", "")  # 身份验证密钥
        self.base_url = config.get("base_url", "https://qianfan.baidubce.com/v2")  # API 接口的基础地址
        self.model = config.get("model", "ernie-4.5-turbo-128k")  # 模型名称
        self.temperature = config.get("temperature", 0.7)  # 创造性/随机性参数[0,1]
        self.max_tokens = config.get("max_tokens", 500)  # 生成内容的长度限制

    def chat(self, messages: List[Dict[str, str]], **kwargs) -> Optional[str]:
        """
        纯文本对话
        """
        try:
            import requests

            payload = {
                "model": kwargs.get("model", self.model),
                "messages": messages,
                "temperature": kwargs.get("temperature", self.temperature),
                "max_tokens": kwargs.get("max_tokens", self.max_tokens),
                "stream": False,
            }
            resp = requests.post(
                f"{self.base_url.rstrip('/')}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json=payload,
                timeout=60,
            )
            resp.raise_for_status()
            data = resp.json()
            return data["choices"][0]["message"]["content"]
        except Exception as e:
            print(COUT_RED, f"[LLM] 错误: {e}", COUT_REST)
            return None

    def chatWithImg(self, messages: List[Dict[str, Any]], image_path: str, **kwargs) -> Optional[str]:
        """
        多模态对话(图像输入)
        """
        print(COUT_YELLOW, "[MLLM] 多模态对话暂未实现", COUT_REST)
        return None

    def chatStream(self, messages: List[Dict[str, str]], **kwargs) -> Iterator[str]:
        """
        流式对话，返回迭代器
        """
        try:
            import requests

            payload = {
                "model": kwargs.get("model", self.model),
                "messages": messages,
                "temperature": kwargs.get("temperature", self.temperature),
                "max_tokens": kwargs.get("max_tokens", self.max_tokens),
                "stream": True,
            }
            resp = requests.post(
                f"{self.base_url.rstrip('/')}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json=payload,
                stream=True,
                timeout=60,
            )
            resp.raise_for_status()
            for line in resp.iter_lines():
                if line:
                    line = line.decode("utf-8").strip()
                    if line.startswith("data: "):
                        data = line[6:]
                        if data == "[DONE]":
                            break
                        import json
                        chunk = json.loads(data)
                        delta = chunk["choices"][0].get("delta", {})
                        content = delta.get("content")
                        if content:
                            yield content
        except Exception as e:
            print(COUT_RED, f"[LLM] 流式输出错误: {e}", COUT_REST)
            pass


# API提供商映射
PROVIDERS = {
    "qianfan": OpenAIProvider,
    "qwen": OpenAIProvider,
    "zhipu": OpenAIProvider,
}

# 指令解析用严格System Prompt
INSTRUCTION_SYSTEM_PROMPT = """你是一个智能的行驶指令解析器。只输出JSON，不要任何其他文字。

## 核心原则
根据用户指令的**语义**理解任务，而不是机械匹配关键词。理解整句话的含义后再做判断。

## 规则
1. 每个独立的行驶动作/目的地对应一个task
2. 按指令中的先后顺序（先→然后→再→最后）排列tasks
3. 只输出纯JSON，不要markdown包裹、不要解释

## 任务类型

### park（停车场/停车位）
用户在停车场停车、在停车位接人、到达某个车位，或**穿过/经过停车场**。
- spot：从"X号停车位"中提取数字X，如"3号停车位"→3
- 如果只是"穿过停车位/经过停车场"而不停车接人，则 spot=0

### construction（施工区）
用户表达要经过施工区、在施工区中间/出口做某事。
- stop=1：中间 / 中间位置
- stop=2：出口 / 第二个位置 / 结尾处

### fork（岔路口）
用户表达要经过岔路口、Y型路口、分岔路，并选择方向。
- direction="left"：左侧 / 左边 / 左
- direction="right"：右侧 / 右边 / 右

## 易错点——请务必仔细核对
- "中间" → stop=1（绝不等于2）
- "出口" → stop=2（绝不等于1）
- "左侧/左边" → direction="left"（绝不等于right）
- "右侧/右边" → direction="right"（绝不等于left）

## 输出格式
{"tasks":[{"type":"park","spot":3},{"type":"construction","stop":2},{"type":"fork","direction":"left"}]}

## 示例

指令：先去施工区中间接个人，把他送到停车场1号停车位，然后把我送到岔路口左侧位置
分析：接人→施工区中间(stop=1)，送人→停车场1号位(spot=1)，最后去岔路口左侧(direction="left")
输出：{"tasks":[{"type":"construction","stop":1},{"type":"park","spot":1},{"type":"fork","direction":"left"}]}

指令：先去施工区出口，再把他送到停车场1号停车位
分析：先去施工区出口(stop=2)，再去停车场1号位(spot=1)
输出：{"tasks":[{"type":"construction","stop":2},{"type":"park","spot":1}]}

指令：先走岔路口右侧，再去施工区中间，最后到停车场3号停车位
分析：岔路口右侧(direction="right")→施工区中间(stop=1)→停车场3号位(spot=3)
输出：{"tasks":[{"type":"fork","direction":"right"},{"type":"construction","stop":1},{"type":"park","spot":3}]}

指令：去停车场4号停车位
输出：{"tasks":[{"type":"park","spot":4}]}

指令：先去施工区中间，然后去停车场3号停车位，最后去岔路口左侧
输出：{"tasks":[{"type":"construction","stop":1},{"type":"park","spot":3},{"type":"fork","direction":"left"}]}

指令：先走岔路口右侧，再去停车场2号停车位
输出：{"tasks":[{"type":"fork","direction":"right"},{"type":"park","spot":2}]}

指令：去施工区中间
输出：{"tasks":[{"type":"construction","stop":1}]}

指令：先去停车场2号停车位接人，再去岔路口右边
输出：{"tasks":[{"type":"park","spot":2},{"type":"fork","direction":"right"}]}

指令：经过施工区中间去停车场3号停车位
输出：{"tasks":[{"type":"construction","stop":1},{"type":"park","spot":3}]}

指令：先穿过停车位，再去岔路口左边
输出：{"tasks":[{"type":"park","spot":0},{"type":"fork","direction":"left"}]}"""

# LLM 硬编码配置（替代 params.yaml）
LLM_CONFIG = {
    "provider": "qianfan",
    "role": "你是一个智能小车任务分析助手, 你叫小赛, 现在任务分成好几圈，你需要将任务分析成多个子任务, 并根据子任务的优先级和依赖关系, 生成一个任务执行计划。",
    "providers": {
        "qianfan": {
            "api_key": "",
            "base_url": "https://qianfan.baidubce.com/v2",
            "model": "ernie-4.5-turbo-128k",
            "temperature": 0.7,
            "max_tokens": 500,
        },
    },
}


class LLM:
    """
    大模型统一接口
    """

    def __init__(self, maxHistory: int = 10):
        self.config = {}  # LLM配置参数
        self.provider: Optional[LLMProvider] = None  # API提供商实例
        self.historyChat: List[Dict[str, str]] = []  # 对话历史记录
        self.maxHistory = maxHistory  # 保留最近10轮对话
        self.lock = threading.Lock()  # 线程锁
        self.loadConfigs()  # 加载配置

    def loadConfigs(self):
        """
        加载配置（从 LLM_CONFIG 硬编码常量）
        """

        self.config = LLM_CONFIG

        # 初始化API提供商
        provider = self.config.get("provider", "openai")
        providers = self.config.get("providers", {}).get(provider, {})
        if provider in PROVIDERS:
            self.provider = PROVIDERS[provider](providers)
            print(COUT_GREEN, f"[LLM] 已加载大模型API提供商: {provider} - {self.provider.model}", COUT_REST)
        else:
            print(COUT_RED, f"[LLM] 未知在线大模型API提供商: {provider}", COUT_REST)

    def chat(self, query: str, **kwargs) -> Optional[str]:
        """
        对话

        Args:
            query: 用户问题
            **kwargs: 其他参数

        Returns:
            回答文本
        """
        if self.provider is None:
            return None

        with self.lock:
            messages = []
            role = self.config.get("role", None)
            messages.append({"role": "system", "content": role})
            messages.extend(self.historyChat)
            messages.append({"role": "user", "content": query})
            response = self.provider.chat(messages, **kwargs)
            if response:
                self.historyChat.append({"role": "user", "content": query})
                self.historyChat.append({"role": "assistant", "content": response})
                if len(self.historyChat) > self.maxHistory * 2:
                    self.historyChat = self.historyChat[-self.maxHistory * 2 :]

            return response

    def chatWithContext(self, query: str, context: str) -> Optional[str]:
        """
        带上下文的对话(RAG用)

        Args:
            query: 用户问题
            context: RAG检索到的上下文
        Returns:
            回答文本
        """
        queryEnhanced = f"""参考信息：
{context}

用户问题：{query}"""

        return self.chat(queryEnhanced)

    def clearHistory(self):
        """清除对话历史"""
        with self.lock:
            self.historyChat.clear()

    def _extractJson(self, text: str) -> Optional[str]:
        """从LLM响应中提取JSON字符串，支持多种包裹格式"""
        text = text.strip()
        # 尝试解析 ```json ... ```、```...``` 包裹
        match = re.search(r'```(?:json)?\s*(\{.*?\})```', text, re.DOTALL)
        if match:
            return match.group(1).strip()
        # 尝试直接找 {...} 顶层结构
        match = re.search(r'(\{.*\})', text, re.DOTALL)
        if match:
            return match.group(1).strip()
        return None

    def _validateTasks(self, data: dict) -> bool:
        """校验解析结果格式是否完整有效"""
        tasks = data.get("tasks", [])
        if not isinstance(tasks, list) or not tasks:
            return False
        for task in tasks:
            if not isinstance(task, dict):
                return False
            ttype = task.get("type")
            if ttype not in ("park", "construction", "fork"):
                return False
            if ttype == "park":
                if task.get("spot") is not None and not isinstance(task.get("spot"), int):
                    return False
                if task.get("spot") is None:
                    task["spot"] = 0
            if ttype == "construction" and task.get("stop") not in (1, 2):
                return False
            if ttype == "fork" and task.get("direction") not in ("left", "right"):
                return False
        return True

    def parseInstruction(self, instruction: str) -> Optional[Dict[str, Any]]:
        """
        解析行驶指令，提取结构化配置参数

        Args:
            instruction: 自然语言行驶指令，如"先去停车场4号停车位..."
        Returns:
            {"tasks": [...]} 或 None（解析失败时）
        """
        if self.provider is None:
            return None

        messages = [
            {"role": "system", "content": INSTRUCTION_SYSTEM_PROMPT},
            {"role": "user", "content": f"指令：{instruction}"}
        ]

        response = self.provider.chat(messages, temperature=0.0)
        if not response:
            return None

        # 提取并解析JSON
        cleaned = self._extractJson(response)
        if not cleaned:
            return None

        try:
            result = json.loads(cleaned)
            if not self._validateTasks(result):
                return None
            return result
        except json.JSONDecodeError:
            return None


def main():
    """LLM测试程序"""
    print("=" * 60)
    print("    [LLM] LLM/MLM接口测试")
    print("=" * 60)

    llm = LLM()

    while True:
        query = input("\n请输入问题 (q退出/c清除历史): ")
        if query.lower() == "q":  # 退出程序
            break
        if query.lower() == "c":  # 清除历史对话
            llm.clearHistory()
            print(COUT_GREEN, "[LLM] 历史对话已清除!", COUT_REST)
            continue

        tStart = time.time()
        response = llm.chat(query)
        tEnd = time.time()
        print(COUT_YELLOW, f"[LLM] 耗时: {(tEnd - tStart):.2f}秒", COUT_REST)
        if response:
            print(COUT_GREEN, f"[LLM] 回答: {response}", COUT_REST)
        else:
            print(COUT_RED, "[LLM] 获取回答失败!", COUT_REST)


if __name__ == "__main__":
    main()
