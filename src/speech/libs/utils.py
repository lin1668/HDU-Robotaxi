#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
工具函数
"""

import os


# 终端颜色输出
COUT_RED = "\033[91m"
COUT_GREEN = "\033[92m"
COUT_YELLOW = "\033[93m"
COUT_REST = "\033[0m"


def getPathPkg(name: str) -> str:
    """获取包路径（兼容旧调用）"""
    return os.path.dirname(os.path.abspath(__file__))
