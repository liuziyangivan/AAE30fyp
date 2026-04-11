"""
core/event_bus.py
轻量级发布/订阅事件总线
"""

from __future__ import annotations
from collections import defaultdict
from typing import Any, Callable
from dataclasses import dataclass 

@dataclass
class TwinState:
    t:     float = 0.0
    x:     float = 0.0
    y:     float = 0.0
    z:     float = 0.0
    vx:    float = 0.0
    vy:    float = 0.0
    vz:    float = 0.0
    roll:  float = 0.0
    pitch: float = 0.0
    yaw:   float = 0.0
    rpm:   float = 0.0

class EventBus:
    """
    用法：
        bus = EventBus()
        bus.subscribe("state_updated", my_callback)
        bus.publish("state_updated", payload=data)
    """

    def __init__(self):
        self._listeners: dict[str, list[Callable]] = defaultdict(list)

    def subscribe(self, event: str, callback: Callable) -> None:
        """注册监听器"""
        self._listeners[event].append(callback)

    def unsubscribe(self, event: str, callback: Callable) -> None:
        """注销监听器"""
        if callback in self._listeners[event]:
            self._listeners[event].remove(callback)

    def publish(self, event: str, **kwargs: Any) -> None:
        """发布事件，所有订阅者按注册顺序同步调用"""
        for cb in self._listeners[event]:
            cb(**kwargs)

    def clear(self, event: str | None = None) -> None:
        """清空指定事件或所有事件的监听器"""
        if event:
            self._listeners[event].clear()
        else:
            self._listeners.clear()

