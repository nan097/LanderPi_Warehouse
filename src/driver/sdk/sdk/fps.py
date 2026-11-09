#!/usr/bin/python3
# coding=utf8
import cv2
import time

class FPS:
    def __init__(self):
        self.fps = 0.0# 初始帧率(initial frame rate)
        self.last_time = 0# 上一帧时间(last frame time)
        self.current_time = 0# 当前帧时间(current frame time)
        self.confidence = 0.1# 置信度(confidence)

    def update(self):
        self.last_time = self.current_time# 上一帧时间(last frame time)
        self.current_time = time.time()# 当前帧时间(current frame time)
        new_fps = 1.0 / (self.current_time - self.last_time)# 计算新帧率(calculate new frame rate)
        if self.fps == 0.0:# 初始帧率为0时，直接赋值(if initial frame rate is 0, assign directly)
            self.fps = new_fps if self.last_time != 0 else 0.0
        else:# 平滑更新帧率(smoothly update frame rate)
            self.fps = new_fps * self.confidence + self.fps * (1.0 - self.confidence)
        return float(self.fps)

    def show_fps(self, img):# 在图像上显示帧率(display frame rate on image)
        line = cv2.LINE_AA
        font = cv2.FONT_HERSHEY_PLAIN
        fps_text = 'FPS: {:.2f}'.format(self.fps)
        cv2.putText(img, fps_text, (11, 20), font, 1.0, (32, 32, 32), 4, line)
        cv2.putText(img, fps_text, (10, 20), font, 1.0, (240, 240, 240), 1, line)
        return img

