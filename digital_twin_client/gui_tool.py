import numpy as np
import cv2

class Button:
    def __init__(self, pos, size):
        self.pos = pos
        self.size = size

    def is_clicked(self, pos):
        return self.pos[0] <= pos[0] <= self.pos[0]+self.size[0] and self.pos[1] <= pos[1] <= self.pos[1]+self.size[1]


class DrawTool:
    def __init__(self, width, height, background_color=(0, 0, 0)):
        self.image = np.full((height, width, 3),
                             background_color, dtype=np.uint8)

    def draw_image(self, image, pos, size):
        show_image = image.copy()

        show_image = cv2.resize(show_image, size)
        self.image[pos[1]:pos[1]+size[1], pos[0]:pos[0]+size[0]] = show_image

    def draw_text(self, text, pos, size=1, color=(255, 255, 255), thickness=1):
        cv2.putText(self.image, text, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

    def draw_at_text(self, text, pos, size=1, color=(255, 255, 255), thickness=1):
        text_size, _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, size, thickness)
        x = pos[0] - text_size[0] // 2
        y = pos[1] + text_size[1] // 2
        cv2.putText(self.image, text, (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

    def draw_button(self, button, text, color=(255, 255, 255), thickness=-1,line_type=cv2.LINE_8, text_size=2, text_color=(0, 0, 0),text_thickness=1):
        bottom_right = tuple(pos+size for pos, size in zip(button.pos, button.size))
        cv2.rectangle(self.image, button.pos, bottom_right, color, thickness,line_type)

        text_pos = tuple(pos+size//2 for pos, size in zip(button.pos, button.size))
        self.draw_at_text(text, text_pos, text_size, text_color, text_thickness)