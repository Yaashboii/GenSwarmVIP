# -*- coding: utf-8 -*-

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def create_char_image(char, font_path, font_size=200, image_size=(300, 300)):
    img = np.zeros((image_size[0], image_size[1], 3), dtype=np.uint8)
    pil_img = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_img)
    font = ImageFont.truetype(font_path, font_size)
    text_size = draw.textbbox((0, 0), char, font=font)
    text_x = (image_size[1] - (text_size[2] - text_size[0])) // 2
    text_y = (image_size[0] - (text_size[3] - text_size[1])) // 2
    draw.text((text_x, text_y), char, font=font, fill=(255, 255, 255))
    img = np.array(pil_img)
    return img


def draw_contours(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary_img = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return contours


def sample_contour_points(contours, num_points):
    all_points = []
    total_length = 0

    for contour in contours:
        contour = contour[:, 0, :]
        distances = np.sqrt(np.sum(np.diff(contour, axis=0) ** 2, axis=1))
        total_length += np.sum(distances)
        all_points.append((contour, distances))

    if total_length == 0:
        return []

    sample_distance = total_length / num_points
    sampled_points = []
    accumulated_distance = 0

    for contour, distances in all_points:
        for i in range(1, len(contour)):
            segment_distance = distances[i - 1]
            accumulated_distance += segment_distance

            while accumulated_distance >= sample_distance:
                ratio = (accumulated_distance - sample_distance) / segment_distance
                new_point = contour[i] - ratio * (contour[i] - contour[i - 1])
                sampled_points.append(tuple(new_point))
                accumulated_distance -= sample_distance

    return sampled_points


def validate_contour_points(char, num_points=150):
    font_path = "HanYiCuYuanJian-1.ttf"
    char_image = create_char_image(char, font_path)
    contours = draw_contours(char_image)
    sampled_points = sample_contour_points(contours, num_points)
    print(len(sampled_points))
    # sampled_img = np.zeros_like(char_image)
    # for point in sampled_points:
    #     cv2.circle(sampled_img, (int(point[0]), int(point[1])), 1, (255, 255, 255), -1)
    #
    # cv2.imshow('Sampled Contour Points', sampled_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    sampled_points = np.array(sampled_points)
    # remap to pygame coordinates
    sampled_points = sampled_points / 300 * 1000
    return sampled_points

#
# # Example usage:
# font_path = "HanYiCuYuanJian-1.ttf"
# character = '￥'
# num_points = 150  # Adjust the number of points as needed
# validate_contour_points(character, num_points)