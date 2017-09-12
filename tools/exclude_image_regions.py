#!/usr/bin/env python
# -*- coding: utf-8 -*-

# based on https://stackoverflow.com/questions/22140880/drawing-rectangle-or-line-using-mouse-events-in-open-cv-using-python
# this is very dirty code ... please dont take it as an example
import cv2
import argparse
import yaml
from copy import deepcopy

boxes = []
global img
img = None
global changed_img
changed_img = None
global sbox
sbox = None
global ebox
ebox = None


def save_boxes_to(boxes, output_yaml):
    with open(output_yaml, 'r') as yaml_file:
        content = yaml.safe_load(yaml_file)

    content["excluded_regions"] = boxes

    with open(output_yaml, 'w') as yaml_file:
        yaml.safe_dump(content, yaml_file, default_flow_style=False)


def on_mouse(event, x, y, flags, params):

    global changed_img
    if not event == cv2.EVENT_LBUTTONUP:
        changed_img = deepcopy(img)

    if y < 0:
        y = 0
    if y > changed_img.shape[0]:
        y = changed_img.shape[0]
    if x < 0:
        x = 0
    if x > changed_img.shape[1]:
        x = changed_img.shape[1]

    if event == cv2.EVENT_LBUTTONDOWN:
        print 'Start Mouse Position: '+str(x)+', '+str(y)
        global sbox
        sbox = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        print 'End Mouse Position: '+str(x)+', '+str(y)
        global ebox
        global img
        ebox = (x, y)
        box = [sbox[0], sbox[1], ebox[0], ebox[1]]
        boxes.append(box)
        img = deepcopy(changed_img)
        sbox = None
        ebox = None
        print("Saved boxes: {}".format(boxes))

    if event == cv2.EVENT_MOUSEMOVE:
        global sbox
        if sbox is not None:
            cbox = (x, y)
            cv2.rectangle(changed_img, sbox, cbox, (0, 0, 255))


arg_parser = argparse.ArgumentParser(description="Image region excluder\n\
Let's you define image regions which will be excepted for ORBextraction \
in ORB-SLAM", formatter_class=argparse.RawTextHelpFormatter)
arg_parser.add_argument("input_image")
arg_parser.add_argument("output_yaml")
args = arg_parser.parse_args()

count = 0
img = cv2.imread(args.input_image)

while(1):
    cv2.namedWindow('Exclude image regions')
    cv2.cv.SetMouseCallback('Exclude image regions', on_mouse, 0)

    if changed_img is not None:
        cv2.imshow('Exclude image regions', changed_img)
    else:
        cv2.imshow('Exclude image regions', img)

    key = cv2.waitKey(50)

    # clear everything on space
    if key == 32:
        global img
        img = None
        global changed_img
        changed_img = None
        global sbox
        sbox = None
        global ebox
        ebox = None
        img = cv2.imread(args.input_image)
        boxes = []
        print("Everything cleared.")

    # save on enter
    elif key == 10 or key == 13:
        save_boxes_to(boxes, args.output_yaml)
        print("Saving to {}".format(args.output_yaml))

    # close program on esc
    elif key == 27:
        cv2.destroyAllWindows()
        break

    # if unknown key print its code
    elif key != -1:
        print("Unknown Key: {}".format(key))
