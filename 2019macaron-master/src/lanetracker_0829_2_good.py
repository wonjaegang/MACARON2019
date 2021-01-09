#!/usr/bin/env python
#-*-coding:utf-8-*_

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
##   rostopic echo color
## this demo shows some of the more advanced APIs in rospy.

import sys
from cv_bridge import CvBridge, CvBridgeError
# from scipy.ndimage import filters

import cv2
import numpy as np
import time
import roslib

roslib.load_manifest('macaron')

import rospy

from matplotlib import pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension

from macaron.msg import Floats
from macaron.msg import Floats_for_mission

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from multiprocessing import Process, Pipe, Queue, queues
import os


flaots = Floats()
flaots_for_mission = Floats_for_mission()




def plot_stop_peak_hist(warped_gray, base_stop_image_yellow, road_width, PIXEL_CONVERSION_RATE, is_plot=True):
    stop_histogram_v = np.sum(warped_gray, axis=1)
    stop_histogram_h = np.sum(warped_gray, axis=0)
    stop_histogram_h[stop_histogram_h > 0] = 1

    yellow_histogram_v = np.sum(base_stop_image_yellow, axis=1)
    yellow_histogram_v[yellow_histogram_v > 0] = 1
    yellow_nonzero = yellow_histogram_v.nonzero()[0]
    yellow_length_nonzero = np.copy(stop_histogram_v).nonzero()[0]

    length_stop_histogram = sum(stop_histogram_h) * PIXEL_CONVERSION_RATE
    yellow_area_per = np.sum(base_stop_image_yellow) / (base_stop_image_yellow.shape[0] * base_stop_image_yellow.shape[1])
    print('yellow_area_per = ', yellow_area_per)

    max_stop_histogram = max(stop_histogram_v) * PIXEL_CONVERSION_RATE
    stop_thresh_low = 20
    stop_thresh_high = 70

    bump_basex_y = 0
    bump_basex_extra = 0
    if len(yellow_nonzero) > 0: bump_basex_y = np.average(yellow_nonzero)
    bump_basex_y_roi = [int(bump_basex_y + 2 / PIXEL_CONVERSION_RATE), 0]
    if bump_basex_y > 3 / PIXEL_CONVERSION_RATE: bump_basex_y_roi = [int(bump_basex_y + 2 / PIXEL_CONVERSION_RATE), int(bump_basex_y - 3 / PIXEL_CONVERSION_RATE) - 1]
    bump_height_extra = len(yellow_length_nonzero[(yellow_length_nonzero >= bump_basex_y_roi[1]) & (yellow_length_nonzero <= bump_basex_y_roi[0])])
    if len(yellow_length_nonzero[(yellow_length_nonzero >= bump_basex_y_roi[1]) & (yellow_length_nonzero <= bump_basex_y_roi[0])]) > 0: bump_basex_extra = np.average(yellow_length_nonzero[(yellow_length_nonzero >= bump_basex_y_roi[1]) & (yellow_length_nonzero <= bump_basex_y_roi[0])])

    if (length_stop_histogram >= road_width - 1) & (max_stop_histogram > road_width - 1.5):
        line_type = 2
        print('line_type = 2, stop_line')
        stopx_base = np.argmax(stop_histogram_v)
        bump_basex = 0
        bump_height = 0


    elif (length_stop_histogram > road_width - 3) & (length_stop_histogram < road_width - 1.7):
        line_type = 1
        print('line_type = 1, not_stop_line')
        stopx_base = 0
        bump_basex = 0
        bump_height = 0

    else:  # sum(stop_histogram_h == 0)
        line_type = 0
        print('line_type = 0, clean_line')
        stopx_base = 0
        bump_basex = 0
        bump_height = 0

    if yellow_area_per > 0.003:
        line_type = 3
        print('line_type = 3, yellow_line')
        stopx_base = 0
        bump_basex = bump_basex_extra
        bump_height = bump_height_extra

    return length_stop_histogram, line_type, stopx_base, max_stop_histogram, bump_basex, bump_height


def plot_peak_hist(warped_gray, road_width, PIXEL_CONVERSION_RATE, is_plot=True):
    # warped_gray[:, (warped_gray.shape[1] // 7) * 3 - 1:(warped_gray.shape[1] // 7) * 4 - 1] = 0
    inner_line = np.copy(warped_gray[:, int(warped_gray.shape[1] // 2 - road_width / PIXEL_CONVERSION_RATE / 2 - 1):int(warped_gray.shape[1] // 2 + road_width / PIXEL_CONVERSION_RATE / 2 - 1)])
    warped_gray[:, int(warped_gray.shape[1] // 2 - road_width / 3 / PIXEL_CONVERSION_RATE - 1):int(warped_gray.shape[1] // 2 + road_width / 3 / PIXEL_CONVERSION_RATE - 1)] = 0
    warped_gray[:, :int(warped_gray.shape[1] // 2 - (road_width / 2 + 2) / PIXEL_CONVERSION_RATE - 1)] = 0
    warped_gray[:, int(warped_gray.shape[1] // 2 + (road_width / 2 + 2) / PIXEL_CONVERSION_RATE - 1):] = 0
    histogram = np.sum(warped_gray[warped_gray.shape[0] // 2:, :], axis=0)
    inner_hist = np.sum(inner_line, axis=0)
    inner_hist[inner_hist > 0] = 1
    inner_length = sum(inner_hist) * PIXEL_CONVERSION_RATE

    # cv2.imshow('warped_gray', warped_gray * 255)
    # whi = np.ones_like(warped_gray) * 255
    # whi[:, int(warped_gray.shape[1] // 2 - road_width / PIXEL_CONVERSION_RATE / 4 - 1):int(warped_gray.shape[1] // 2 + road_width / PIXEL_CONVERSION_RATE / 4 - 1)] = 0
    # whi[:, :int(warped_gray.shape[1] // 2 - (road_width / 2 + 1) / PIXEL_CONVERSION_RATE - 1)] = 0
    # whi[:, int(warped_gray.shape[1] // 2 + (road_width / 2 + 1) / PIXEL_CONVERSION_RATE - 1):] = 0
    # cv2.imshow('whi', whi)
    if is_plot: plt.plot(histogram)
    midpoint = np.int(histogram.shape[0] / 2)

    if max(histogram[:midpoint]) <= 10 and max(histogram[midpoint:]) <= 10:
        error = 1
        #print('error = 1, plot_peak_hist')
        return [], error, inner_length
    error = 0
    return histogram, error, inner_length


def find_left_right_via_histogram(binary_warped, road_width, PIXEL_CONVERSION_RATE, is_debug=False, is_plot=False, add_debug_image=False):
    debug_images = []

    histogram, error, inner_length = plot_peak_hist(binary_warped, road_width, PIXEL_CONVERSION_RATE, is_plot=False)
    if error == 1:
        print('error = 1, plot_peak_hist')
        return [], [], [], error, inner_length
    error = 0

    if add_debug_image: debug_images.append(histogram)
    # if add_debug_image: debug_images.append(combined_binary)

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    # if np.max(histogram[:midpoint]) <= 10:
    #    leftx_base = 0
    #    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    #    following_left_lane = False
    #    following_right_lane = True
    # elif np.max(histogram[midpoint:]) <= 10:
    #    leftx_base = np.argmax(histogram[:midpoint])
    #    rightx_base = 0
    #    following_left_lane = True
    #    following_right_lane = False
    # else:
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    # following_left_lane = False
    # following_right_lane = False

    if is_debug:
        print("leftx_base:" + str(leftx_base))
        print("rightx_base:" + str(rightx_base))
        print("midpoint:" + str(midpoint))

    return leftx_base, rightx_base, debug_images, error, inner_length  # , following_left_lane, following_right_lane


def find_ploy_fit_window_search(binary_warped, leftx_base, rightx_base, following_left_lane, following_right_lane, road_width, PIXEL_CONVERSION_RATE, target_left_pixel_percent, target_right_pixel_percent, full_line_target_percent, roi_limit_L, roi_limit_M, roi_limit_R, PT_view_height, stopx_base,
                                nwindows=9, is_debug=False, margin=100, minpix=50, is_plot=True, add_debug_image=True):
    debug_images = []

    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    if is_debug:
        print("window_height:" + str(window_height))

    binary_warped[:, int(binary_warped.shape[1] // 2 - road_width / PIXEL_CONVERSION_RATE / 4 - 1):int(binary_warped.shape[1] // 2 + road_width / PIXEL_CONVERSION_RATE / 4 - 1)] = 0
    binary_warped[:, :int(binary_warped.shape[1] // 2 - (road_width / 2 + 2) / PIXEL_CONVERSION_RATE - 1)] = 0
    binary_warped[:, int(binary_warped.shape[1] // 2 + (road_width / 2 + 2) / PIXEL_CONVERSION_RATE - 1):] = 0

    roi_stop = stopx_base
    if stopx_base > 5: roi_stop = stopx_base + PT_view_height / 10

    binary_warped[: int(roi_stop), :] = 0
    print('roi_stop = ', roi_stop)

    L_per = np.sum(binary_warped[:, :binary_warped.shape[1] // 2]) / (binary_warped.shape[0] * binary_warped.shape[1] / 2)
    R_per = np.sum(binary_warped[:, binary_warped.shape[1] // 2:]) / (binary_warped.shape[0] * binary_warped.shape[1] / 2)

    if ((L_per <= full_line_target_percent / 2) & (R_per > target_right_pixel_percent * 1.5)):
        following_right_lane = True
        following_left_lane = False
        print('in_window :: following_right_lane = ', following_right_lane, '****', '  L_per = ', L_per, 'R_per = ', R_per)
        # print('윈도우 팔로우 1')
    elif ((R_per <= full_line_target_percent / 2) & (L_per > target_left_pixel_percent * 1.5)):
        following_right_lane = False
        following_left_lane = True
        print('in_window :: following_left_lane = ', following_left_lane, '****', '  L_per= ', L_per, 'R_per = ', R_per)
        # print('윈도우 팔로우 2')
    else:
        following_right_lane = False
        following_left_lane = False
        print('윈도우 팔로우 3  L_per, R_per = ', L_per, ' , ', R_per)

    # if ((L_per <= full_line_target_percent / 5) & (R_per > target_right_pixel_percent / 2)) | ((L_per <= full_line_target_percent / 2) & (R_per > target_right_pixel_percent * 1.5)):
    #     following_right_lane = True
    #     following_left_lane = False
    #     print('in_window :: following_right_lane = ', following_right_lane, '****', '  L_per = ', L_per, 'R_per = ',R_per)
    #     # print('윈도우 팔로우 1')
    # elif ((R_per <= full_line_target_percent / 5) & (L_per > target_left_pixel_percent / 2)) | ((R_per <= full_line_target_percent / 2) & (L_per > target_left_pixel_percent * 1.5)):
    #     following_right_lane = False
    #     following_left_lane = True
    #     print('in_window :: following_left_lane = ', following_left_lane, '****', '  L_per= ', L_per, 'R_per = ', R_per)
    #     # print('윈도우 팔로우 2')
    # else:
    #     following_right_lane = False
    #     following_left_lane = False
    #     print('윈도우 팔로우 3  L_per, R_per = ', L_per,' , ', R_per)

    # get x, y points for non zero pixels.
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated for each window
    # this is the max position for left and right
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Set the width of the windows +/- margin
    # Set minimum number of pixels found to recenter window

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    left_top_y = roi_limit_L
    left_mid_y = (PT_view_height - 1 + roi_limit_L) // 2
    left_bot_y = PT_view_height - 1

    center_top_y = roi_limit_M
    center_mid_y = (PT_view_height - 1 + roi_limit_M) // 2
    center_bot_y = PT_view_height - 1

    right_top_y = roi_limit_R
    right_mid_y = (PT_view_height - 1 + roi_limit_R) // 2
    right_bot_y = PT_view_height - 1

    # Step through the windows one by one
    if following_left_lane == False and following_right_lane == False:
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 3)
            cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)), (0, 255, 0), 3)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]
            # good_left_inds = list(good_left_inds)
            # good_right_inds = good_right_inds

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # # # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]

        if len(leftx) < 3 or len(rightx) < 3:
            error = 2
            print('error = 2, find_ploy_fit_window_search, cant_find_line - leftx : ', len(leftx), ', rightx : ', len(rightx))
            return [], [], [], error, 0, 0

        left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, rightx, righty)

    if following_left_lane == True and following_right_lane == False:
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            # win_xright_low = rightx_current - margin
            # win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 3)
            # cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)), (0, 255, 0), 3)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]
            # good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
            #            nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            # right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            # if len(good_right_inds) > minpix:
            #    rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        # right_lane_inds = np.concatenate(right_lane_inds)

        # # # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]

        # print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
        out_img[lefty, leftx] = [255, 0, 0]
        # out_img[righty, rightx] = [255, 0, 0]

        if len(leftx) < 3:
            error = 2
            print('error = 2, find_ploy_fit_window_search, cant_find_line - leftx : ', len(leftx))
            return [], [], [], error, 0, 0

        left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, leftx, lefty)

        left_fit_mid_grad = 2 * left_fit[0] * left_mid_y + left_fit[1]

        if (left_fit_mid_grad == 0) | (abs(left_fit_mid_grad) < 0.01):
            move_y = 0
            move_x = road_width / PIXEL_CONVERSION_RATE
            print('unwarped 1 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
        else:
            left_fit_mid_normal_grad = -1 / left_fit_mid_grad
            left_fit_mid_normal_rad = np.arctan(left_fit_mid_normal_grad)
            if left_fit_mid_normal_rad < 0:
                move_y = -1 * road_width / PIXEL_CONVERSION_RATE * np.cos(left_fit_mid_normal_rad)
                move_x = -1 * road_width / PIXEL_CONVERSION_RATE * np.sin(left_fit_mid_normal_rad)
                print('unwarped 2 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
            else:
                move_y = road_width / PIXEL_CONVERSION_RATE * np.cos(left_fit_mid_normal_rad)
                move_x = road_width / PIXEL_CONVERSION_RATE * np.sin(left_fit_mid_normal_rad)
                print('unwarped 3 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)

        right_fit = [left_fit[0], left_fit[1] - 2 * left_fit[0] * move_y, left_fit[0] * move_y ** 2 - left_fit[1] * move_y + left_fit[2] + move_x]

    if following_left_lane == False and following_right_lane == True:
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            # win_xleft_low = leftx_current - margin
            # win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            # cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 3)
            cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), (int(win_xright_high), int(win_y_high)), (0, 255, 0), 3)

            # Identify the nonzero pixels in x and y within the window
            # good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
            #            nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            # left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            # if len(good_left_inds) > minpix:
            #    leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        # left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # # # Extract left and right line pixel positions
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
        # out_img[lefty, leftx] = [0, 0, 255]
        out_img[righty, rightx] = [0, 0, 255]

        if len(rightx) < 3:
            error = 2
            print('error = 2, find_ploy_fit_window_search, cant_find_line - rightx : ', len(rightx))
            return [], [], [], error, 0, 0

        left_fit, right_fit = fit_ploy_2_degree(rightx, righty, rightx, righty)

        right_fit_mid_grad = 2 * right_fit[0] * right_mid_y + right_fit[1]
        if (right_fit_mid_grad == 0) | (abs(right_fit_mid_grad) < 0.01):
            move_y = 0
            move_x = -1 * (road_width / PIXEL_CONVERSION_RATE)
            print('unwarped 1 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
        else:
            right_fit_mid_normal_grad = -1 / right_fit_mid_grad
            right_fit_mid_normal_rad = np.arctan(right_fit_mid_normal_grad)
            if right_fit_mid_normal_rad < 0:
                move_y = road_width / PIXEL_CONVERSION_RATE * np.cos(right_fit_mid_normal_rad)
                move_x = road_width / PIXEL_CONVERSION_RATE * np.sin(right_fit_mid_normal_rad)
                print('unwarped 2 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
            else:
                move_y = -1 * road_width / PIXEL_CONVERSION_RATE * np.cos(right_fit_mid_normal_rad)
                move_x = -1 * road_width / PIXEL_CONVERSION_RATE * np.sin(right_fit_mid_normal_rad)
                print('unwarped 3 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)

        left_fit = [right_fit[0], right_fit[1] - 2 * right_fit[0] * move_y, right_fit[0] * move_y ** 2 - right_fit[1] * move_y + right_fit[2] + move_x]

    error = 0

    if add_debug_image:
        out_img = plot_detected_poly(out_img, left_fit, right_fit, is_plot=is_plot)
        debug_images.append(out_img)

    return left_fit, right_fit, debug_images, error, following_left_lane, following_right_lane


def fit_ploy_2_degree(leftx, lefty, rightx, righty):
    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    return left_fit, right_fit


def plot_detected_poly(out_img, left_fit, right_fit, is_plot=True):
    # Generate x and y values for plotting
    ploty = np.linspace(0, out_img.shape[0] - 1, out_img.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    for pts, index in enumerate(left_fitx):
        cv2.circle(out_img, ((int)(left_fitx[pts]), (int)(ploty[pts])), 3, (255, 255, 255), -1)

    for pts, index in enumerate(right_fitx):
        cv2.circle(out_img, ((int)(right_fitx[pts]), (int)(ploty[pts])), 3, (255, 255, 255), -1)

    #     plt.plot(left_fitx, ploty, color='yellow')
    #     plt.plot(right_fitx, ploty, color='yellow')

    if is_plot:
        plt.imshow(out_img)
        # print(out_img.shape, '-', out_img.dtype, '-', np.min(out_img), '-', np.max(out_img))
        # out_img_uint32 = out_img.astype(np.uint8)
        # cv2.imshow('out_img_uint32',out_img_uint32)
        # plt.xlim(0, out_img.shape[1])
        # plt.ylim(out_img.shape[0], 0)
    return out_img


def next_frame_find_poly_already_fitted(binary_warped_left, binary_warped_right, left_fit, right_fit, lower_offset, upper_offset, following_right_lane, following_left_lane, road_width, PIXEL_CONVERSION_RATE, error_loop, PT_view_height, stopx_base, zero_1, zero_3, line_type, bump_basex, bump_height,
                                        viewsize, ym_per_pix, margin=50, is_plot=True,
                                        add_debug_image=True):
    # Assume you now have a new warped binary image
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!

    left_fit_grad_sum = 0
    for i in range(23):
        left_fit_grad_sum = left_fit_grad_sum + 2 * left_fit[0] * (viewsize[1] // 23) * i + left_fit[1]
        # print(np.rad2deg(np.arctan(2 * center_fit[0] * (viewsize[1] // 23) * i + center_fit[1])))
    left_fit_grad_ave = left_fit_grad_sum / 23
    left_fit_deg_ave = np.rad2deg(np.arctan(left_fit_grad_ave))

    right_fit_grad_sum = 0
    for i in range(23):
        right_fit_grad_sum = right_fit_grad_sum + 2 * right_fit[0] * (viewsize[1] // 23) * i + right_fit[1]
        # print(np.rad2deg(np.arctan(2 * center_fit[0] * (viewsize[1] // 23) * i + center_fit[1])))
    right_fit_grad_ave = right_fit_grad_sum / 23
    right_fit_deg_ave = np.rad2deg(np.arctan(right_fit_grad_ave))
    # print(left_fit_deg_ave * right_fit_deg_ave)

    center_fit = np.add(left_fit, right_fit) / 2
    center_fit_grad_sum = 0
    for i in range(23):
        center_fit_grad_sum = center_fit_grad_sum + 2 * center_fit[0] * (viewsize[1] // 23) * i + center_fit[1]
        # print(np.rad2deg(np.arctan(2 * center_fit[0] * (viewsize[1] // 23) * i + center_fit[1])))
    center_fit_grad_ave = center_fit_grad_sum / 23
    center_fit_deg_ave = np.rad2deg(np.arctan(center_fit_grad_ave))

    debug_images = []

    nonzero_left = binary_warped_left.nonzero()
    nonzero_right = binary_warped_right.nonzero()
    nonzeroy_left = np.array(nonzero_left[0])
    nonzerox_left = np.array(nonzero_left[1])
    nonzeroy_right = np.array(nonzero_right[0])
    nonzerox_right = np.array(nonzero_right[1])
    roi_limit = 0
    roi_limit_M = 0
    roi_limit_L = 0
    roi_limit_R = 0
    roi_curv_L = 0
    roi_curv_R = 0
    roi_stop = stopx_base
    roi_curv = 0
    roi_limit_bump = [0, 0]
    if stopx_base > 5: roi_stop = stopx_base + PT_view_height / 10
    roi_limit = int(roi_stop)
    print('bump_basex = ', bump_basex, 'bump_height = ', bump_height)
    if line_type == 3: roi_limit_bump = [int(bump_basex + bump_height / 2 + PT_view_height / 12), int(bump_basex - bump_height / 2)]

    if bump_basex - bump_height / 2 > PT_view_height / 12: roi_limit_bump = [int(bump_basex + bump_height / 2 + PT_view_height / 12), int(bump_basex - bump_height / 2 - PT_view_height / 12)]
    print('roi_limit_bump = ', roi_limit_bump)

    # roi_ind_left = nonzeroy_left > roi_limit
    # roi_ind_right = nonzeroy_right > roi_limit

    # nonzeroy_left = nonzeroy_left[roi_ind_left]
    # nonzerox_left = nonzerox_left[roi_ind_left]
    # nonzeroy_right = nonzeroy_right[roi_ind_right]
    # nonzerox_right = nonzerox_right[roi_ind_right]

    max_curb = 10  # 20 - 10
    target_ratio = 3 / 5
    curb_factor = target_ratio / max_curb ** 2

    # if (abs(center_fit_deg_ave) > 10):
    #    roi_curv_M = int(PT_view_height * (curb_factor * (abs(center_fit_deg_ave) - 10) ** 2))
    #    if roi_curv_M >= PT_view_height / 3 * 2 - 1: roi_curv_M = PT_view_height // 3 * 2 - 1

    if (abs(left_fit_deg_ave) > 10):
        roi_curv_L = int(PT_view_height * (curb_factor * (abs(left_fit_deg_ave) - 10) ** 2))
        if roi_curv_L >= PT_view_height * target_ratio - 1: roi_curv_L = int(PT_view_height * target_ratio - 1)

    if (abs(right_fit_deg_ave) > 10):
        roi_curv_R = int(PT_view_height * (curb_factor * (abs(right_fit_deg_ave) - 10) ** 2))
        if roi_curv_R >= PT_view_height * target_ratio - 1: roi_curv_R = int(PT_view_height * target_ratio - 1)

    roi_limit_L = int(max(roi_curv_L, roi_limit))
    roi_limit_R = int(max(roi_curv_R, roi_limit))

    if roi_limit_L > PT_view_height / 5 * 4: roi_limit_L = int(PT_view_height / 5 * 4)
    if roi_limit_R > PT_view_height / 5 * 4: roi_limit_R = int(PT_view_height / 5 * 4)

    if roi_limit_L - roi_limit_R > 2.5 / PIXEL_CONVERSION_RATE:
        roi_limit_R = int(roi_limit_L - 2.5 / PIXEL_CONVERSION_RATE)
    elif roi_limit_R - roi_limit_L > 2.5 / PIXEL_CONVERSION_RATE:
        roi_limit_L = int(roi_limit_R - 2.5 / PIXEL_CONVERSION_RATE)

    roi_limit_M = int((roi_limit_L + roi_limit_R) / 2)

    left_top_y = roi_limit_L
    left_bot_y = PT_view_height - 1
    left_mid_y = (PT_view_height - 1 + roi_limit_L) // 2

    center_top_y = roi_limit_M
    center_bot_y = PT_view_height - 1
    center_mid_y = (PT_view_height - 1 + roi_limit_M) // 2

    right_top_y = roi_limit_R
    right_bot_y = PT_view_height - 1
    right_mid_y = (PT_view_height - 1 + roi_limit_R) // 2

    roi_ind_left = nonzeroy_left > roi_limit_L
    roi_ind_right = nonzeroy_right > roi_limit_R
    nonzeroy_left = nonzeroy_left[roi_ind_left]
    nonzerox_left = nonzerox_left[roi_ind_left]
    nonzeroy_right = nonzeroy_right[roi_ind_right]
    nonzerox_right = nonzerox_right[roi_ind_right]

    if line_type == 3:
        roi_ind_left = (nonzeroy_left > roi_limit_bump[0]) | (nonzeroy_left < roi_limit_bump[1])
        roi_ind_right = (nonzeroy_right > roi_limit_bump[0]) | (nonzeroy_right < roi_limit_bump[1])
        nonzeroy_left = nonzeroy_left[roi_ind_left]
        nonzerox_left = nonzerox_left[roi_ind_left]
        nonzeroy_right = nonzeroy_right[roi_ind_right]
        nonzerox_right = nonzerox_right[roi_ind_right]

    left_lane_inds = ((nonzerox_left > (left_fit[0] * (nonzeroy_left ** 2) + left_fit[1] * nonzeroy_left + left_fit[2] - margin))
                      & (nonzerox_left < (left_fit[0] * (nonzeroy_left ** 2) + left_fit[1] * nonzeroy_left + left_fit[2] + margin)))
    right_lane_inds = ((nonzerox_right > (right_fit[0] * (nonzeroy_right ** 2) + right_fit[1] * nonzeroy_right + right_fit[2] - margin))
                       & (nonzerox_right < (right_fit[0] * (nonzeroy_right ** 2) + right_fit[1] * nonzeroy_right + right_fit[2] + margin)))

    # Again, extract left and right line pixel positions
    leftx = nonzerox_left[left_lane_inds]
    lefty = nonzeroy_left[left_lane_inds]
    rightx = nonzerox_right[right_lane_inds]
    righty = nonzeroy_right[right_lane_inds]

    # spectrum_left = 0
    # spectrum_right = 0
    # spectrum_left_block = 0
    # spectrum_right_block = 0
    # if error_loop == 0:
    #     for i in range(len(lefty) - 1):
    #         spectrum_left = spectrum_left + (lefty[i] != lefty[i + 1])
    #         if lefty[i + 1] - lefty[i] > 3 / PIXEL_CONVERSION_RATE:
    #             spectrum_left_block = spectrum_left_block + 1
    #     for i in range(len(righty) - 1):
    #         spectrum_right = spectrum_right + (righty[i] != righty[i + 1])
    #         if righty[i + 1] - righty[i] > 3 / PIXEL_CONVERSION_RATE:
    #             spectrum_right_block = spectrum_right_block + 1
    #     if spectrum_left / binary_warped_left.shape[0] < 0.2:
    #         left_dotted = True
    #     else:
    #         left_dotted = False
    #     if spectrum_right / binary_warped_left.shape[0] < 0.2:
    #         right_dotted = True
    #     else:
    #         right_dotted = False
    #     print('left_dotted = ', left_dotted, 'right_dotted = ', right_dotted)
    #     print('spectrum_left_block = ', spectrum_left_block, 'spectrum_right_block = ', spectrum_right_block)
    #     print('spectrum_left = ', spectrum_left, 'spectrum_right = ', spectrum_right)
    #     print('spectrum_right/binary_warped_left.shape[0] = ', spectrum_right / binary_warped_left.shape[0])

    # Fit a second order polynomial to each
    if len(leftx) < 3 and len(rightx) < 3:
        print('error = 2, next_frame_find_poly_already_fitted, cant_find_line :: leftx : ', len(leftx), ', rightx : ', len(rightx))
        error = 2
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

    # if left_fit_deg_ave * right_fit_deg_ave < 0:
    #    following_left_lane = False
    #    following_right_lane = True

    if following_left_lane:
        if len(leftx) < 3:
            print('error = 2, next_frame_find_poly_already_fitted, following_left_lane, cant_find_line :: leftx : ', len(leftx))
            error = 2
            return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, leftx, lefty)

        left_fit_mid_grad = 2 * left_fit[0] * left_mid_y + left_fit[1]
        if (left_fit_mid_grad == 0) | (abs(left_fit_mid_grad) < 0.01):
            move_y = 0
            move_x = road_width / PIXEL_CONVERSION_RATE
            print('unwarped 1 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
        else:
            left_fit_mid_normal_grad = -1 / left_fit_mid_grad
            left_fit_mid_normal_rad = np.arctan(left_fit_mid_normal_grad)
            if left_fit_mid_normal_rad < 0:
                move_y = -1 * road_width / PIXEL_CONVERSION_RATE * np.cos(left_fit_mid_normal_rad)
                move_x = -1 * road_width / PIXEL_CONVERSION_RATE * np.sin(left_fit_mid_normal_rad)
                print('unwarped 2 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
            else:
                move_y = road_width / PIXEL_CONVERSION_RATE * np.cos(left_fit_mid_normal_rad)
                move_x = road_width / PIXEL_CONVERSION_RATE * np.sin(left_fit_mid_normal_rad)
                print('unwarped 3 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)

        right_fit = [left_fit[0], left_fit[1] - 2 * left_fit[0] * move_y, left_fit[0] * move_y ** 2 - left_fit[1] * move_y + left_fit[2] + move_x]
        # right_fit[2] = left_fit[2] + road_width / PIXEL_CONVERSION_RATE


    elif following_right_lane:
        if len(rightx) < 3:
            print('error = 2, next_frame_find_poly_already_fitted, following_right_lane, cant_find_line ::  rightx : ', len(rightx))
            error = 2
            return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        left_fit, right_fit = fit_ploy_2_degree(rightx, righty, rightx, righty)

        right_fit_mid_grad = 2 * right_fit[0] * right_mid_y + right_fit[1]
        if (right_fit_mid_grad == 0) | (abs(right_fit_mid_grad) < 0.01):
            move_y = 0
            move_x = -1 * (road_width / PIXEL_CONVERSION_RATE)
            print('unwarped 1 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
        else:
            right_fit_mid_normal_grad = -1 / right_fit_mid_grad
            right_fit_mid_normal_rad = np.arctan(right_fit_mid_normal_grad)
            if right_fit_mid_normal_rad < 0:
                move_y = road_width / PIXEL_CONVERSION_RATE * np.cos(right_fit_mid_normal_rad)
                move_x = road_width / PIXEL_CONVERSION_RATE * np.sin(right_fit_mid_normal_rad)
                print('unwarped 2 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
            else:
                move_y = -1 * road_width / PIXEL_CONVERSION_RATE * np.cos(right_fit_mid_normal_rad)
                move_x = -1 * road_width / PIXEL_CONVERSION_RATE * np.sin(right_fit_mid_normal_rad)
                print('unwarped 3 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)

        left_fit = [right_fit[0], right_fit[1] - 2 * right_fit[0] * move_y, right_fit[0] * move_y ** 2 - right_fit[1] * move_y + right_fit[2] + move_x]


    else:
        if len(leftx) < 3 or len(rightx) < 3:
            print('error = 2, next_frame_find_poly_already_fitted, non, cant_find_line :: leftx : ', len(leftx), ', rightx : ', len(rightx))
            error = 2
            return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, rightx, righty)

    left_fitx = left_fit[0] * left_bot_y ** 2 + left_fit[1] * left_bot_y + left_fit[2]
    right_fitx = right_fit[0] * right_bot_y ** 2 + right_fit[1] * right_bot_y + right_fit[2]
    # left_fitx_top = left_fit[2]
    # right_fitx_top = right_fit[2]
    # left_fitx_mid = left_fit[0] * (PT_view_height // 2) ** 2 + left_fit[1] * (PT_view_height // 2) + left_fit[2]
    # right_fitx_mid = right_fit[0] * (PT_view_height // 2) ** 2 + right_fit[1] * (PT_view_height // 2) + right_fit[2]

    center_fitx_bot = center_fit[0] * center_bot_y ** 2 + center_fit[1] * center_bot_y + center_fit[2]
    center_fitx_mid = center_fit[0] * center_mid_y ** 2 + center_fit[1] * center_mid_y + center_fit[2]
    center_fitx_top = center_fit[0] * center_top_y ** 2 + center_fit[1] * center_top_y + center_fit[2]
    # print('center_fitx_bot = ', center_fitx_bot)
    # print('PT_view_height', PT_view_height)
    # print('left_fit, right_fit, center_fit = ', left_fit, right_fit, center_fit)
    center_fit_bot_normal_grad = -1 / (2 * center_fit[0] * center_bot_y + center_fit[1])
    center_fit_mid_normal_grad = -1 / (2 * center_fit[0] * center_mid_y + center_fit[1])
    center_fit_top_normal_grad = -1 / (2 * center_fit[0] * center_top_y + center_fit[1])

    # center_fit_bot_normal_grad * (PT_view_height) + center_fitx_bot- center_fit_bot_normal_grad * (PT_view_height)
    center_fit_bot_normal_line = [center_fit_bot_normal_grad, center_fitx_bot - center_fit_bot_normal_grad * center_bot_y]
    center_fit_mid_normal_line = [center_fit_mid_normal_grad, center_fitx_mid - center_fit_mid_normal_grad * center_mid_y]
    center_fit_top_normal_line = [center_fit_top_normal_grad, center_fitx_top - center_fit_top_normal_grad * center_top_y]
    # print('center_fit_bot_normal_line, center_fit_mid_normal_line, center_fit_top_normal_line = ', center_fit_bot_normal_line, center_fit_mid_normal_line, center_fit_top_normal_line)

    if center_fit_bot_normal_grad < 0:
        normal_bot_left_contacty = (-1 * (left_fit[1] - center_fit_bot_normal_line[0]) + ((left_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_bot_normal_line[1])) ** 0.5) / (2 * left_fit[0])
        normal_bot_right_contacty = (-1 * (right_fit[1] - center_fit_bot_normal_line[0]) + ((right_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_bot_normal_line[1])) ** 0.5) / (2 * right_fit[0])
        normal_bot_left_contactx = center_fit_bot_normal_line[0] * normal_bot_left_contacty + center_fit_bot_normal_line[1]
        normal_bot_right_contactx = center_fit_bot_normal_line[0] * normal_bot_right_contacty + center_fit_bot_normal_line[1]
        print('a')
    else:
        normal_bot_left_contacty = (-1 * (left_fit[1] - center_fit_bot_normal_line[0]) - ((left_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_bot_normal_line[1])) ** 0.5) / (2 * left_fit[0])
        normal_bot_right_contacty = (-1 * (right_fit[1] - center_fit_bot_normal_line[0]) - ((right_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_bot_normal_line[1])) ** 0.5) / (2 * right_fit[0])
        normal_bot_left_contactx = center_fit_bot_normal_line[0] * normal_bot_left_contacty + center_fit_bot_normal_line[1]
        normal_bot_right_contactx = center_fit_bot_normal_line[0] * normal_bot_right_contacty + center_fit_bot_normal_line[1]
        print('b')
        if center_fit_bot_normal_grad == 0: print('center_fit_bot_normal_grad = 0')

    if center_fit_mid_normal_grad < 0:
        normal_mid_left_contacty = (-1 * (left_fit[1] - center_fit_mid_normal_line[0]) + ((left_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_mid_normal_line[1])) ** 0.5) / (2 * left_fit[0])
        normal_mid_right_contacty = (-1 * (right_fit[1] - center_fit_mid_normal_line[0]) + ((right_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_mid_normal_line[1])) ** 0.5) / (2 * right_fit[0])
        normal_mid_left_contactx = center_fit_mid_normal_line[0] * normal_mid_left_contacty + center_fit_mid_normal_line[1]
        normal_mid_right_contactx = center_fit_mid_normal_line[0] * normal_mid_right_contacty + center_fit_mid_normal_line[1]
        print('a')
    else:
        normal_mid_left_contacty = (-1 * (left_fit[1] - center_fit_mid_normal_line[0]) - ((left_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_mid_normal_line[1])) ** 0.5) / (2 * left_fit[0])
        normal_mid_right_contacty = (-1 * (right_fit[1] - center_fit_mid_normal_line[0]) - ((right_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_mid_normal_line[1])) ** 0.5) / (2 * right_fit[0])
        normal_mid_left_contactx = center_fit_mid_normal_line[0] * normal_mid_left_contacty + center_fit_mid_normal_line[1]
        normal_mid_right_contactx = center_fit_mid_normal_line[0] * normal_mid_right_contacty + center_fit_mid_normal_line[1]
        print('b')
        if center_fit_mid_normal_grad == 0: print('center_fit_mid_normal_grad = 0')

    if center_fit_top_normal_grad < 0:
        normal_top_left_contacty = (-1 * (left_fit[1] - center_fit_top_normal_line[0]) + ((left_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_top_normal_line[1])) ** 0.5) / (2 * left_fit[0])
        normal_top_right_contacty = (-1 * (right_fit[1] - center_fit_top_normal_line[0]) + ((right_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_top_normal_line[1])) ** 0.5) / (2 * right_fit[0])
        normal_top_left_contactx = center_fit_top_normal_line[0] * normal_top_left_contacty + center_fit_top_normal_line[1]
        normal_top_right_contactx = center_fit_top_normal_line[0] * normal_top_right_contacty + center_fit_top_normal_line[1]
        print('a')
    else:
        normal_top_left_contacty = (-1 * (left_fit[1] - center_fit_top_normal_line[0]) - ((left_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_top_normal_line[1])) ** 0.5) / (2 * left_fit[0])
        normal_top_right_contacty = (-1 * (right_fit[1] - center_fit_top_normal_line[0]) - ((right_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_top_normal_line[1])) ** 0.5) / (2 * right_fit[0])
        normal_top_left_contactx = center_fit_top_normal_line[0] * normal_top_left_contacty + center_fit_top_normal_line[1]
        normal_top_right_contactx = center_fit_top_normal_line[0] * normal_top_right_contacty + center_fit_top_normal_line[1]
        print('b')
        if center_fit_top_normal_grad == 0: print('center_fit_top_normal_grad = 0')

    # print('(left_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_bot_normal_line[1]) = ', (left_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_bot_normal_line[1]))
    # print('(left_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_mid_normal_line[1]) = ', (left_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_mid_normal_line[1]))
    # print('(left_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_top_normal_line[1]) = ', (left_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * left_fit[0] * (left_fit[2] - center_fit_top_normal_line[1]))
    # print('(right_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_bot_normal_line[1]) = ', (right_fit[1] - center_fit_bot_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_bot_normal_line[1]))
    # print('(right_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_mid_normal_line[1]) = ', (right_fit[1] - center_fit_mid_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_mid_normal_line[1]))
    # print('(right_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_top_normal_line[1]) = ', (right_fit[1] - center_fit_top_normal_line[0]) ** 2 - 4 * right_fit[0] * (right_fit[2] - center_fit_top_normal_line[1]))
    bot_offset = ((normal_bot_left_contactx - normal_bot_right_contactx) ** 2 + (normal_bot_left_contacty - normal_bot_right_contacty) ** 2) ** 0.5 * PIXEL_CONVERSION_RATE
    mid_offset = ((normal_mid_left_contactx - normal_mid_right_contactx) ** 2 + (normal_mid_left_contacty - normal_mid_right_contacty) ** 2) ** 0.5 * PIXEL_CONVERSION_RATE
    top_offset = ((normal_top_left_contactx - normal_top_right_contactx) ** 2 + (normal_top_left_contacty - normal_top_right_contacty) ** 2) ** 0.5 * PIXEL_CONVERSION_RATE
    # print('normal_bot_left_contactx = ',normal_bot_left_contactx, 'normal_bot_left_contacty = ',normal_bot_left_contacty, 'normal_bot_right_contactx = ',normal_bot_right_contactx, 'normal_bot_right_contacty = ',normal_bot_right_contacty )
    # print('normal_bot_left_contactx = ', normal_bot_left_contactx, 'normal_bot_left_contacty = ', normal_bot_left_contacty, 'normal_bot_right_contactx = ', normal_bot_right_contactx, 'normal_bot_right_contacty = ', normal_bot_right_contacty)
    # print('normal_mid_left_contactx = ', normal_mid_left_contactx, 'normal_mid_left_contacty = ', normal_mid_left_contacty, 'normal_mid_right_contactx = ', normal_mid_right_contactx, 'normal_mid_right_contacty = ', normal_mid_right_contacty)
    # print('normal_top_left_contactx = ', normal_top_left_contactx, 'normal_top_left_contacty = ', normal_top_left_contacty, 'normal_top_right_contactx = ', normal_top_right_contactx, 'normal_top_right_contacty = ', normal_top_right_contacty)
    # print('bot_offset = ', bot_offset * PIXEL_CONVERSION_RATE, 'mid_offset = ', mid_offset * PIXEL_CONVERSION_RATE, 'top_offset = ', top_offset * PIXEL_CONVERSION_RATE)

    # print('bot_line = ', center_fit_bot_normal_grad, ' * y + ', center_fitx_bot - center_fit_bot_normal_grad * (PT_view_height))

    if (bot_offset <= lower_offset) | (bot_offset >= upper_offset) | (np.isnan(bot_offset)):  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 3, next_frame_find_poly_already_fitted, _bot_line_overlaped or line_breakaway ::bot_offset = ', bot_offset, 'normal_bot_left_contactx = ', normal_bot_left_contactx, 'normal_bot_right_contactx = ', normal_bot_right_contactx)
        error = 3
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if (mid_offset <= lower_offset) | (mid_offset >= upper_offset) | (np.isnan(mid_offset)):  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 4, next_frame_find_poly_already_fitted, _mid_line_overlaped or line_breakaway :: mid_offset = ', mid_offset, 'normal_mid_left_contactx = ', normal_mid_left_contactx, 'normal_mid_right_contactx = ', normal_mid_right_contactx)
        error = 4
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if (top_offset <= lower_offset) | (top_offset >= upper_offset) | (np.isnan(top_offset)):  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 5, next_frame_find_poly_already_fitted, _top_line_overlaped or line_breakaway :: top_offset = ', top_offset, 'normal_top_left_contactx = ', normal_top_left_contactx, 'normal_top_right_contactx = ', normal_top_right_contactx)
        error = 5
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if left_fitx > (binary_warped_left.shape[1] / 2) or right_fitx < (
            binary_warped_right.shape[1] / 2):  # left_fitx > (binary_warped_left.shape[1] / 2 + road_width / 2 / PIXEL_CONVERSION_RATE) or right_fitx < (binary_warped_right.shape[1] / 2 - road_width / 2 / PIXEL_CONVERSION_RATE):  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 6, next_frame_find_poly_already_fitted, driving_on_lane(deviation>(road_width/2) :: deviation = ', (binary_warped_right.shape[1] // 2 - (right_fitx + left_fitx) / 2) * PIXEL_CONVERSION_RATE, '  road_width/2 = ', road_width / 2)
        error = 6
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if abs(center_fit_deg_ave) >= 60:
        print('error = 7, next_frame_find_poly_already_fitted, lane_deg_ave is over than 55 :: center_fit_deg_ave = ', center_fit_deg_ave)
        error = 7
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if abs(top_offset - bot_offset) >= 1:  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 8, next_frame_find_poly_already_fitted, top_lane_unbalance :: top_offset - bot_offset = ', abs(top_offset - bot_offset))
        error = 8
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if abs(mid_offset - bot_offset) >= 0.7:  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 9, next_frame_find_poly_already_fitted, mid_lane_unbalance :: mid_offset - bot_offset = ', abs(mid_offset - bot_offset))
        error = 9
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    if (np.isnan(bot_offset)) | (np.isnan(mid_offset)) | (np.isnan(top_offset)):  # 추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 10, next_frame_find_poly_already_fitted, offset_is_nan :: mid_offset - bot_offset = ')
        error = 10
        return [], [], 0, 0, 0, [], error, False, False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

    error = 0

    # Create an image to draw on and an image to show the selection window
    out_img_left = np.dstack((binary_warped_left, binary_warped_left, binary_warped_left)) * 255
    out_img_right = np.dstack((binary_warped_right, binary_warped_right, binary_warped_right)) * 255
    # Color in left and right line pixels
    out_img_left[lefty, leftx] = [255, 0, 0]
    out_img_right[righty, rightx] = [0, 0, 255]

    out_img_half_sum = np.copy(zero_3)
    out_img_half_sum[:, : out_img_half_sum.shape[1] // 2, :] = out_img_left[:, : out_img_left.shape[1] // 2, :]
    out_img_half_sum[:, out_img_half_sum.shape[1] // 2:, :] = out_img_right[:, out_img_right.shape[1] // 2:, :]
    # cv2.imshow('red_blue',out_img_half_sum)
    result = None
    if add_debug_image:
        # debug_images.append(out_img_left)
        result, window_img_left, window_img_right, window_img_left_submargin, window_img_right_submargin = get_already_fit_image_plot(out_img_half_sum, left_fit, right_fit, center_fit, road_width, PIXEL_CONVERSION_RATE, roi_limit_M, roi_limit_L, roi_limit_R, zero_1, zero_3, PT_view_height,
                                                                                                                                      normal_mid_left_contactx, normal_mid_right_contactx, normal_mid_left_contacty, normal_mid_right_contacty, normal_top_left_contactx, normal_top_right_contactx,
                                                                                                                                      normal_top_left_contacty, normal_top_right_contacty, is_plot=is_plot,
                                                                                                                                      margin=margin)

        debug_images.append(result)
        debug_images.append(window_img_left)
        debug_images.append(window_img_right)
        debug_images.append(window_img_left_submargin)
        debug_images.append(window_img_right_submargin)
        debug_images.append(out_img_half_sum)

    left_curverad, right_curverad = get_radius_of_curvature(left_fit, right_fit, out_img_left, ym_per_pix)
    deviation = get_vehicle_deviation(out_img_left, left_fit, right_fit, PIXEL_CONVERSION_RATE)

    return left_fit, right_fit, left_curverad, right_curverad, deviation, debug_images, error, following_left_lane, following_right_lane, top_offset, mid_offset, bot_offset, left_fit_deg_ave, center_fit_deg_ave, right_fit_deg_ave, roi_limit_L, roi_limit_M, roi_limit_R, center_fit_mid_normal_grad, roi_limit_bump


def get_radius_of_curvature(left_fit_cr, right_fit_cr, image_for_size, ym_per_pix):
    plot_yyy = np.linspace(0, image_for_size.shape[0] - 1, image_for_size.shape[0])

    # Define conversions in x and y from pixels space to meters
    # ym_per_pix = 30 / 720  # meters per pixel in y dimension
    # xm_per_pix = 3.7 / 700  # meters per pixel in x dimension
    y_eval = np.max(plot_yyy)

    # Fit new polynomials to x,y in world space
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    # Now our radius of curvature is in meters
    return left_curverad, right_curverad


def get_vehicle_deviation(img, left_fit, right_fit, PIXEL_CONVERSION_RATE):
    image_bottom_pixel = img.shape[0] - 1

    bottom_x_position_left_lane = left_fit[0] * (image_bottom_pixel ** 2) + left_fit[1] * (image_bottom_pixel) \
                                  + left_fit[2]
    bottom_x_position_right_lane = right_fit[0] * (image_bottom_pixel ** 2) + right_fit[1] * (image_bottom_pixel) \
                                   + right_fit[2]
    vehicle_offset = (bottom_x_position_left_lane + bottom_x_position_right_lane) / 2.0 - img.shape[1] / 2

    # convert pixels to real space
    return vehicle_offset * PIXEL_CONVERSION_RATE


def get_already_fit_image_plot(out_img_half_sum, left_fit, right_fit, center_fit, road_width, PIXEL_CONVERSION_RATE, roi_limit_M, roi_limit_L, roi_limit_R, zero_1, zero_3, PT_view_height, normal_mid_left_contactx, normal_mid_right_contactx, normal_mid_left_contacty, normal_mid_right_contacty,
                               normal_top_left_contactx, normal_top_right_contactx, normal_top_left_contacty, normal_top_right_contacty, margin=50, is_plot=True):
    # Generate x and y values for plotting

    ploty_L = np.linspace(roi_limit_L, PT_view_height - 1, PT_view_height - roi_limit_L)
    ploty_R = np.linspace(roi_limit_R, PT_view_height - 1, PT_view_height - roi_limit_R)
    ploty_M = np.linspace(roi_limit_M, PT_view_height - 1, PT_view_height - roi_limit_M)
    ploty = np.linspace(0, PT_view_height - 1, PT_view_height)

    left_fitx = left_fit[0] * ploty_L ** 2 + left_fit[1] * ploty_L + left_fit[2]
    # sub_left_fitx = right_fit[0] * ploty_L ** 2 + right_fit[1] * ploty_L + right_fit[2] - road_width / PIXEL_CONVERSION_RATE
    right_fitx = right_fit[0] * ploty_R ** 2 + right_fit[1] * ploty_R + right_fit[2]

    left_top_y = roi_limit_L
    left_mid_y = (PT_view_height - 1 + roi_limit_L) // 2
    left_bot_y = PT_view_height - 1

    left_fit_mid_grad = 2 * left_fit[0] * left_mid_y + left_fit[1]

    if (left_fit_mid_grad == 0) | (abs(left_fit_mid_grad) < 0.01):
        move_y = 0
        move_x = road_width / PIXEL_CONVERSION_RATE
        print('unwarped 1 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
    else:
        left_fit_mid_normal_grad = -1 / left_fit_mid_grad
        left_fit_mid_normal_rad = np.arctan(left_fit_mid_normal_grad)
        if left_fit_mid_normal_rad < 0:
            move_y = -1 * road_width / PIXEL_CONVERSION_RATE * np.cos(left_fit_mid_normal_rad)
            move_x = -1 * road_width / PIXEL_CONVERSION_RATE * np.sin(left_fit_mid_normal_rad)
            print('unwarped 2 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)
        else:
            move_y = road_width / PIXEL_CONVERSION_RATE * np.cos(left_fit_mid_normal_rad)
            move_x = road_width / PIXEL_CONVERSION_RATE * np.sin(left_fit_mid_normal_rad)
            print('unwarped 3 :: move_y, move_x = ', move_y, ' ', move_x, '::', (move_y ** 2 + move_x ** 2) ** 0.5 * PIXEL_CONVERSION_RATE)

    sub_right_fit = [left_fit[0], left_fit[1] - 2 * left_fit[0] * move_y, left_fit[0] * move_y ** 2 - left_fit[1] * move_y + left_fit[2] + move_x]

    # left_fit_mid_normal_line = [left_fit_mid_normal_grad, -1 * left_fit_mid_normal_grad * mid_y + left_fit[0] * mid_y ** 2 + left_fit[1] * mid_y + left_fit[2]]
    # left_fit_mid_normal_linex = left_fit_mid_normal_line[0] * ploty + left_fit_mid_normal_line[1]
    # print('left_fit_mid_normal_line[0] * mid_y + left_fit_mid_normal_line[1] = ',left_fit_mid_normal_line[0] * mid_y + left_fit_mid_normal_line[1])

    sub_right_fitx = sub_right_fit[0] * ploty_L ** 2 + sub_right_fit[1] * ploty_L + sub_right_fit[2]

    center_fitx = center_fit[0] * ploty_M ** 2 + center_fit[1] * ploty_M + center_fit[2]
    print('road_width1', road_width)

    # # Generate a polygon to illustrate the search window area
    # # And recast the x and y points into usable format for cv2.fillPoly()
    # stack -> v(row), h(column), d(depth)
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty_L]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty_L])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty_R]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty_R])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    margin2 = round(0.3 / PIXEL_CONVERSION_RATE)
    left_line_window1_submargin = np.array([np.transpose(np.vstack([left_fitx - margin2, ploty_L]))])
    left_line_window2_submargin = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin2, ploty_L])))])
    left_line_pts_submargin = np.hstack((left_line_window1_submargin, left_line_window2_submargin))
    right_line_window1_submargin = np.array([np.transpose(np.vstack([right_fitx - margin2, ploty_R]))])
    right_line_window2_submargin = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin2, ploty_R])))])
    right_line_pts_submargin = np.hstack((right_line_window1_submargin, right_line_window2_submargin))

    # # Draw the lane onto the warped blank image
    window_img_full = np.copy(zero_3)
    window_img_left = np.copy(zero_1)
    window_img_right = np.copy(zero_1)
    window_img_left_submargin = np.copy(zero_1)
    window_img_right_submargin = np.copy(zero_1)
    cv2.fillPoly(window_img_full, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img_full, np.int_([right_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img_left, np.int_([left_line_pts]), (1))
    cv2.fillPoly(window_img_right, np.int_([right_line_pts]), (1))
    cv2.fillPoly(window_img_left_submargin, np.int_([left_line_pts_submargin]), (1))
    cv2.fillPoly(window_img_right_submargin, np.int_([right_line_pts_submargin]), (1))  # 필요하면(간소화 등) 특정 경우에만 코드가 돌도록 수정
    window_img_left = np.uint8(window_img_left)
    window_img_right = np.uint8(window_img_right)
    window_img_left_submargin = np.uint8(window_img_left_submargin)
    window_img_right_submargin = np.uint8(window_img_right_submargin)
    result = cv2.addWeighted(out_img_half_sum, 1, window_img_full, 0.3, 0)

    for pts, index in enumerate(left_fitx):
        cv2.circle(result, ((int)(left_fitx[pts]), (int)(ploty_L[pts])), 3, (255, 0, 255), -1)

    for pts, index in enumerate(right_fitx):
        cv2.circle(result, ((int)(right_fitx[pts]), (int)(ploty_R[pts])), 3, (255, 0, 255), -1)

    for pts, index in enumerate(center_fitx):
        cv2.circle(result, ((int)(center_fitx[pts]), (int)(ploty_M[pts])), 1, (0, 155, 255), -1)

    for pts, index in enumerate(sub_right_fitx):
        cv2.circle(result, ((int)(sub_right_fitx[pts]), (int)(ploty_L[pts])), 3, (0, 255, 255), -1)

    # for pts, index in enumerate(left_fit_mid_normal_linex):
    #    cv2.circle(result, ((int)(left_fit_mid_normal_linex[pts]), (int)(ploty[pts])), 3, (0, 0, 255), -1)
    cv2.circle(result, ((int)(normal_mid_left_contactx), (int)(normal_mid_left_contacty)), 3, (0, 0, 255), -1)
    cv2.circle(result, ((int)(normal_mid_right_contactx), (int)(normal_mid_right_contacty)), 3, (0, 0, 255), -1)
    cv2.circle(result, ((int)(normal_top_left_contactx), (int)(normal_top_left_contacty)), 3, (0, 0, 255), -1)
    cv2.circle(result, ((int)(normal_top_right_contactx), (int)(normal_top_right_contacty)), 3, (0, 0, 255), -1)

    if is_plot:
        plt.xlim(0, 1280)
        plt.ylim(720, 0)
        plt.imshow(result)

    return result, window_img_left, window_img_right, window_img_left_submargin, window_img_right_submargin


def plot_lanes_unwrap(viewsize, left_poly, right_poly, left_turn, right_turn, left_yellow_lane, right_yellow_lane, undistorted_in_rgb, M_INV, stopx_base, roi_limit_M, roi_limit_L, roi_limit_R, originalsize, dst_pts, is_plot=False, add_debug_image=True):
    debug_images = []

    if stopx_base > viewsize[1] / 14:
        stopx_limit = int(stopx_base - viewsize[1] / 14)
    else:
        stopx_limit = 0

    if roi_limit_M > viewsize[1] / 14:
        roi_limit_M = int(roi_limit_M - viewsize[1] / 14)
    else:
        roi_limit_M = 0

    # Create an image to draw the lines on
    color_warp = np.zeros((viewsize[1], viewsize[0], 3)).astype(np.uint8)
    # color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    # color_stop_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    color_stop_warp, _, _ = cv2.split(color_warp)
    center_fit = np.add(left_poly, right_poly) / 2

    plot_yyy = np.linspace(stopx_limit, viewsize[1] - 1, viewsize[1] - stopx_limit)
    plot_yyy2 = np.linspace(roi_limit_M, viewsize[1] - 1, viewsize[1] - roi_limit_M)

    #     pdb.set_trace()
    left_fitxxx = left_poly[0] * plot_yyy ** 2 + left_poly[1] * plot_yyy + left_poly[2]
    right_fitxxx = right_poly[0] * plot_yyy ** 2 + right_poly[1] * plot_yyy + right_poly[2]
    center_fitxxx = center_fit[0] * plot_yyy ** 2 + center_fit[1] * plot_yyy + center_fit[2]
    left_stop_fitxxx = left_poly[0] * plot_yyy2 ** 2 + left_poly[1] * plot_yyy2 + left_poly[2] + 15
    right_stop_fitxxx = right_poly[0] * plot_yyy2 ** 2 + right_poly[1] * plot_yyy2 + right_poly[2] - 15

    right_endxxx = np.copy(plot_yyy)
    left_endxxx = np.copy(plot_yyy)
    right_endxxx[:] = viewsize[0] - 1
    left_endxxx[:] = 0

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitxxx, plot_yyy]))])
    pts_left2 = np.array([np.flipud(np.transpose(np.vstack([left_fitxxx, plot_yyy])))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitxxx, plot_yyy])))])
    pts = np.hstack((pts_left, pts_right))
    pts_left_end = np.array([np.transpose(np.vstack([left_endxxx, plot_yyy]))])
    pts_right_end = np.array([np.transpose(np.vstack([right_endxxx, plot_yyy]))])
    pts_left_end_area = np.hstack((pts_left_end, pts_left2))
    pts_right_end_area = np.hstack((pts_right, pts_right_end))
    pts_stop_left = np.array([np.transpose(np.vstack([left_stop_fitxxx, plot_yyy2]))])
    pts_stop_right = np.array([np.flipud(np.transpose(np.vstack([right_stop_fitxxx, plot_yyy2])))])
    pts_stop = np.hstack((pts_stop_left, pts_stop_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_stop_warp, np.int_([pts_stop]), (1))
    # color_stop_warp = cv2.cvtColor(color_stop_warp, cv2.COLOR_BGR2GRAY)

    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    if left_turn:
        cv2.fillPoly(color_warp, np.int_([pts_left_end_area]), (255, 0, 0))
    if right_turn:
        cv2.fillPoly(color_warp, np.int_([pts_right_end_area]), (255, 0, 0))
    if left_yellow_lane:
        cv2.fillPoly(color_warp, np.int_([pts_left_end_area]), (0, 0, 255))
    if right_yellow_lane:
        cv2.fillPoly(color_warp, np.int_([pts_right_end_area]), (0, 0, 255))
    for pts, index in enumerate(center_fitxxx):
        cv2.circle(color_warp, ((int)(center_fitxxx[pts]), (int)(plot_yyy[pts])), 2, (0, 155, 255), -1)
    dot = center_fit[0] * (viewsize[1] - 15) ** 2 + center_fit[1] * (viewsize[1] - 15) + center_fit[2]
    cv2.circle(color_warp, ((int)(dot), (int)(viewsize[1] - 15)), 2, (255, 0, 0), -1)

    if add_debug_image: debug_images.append(color_warp)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, M_INV, (originalsize))
    # cv2.imshow('newwarp',newwarp)
    # cv2.imshow('color_warp', color_warp)
    if add_debug_image: debug_images.append(newwarp)

    # Combine the result with the original image
    result = cv2.addWeighted(undistorted_in_rgb, 1, newwarp, 0.3, 0)

    # dot = center_fitxxx[ viewsize[1] - 1]
    dot_diff = dot - viewsize[0] // 2
    dot_diff_inv = (dot_diff + 1) / (dst_pts[3, 0] - dst_pts[0, 0] + 20) * originalsize[0]
    cv2.line(result, (int(originalsize[0] // 2 - 1), int((originalsize[1] / 2 + (originalsize[1] / 2 // 9) * 7))), (int(originalsize[0] // 2 + dot_diff_inv), int(originalsize[1] / 2 + (originalsize[1] / 2 // 9) * 7)),
             [0, 0, 255], 3)
    cv2.line(result, (int(originalsize[0] // 2 - 1), int((originalsize[1] / 2 + (originalsize[1] / 2 // 9) * 7))), (int(originalsize[0] // 2 - 1), int(originalsize[1] - 1)), [0, 0, 255], 3)

    # if is_plot:
    # plot_two_images(color_warp, result, plot_diff=False)
    # plot_two_images(newwarp, result, plot_diff=False)

    if add_debug_image: debug_images.append(result)

    return color_stop_warp, color_warp, result, debug_images


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def PTtrans_and_Circle(video_copy1, video_copy2, src_pts_yfix, dst_pts, viewsize):
    print('p2_what_happen2_1')
    cv2.circle(video_copy2, tuple(src_pts_yfix[0]), 10, (0, 0, 255), 3)  # 변환하는 위치를 원본 이미지에 표시
    cv2.circle(video_copy2, tuple(src_pts_yfix[1]), 10, (0, 127, 255), 3)  # 빨주노초, 왼쪽 아래부터 시계방향
    cv2.circle(video_copy2, tuple(src_pts_yfix[2]), 10, (0, 255, 255), 3)
    cv2.circle(video_copy2, tuple(src_pts_yfix[3]), 10, (0, 255, 0), 3)

    print(video_copy1.shape , '-', video_copy1.dtype , '-', np.max(video_copy1), '-', np.min(video_copy1))
    print(video_copy2.shape , '-', video_copy2.dtype , '-', np.max(video_copy2), '-', np.min(video_copy2))
    perspective_m = cv2.getPerspectiveTransform(src_pts_yfix, dst_pts)

    video_copy1_PTtrans = cv2.warpPerspective(video_copy1, perspective_m, (viewsize[0], viewsize[1]), flags=cv2.INTER_LINEAR)  # 변환된 hsv 영상 출력 영역 (dst_pts 랑 똑같이 맞춰주면 됨)
    #video_copy1_PTtrans = cv2.warpPerspective(video_copy2, perspective_m, (viewsize[0],viewsize[1]))  # circle 포함
    #video_copy1_PTtrans = np.copy(video_copy1)                                      # 투상변환 적용 안하기
    # cv2.imshow('video_copy1_PTtrans', video_copy1_PTtrans)  #변환 이미지 출력
    return video_copy1_PTtrans, video_copy2


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ역투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def PTtrans_Reverse(video_window, src_pts_yfix, dst_pts, originalsize):
    perspective_m_resvers = cv2.getPerspectiveTransform(dst_pts, src_pts_yfix)
    video_window_PTtrans_reverse = cv2.warpPerspective(video_window, perspective_m_resvers, originalsize, flags=cv2.INTER_LINEAR)
    return video_window_PTtrans_reverse


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ역투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ블러필터, 정규화 적용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Blur_and_Normalize(video_copy1_PTtrans):
    video_copy1_PTtrans_blur = cv2.GaussianBlur(video_copy1_PTtrans, ksize=(3, 3), sigmaX=0.0)  # 투상변환 + 블러

    hsv = cv2.cvtColor(video_copy1_PTtrans_blur, cv2.COLOR_BGR2HSV)  # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡhsv
    h, s, v = cv2.split(hsv)
    v2 = np.copy(v)
    s2 = np.copy(s)
    cv2.normalize(v, v2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    cv2.normalize(s, s2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    # hsv2 = cv2.merge([h, s2, v2])
    hsv2 = cv2.merge([h, s, v2])

    video_copy1_PTtrans_blur_normalize4hsv = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)  # 투상변환 + 블러 + hsv정규화

    # yCrCv = cv2.cvtColor(video_copy1_PTtrans_blur, cv2.COLOR_BGR2YCrCb)    #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡyCrCv
    # y, Cr, Cv = cv2.split(yCrCv)
    # y2 = np.copy(y)
    # cv2.normalize(y, y2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    # yCrCv2 = cv2.merge([y2, Cr, Cv])

    # video_copy1_PTtrans_blur_normalize4yCrCv = cv2.cvtColor(yCrCv2, cv2.COLOR_YCrCb2BGR) # 투상변환 + 블러 + yCrCb정규화

    # gray = cv2.cvtColor(video_copy1_PTtrans_blur, cv2.COLOR_BGR2GRAY)
    # gray2 = np.copy(gray)
    # cv2.normalize(gray, gray2, 0, 255, cv2.NORM_MINMAX)  # 정규화
    # video_copy1_PTtrans_blur_normalize4gray = cv2.cvtColor(gray2, cv2.COLOR_GRAY2BGR) #투상변환 + 블러 + gray정규화       #야간 도로에서 테스트 해보자 (그레이스케일 변환 후 정규화 해서 이진화하기)
    return video_copy1_PTtrans_blur_normalize4hsv


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ블러필터, 정규화 적용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV Blue Yellow 추출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_img, upper_img):
    # video_adapt = np.copy(video_copy1_PTtrans_blur_normalize4hsv)  # hsv정규화 영상 적용
    ##video_adapt = np.copy(video_copy1_PTtrans_blur_normalize4yCrCv) #yCrCv정규화 영상 적용
    hsv_change = cv2.cvtColor(video_copy1_PTtrans_blur_normalize4hsv, cv2.COLOR_BGR2HSV)

    img_mask = cv2.inRange(hsv_change, lower_img, upper_img)
    # print('aaa',img_mask.dtype,'-',img_mask.shape, '-', np.max(img_mask), '-', np.min(img_mask))
    kernel_img = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    opening_img = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, kernel_img, iterations=2)
    closing_img = cv2.morphologyEx(opening_img, cv2.MORPH_CLOSE, kernel_img, iterations=2)
    img_mask_area_detect_morph_rgb = np.uint8(closing_img / 255)
    img_detect_morph_brg = cv2.bitwise_and(video_copy1_PTtrans_blur_normalize4hsv, video_copy1_PTtrans_blur_normalize4hsv, mask=closing_img)
    return img_detect_morph_brg, img_mask_area_detect_morph_rgb
    # blue_mask = cv2.inRange(hsv_change, lower_blue, upper_blue)
    # blue_mask_area_detect = cv2.bitwise_and(white, white, mask=blue_mask)
    # kernel_blue = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    # opening_blue = cv2.morphologyEx(blue_mask_area_detect, cv2.MORPH_OPEN, kernel_blue, iterations=2)
    # closing_blue = cv2.morphologyEx(opening_blue, cv2.MORPH_CLOSE, kernel_blue, iterations=2)
    # blue_mask_area_detect_morph = np.copy(closing_blue) #-간소화 가능
    # blue_mask_area_detect_morph_hsv = cv2.cvtColor(closing_blue, cv2.COLOR_BGR2HSV)
    # blue_mask_morph = cv2.inRange(blue_mask_area_detect_morph_hsv, (0,0,212), (131,255,255))
    # blue_detect_morph_brg = cv2.bitwise_and(video_adapt, video_adapt, mask=blue_mask_morph)

    # cv2.imshow('yellow_detect_morph_brg', yellow_detect_morph_brg)
    # color_brg = cv2.bitwise_or(yellow_detect_morph_brg, blue_detect_morph_brg)
    # color_brg_hsv = cv2.cvtColor(color_brg, cv2.COLOR_BGR2HSV)


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV Blue Yellow 추출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이진화ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def White_binary(video_copy1_Gray, white_tresh):
    # ㅡㅡㅡㅡㅡㅡㅡtest용ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    # video_binary_check = np.copy(gray2) # gray 정규화 영상 적용(gray)
    # ret_b, binary_detect_check = cv2.threshold(video_binary_check, 205, 255, cv2.THRESH_BINARY)
    # kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    # closing_binary_check = cv2.morphologyEx(binary_detect_check, cv2.MORPH_CLOSE, kernel, iterations=2)
    # opening_binary_check = cv2.morphologyEx(closing_binary_check, cv2.MORPH_OPEN, kernel, iterations=2)
    # ㅡㅡㅡㅡㅡㅡㅡtest용ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ #야간 도로에서 테스트 해보자

    # video_binary = np.copy(video_copy1_PTtrans_blur_normalize4yCrCv) # yCrCv 정규화 영상 적용
    # video_binary = np.copy(video_copy1_PTtrans_blur_normalize4hsv)  # hsv 정규화 영상 적용
    # video_binary = np.copy(gray2) # gray 정규화 영상 적용(GRAY)
    # gray_change = cv2.cvtColor(video_copy1_PTtrans_blur_normalize4hsv, cv2.COLOR_BGR2GRAY)
    ret_b, binary_detect = cv2.threshold(video_copy1_Gray, white_tresh[0], white_tresh[1], cv2.THRESH_BINARY)
    # ret_b, binary_detect = cv2.threshold(video_binary, 205, 255, cv2.THRESH_BINARY) # gray 정규화 영상 적용할 때
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    closing_binary = cv2.morphologyEx(binary_detect, cv2.MORPH_CLOSE, kernel, iterations=2)
    opening_binary = cv2.morphologyEx(closing_binary, cv2.MORPH_OPEN, kernel, iterations=2)

    # binary_detect_morph_gray = np.copy(opening_binary) #비활성화 해도 됨
    binary_detect_morph_brg = np.uint8(opening_binary / 255)
    return binary_detect_morph_brg


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이진화ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ최종 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Final_Bynary_Mix(color_brg, binary_detect_morph_brg, blue_mask_area_detect_morph, yellow_mask_area_detect_morph):
    binary_over = np.zeros_like(binary_detect_morph_brg)
    binary_over[((blue_mask_area_detect_morph == 1) | (yellow_mask_area_detect_morph == 1)) & (binary_detect_morph_brg == 1)] = 1
    binary_seperate = cv2.absdiff(binary_over, binary_detect_morph_brg)
    binary_seperate_3 = np.uint8(np.dstack((binary_seperate, binary_seperate, binary_seperate)) * 255)
    final_line_brg = cv2.bitwise_or(color_brg, binary_seperate_3)
    return final_line_brg


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ최종 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV lower upper 값 추적용 히스토그램ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def HSV_Tresh_Trace(input_brg_image):
    input_hsv_image = cv2.cvtColor(input_brg_image, cv2.COLOR_BGR2HSV)

    plt.clf()
    histColor = ('b', 'g', 'r')
    binX = np.arange(32) * 8
    plt.ylim(0, 4000)
    for i in range(3):
        hist = cv2.calcHist(images=[input_hsv_image], channels=[i], mask=None, histSize=[256 / 8], ranges=[0, 256])
        plt.plot(binX, hist, color=histColor[i])
    plt.show()


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV lower upper 값 추적용 히스토그램ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ벼열 변환 및 dtype 변환ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def List_to_Array_and_Type(out_img2, binary_detect_morph_brg):
    out_img2_array = np.array(out_img2)
    # print(out_img2_array.shape,'-',out_img2_array.dtype, '-', np.min(out_img2_array) , '-' , np.max(out_img2_array))
    out_img2_shape = out_img2_array.reshape(binary_detect_morph_brg.shape)
    # print(out_img2_shape.shape, '-', out_img2_shape.dtype, '-', np.min(out_img2_shape), '-', np.max(out_img2_shape))
    out_img2_shape_uint8 = out_img2_shape.astype(np.uint8)
    return out_img2_shape_uint8


# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ벼열 변환 및 dtype 변환ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ원본 영상, 윈도우 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize):
    video_window = np.copy(bynary_and_red_blue_and_poly_area)
    video_window_PTtrans_reverse = PTtrans_Reverse(video_window, src_pts_yfix, dst_pts, originalsize)
    # cv2.imshow('video_window_PTtrans_reverse',video_window_PTtrans_reverse)
    video_window_PTtrans_reverse_gray = cv2.cvtColor(video_window_PTtrans_reverse, cv2.COLOR_BGR2GRAY)
    video_window_PTtrans_reverse_vinary = np.zeros_like(video_window_PTtrans_reverse_gray)
    video_window_PTtrans_reverse_vinary[(video_window_PTtrans_reverse_gray >= 1)] = 255
    video_reverse_diff = np.copy(video_copy1)
    video_reverse_diff[(video_window_PTtrans_reverse_vinary == 255)] = 0
    video_original_window_mix = cv2.add(video_reverse_diff, video_window_PTtrans_reverse)
    # cv2.imshow('video_original_window_mix', video_original_window_mix)
    return video_window_PTtrans_reverse, video_original_window_mix


def find_proper_transper_point(proper_size, sight_height, target_distance, road_width, margin, view_scale, focal_length_horizontal, focal_length_vertical, view_length1_h, view_length1_h_bot, view_length2_v, PIXEL_CONVERSION_RATE):
    angle_of_view_vertical_center = np.arctan(view_length2_v / focal_length_vertical)
    angle_of_view_horizontal_center = np.arctan(view_length1_h / focal_length_horizontal)
    angle_of_view_horizontal_bottom = np.arctan(np.cos(angle_of_view_vertical_center) * view_length1_h_bot / focal_length_horizontal)
    print('angle_of_view_horizontal_center = ', np.rad2deg(angle_of_view_horizontal_center), '  angle_of_view_vertical_center = ', np.rad2deg(angle_of_view_vertical_center), '   angle_of_view_horizontal_bottom = ', np.rad2deg(angle_of_view_horizontal_bottom))

    view_angle = np.arctan(target_distance / sight_height)
    print('view_angle = ', np.rad2deg(view_angle))
    loss_distance = sight_height * np.tan(view_angle - angle_of_view_vertical_center)
    print('loss_distance = ', loss_distance)
    Hypotenuse_distance = sight_height / np.cos(view_angle - angle_of_view_vertical_center)
    straight_distance = sight_height / np.cos(view_angle)

    window_face_bottom_length = Hypotenuse_distance * np.tan(angle_of_view_horizontal_bottom)
    print('window_face_bottom_length*2 = ', window_face_bottom_length * 2)
    ratio_window_target_road = road_width / (straight_distance * np.tan(angle_of_view_horizontal_center))
    ratio_window_target_margin = (road_width + margin) / (2 * straight_distance * np.tan(angle_of_view_horizontal_center))

    # window_face_target_road_width_length = ratio_window_target_road * window_face_bottom_length
    PT_view_width = np.int(round((road_width + margin) * 100 * view_scale))  # 현재 1미터당 100픽셀로 설정되어있다. 비율을 줄여서 연산 속도를 향상할 수도 있으나 그만큼 세밀함이 저하될 수 있다.
    PT_view_width_2 = np.int(round((road_width + margin) * 100))
    PT_view_height = np.int(round((target_distance - loss_distance) * 100 * view_scale))
    PT_view_height_2 = np.int(round((target_distance - loss_distance) * 100))
    print('PT_view_width = ', PT_view_width, '   PT_view_height = ', PT_view_height)
    print('PT_view_width_2 = ', PT_view_width_2, '   PT_view_height_2 = ', PT_view_height_2)
    print('view_real_area_height = ', PT_view_width_2 / 100, '   view_real_area_width = ', PT_view_height_2 / 100)

    road_margin_length_pixel = round(proper_size.shape[1] * ratio_window_target_margin)
    print('center_view_horizontal_center_length = ', (straight_distance * np.tan(angle_of_view_horizontal_center)))
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)
    target_road_margin_srt = np.array(
        [[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 20], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 20], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]],
        dtype=np.float32)
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1 - 70], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 90], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 90], [proper_size.shape[1] - 1, proper_size.shape[0] - 1 - 70]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_good.mp4
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 55], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 55], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_v1.mp4
    # target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 205], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 205], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #shool_road1.mp4
    dst_bot_pixel = np.int(round(2 * window_face_bottom_length / (road_width + margin) * PT_view_width))
    target_road_margin_dst = np.array([[(PT_view_width - dst_bot_pixel) / 2 - 1, PT_view_height - 1], [0, 0], [PT_view_width - 1, 0], [(PT_view_width + dst_bot_pixel) / 2 - 1, PT_view_height - 1]], dtype=np.float32)

    start_focus_distance = (road_width / 2) / np.tan(angle_of_view_horizontal_center)
    start_zet_distance = (straight_distance - start_focus_distance) * sight_height / target_distance
    start_distance = sight_height * np.tan(view_angle - np.arctan(start_zet_distance / start_focus_distance))
    print('start_distance = ', start_distance)

    vertical_lane = np.array([[0, round((target_distance - start_distance) / PIXEL_CONVERSION_RATE) - 1], [PT_view_width - 1, round((target_distance - start_distance) / PIXEL_CONVERSION_RATE) - 1]], dtype=np.float32)
    straght_lane_left = np.array([[round(PT_view_width / 2 - road_width / 2 / PIXEL_CONVERSION_RATE - 1), PT_view_height - 1], [round(PT_view_width / 2 - road_width / 2 / PIXEL_CONVERSION_RATE - 1), 0]])
    straght_lane_right = np.array([[round(PT_view_width / 2 + road_width / 2 / PIXEL_CONVERSION_RATE - 1), PT_view_height - 1], [round(PT_view_width / 2 + road_width / 2 / PIXEL_CONVERSION_RATE - 1), 0]])
    return target_road_margin_srt, target_road_margin_dst, PT_view_width, PT_view_height, PT_view_width_2, PT_view_height_2, vertical_lane, straght_lane_left, straght_lane_right


def get_lane(new_image_set, error, left_fit, right_fit, following_left_lane, following_right_lane, target_left_pixel_percent, target_right_pixel_percent, full_line_target_percent, roi_limit_L, roi_limit_M, roi_limit_R, line_type, bump_basex, bump_height, left_turn, right_turn,
             error_pixel_percent_left, error_pixel_percent_right, total_error_left, total_error_right, total_error_half_sum, error_pixel_percent_yellow, error_pixel_percent_half_sum, white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, yellow_s_tresh_lower, PT_view_width,
             PT_view_height, loop, bynary_half_sum_pixel_percent, target_half_sum_pixel_percent, out, bynary_left_pixel_percent, bynary_right_pixel_percent, deviation, top_offset, mid_offset, bot_offset, tresh_factor_right, left_fit_deg_ave, center_fit_deg_ave, right_fit_deg_ave, stop_deviation,
             length_stop_histogram, max_stop_histogram, center_fit_mid_normal_grad, stopx_base, road_width, PIXEL_CONVERSION_RATE, lower_offset, upper_offset, zero_1, zero_3, viewsize, ym_per_pix, kp, ki, kd, M_INV, dst_pts, PT_view_width_2, PT_view_height_2, vertical_lane, straght_lane_left,
             straght_lane_right, src_pts_yfix):
    binary_all_left_gray = np.copy(new_image_set[0])
    binary_all_right_gray = np.copy(new_image_set[1])
    binary_all_half_sum_gray = np.copy(new_image_set[2])
    binary_detect_morph_brg_left = np.copy(new_image_set[3])
    binary_detect_morph_brg_right = np.copy(new_image_set[4])
    binary_detect_morph_brg_half_sum = np.copy(new_image_set[5])
    binary_detect_stop_morph_brg = np.copy(new_image_set[6])
    yellow_mask_area_detect_morph = np.copy(new_image_set[7])
    video_copy1 = np.copy(new_image_set[8])
    video_copy2 = np.copy(new_image_set[9])
    final_line_color = np.copy(new_image_set[10])
    video_copy1_PTtrans = np.copy(new_image_set[11])

    originalsize = (video_copy1.shape[1], video_copy1.shape[0])  # h, w, d = video.shape
    #print('p1_',originalsize)

    add_debug_image1 = True
    add_debug_image1_a = False
    add_debug_image2 = True
    add_debug_image2_a = False
    add_debug_image2_b = False
    add_debug_image3 = True
    is_debug = True
    error_loop = 0

    view_area = PT_view_width * PT_view_height

    # flaots.testset.layout.dim.append(MultiArrayDimension())
    # flaots.testset.layout.dim.append(MultiArrayDimension())
    # flaots.testset.layout.dim[0].label = "height"
    # flaots.testset.layout.dim[1].label = "width"
    # flaots.testset.layout.dim[0].size = 2
    # flaots.testset.layout.dim[1].size = 23
    # flaots.testset.layout.dim[0].stride = 2 * 23
    # flaots.testset.layout.dim[1].stride = 23
    # flaots.testset.layout.data_offset = 0
    # flaots.testset.data = np.zeros((46))
    #
    # dstride0 = flaots.testset.layout.dim[0].stride
    # dstride1 = flaots.testset.layout.dim[1].stride
    # offset = flaots.testset.layout.data_offset

    if error != 0:
        half_sum_pix_img = np.copy(binary_detect_morph_brg_half_sum)
        half_sum_pix_img[:, int(half_sum_pix_img.shape[1] // 2 - road_width / PIXEL_CONVERSION_RATE / 4 - 1):int(half_sum_pix_img.shape[1] // 2 + road_width / PIXEL_CONVERSION_RATE / 4 - 1)] = 0
        half_sum_pix_img[:, :int(half_sum_pix_img.shape[1] // 2 - (road_width / 2 + 1) / PIXEL_CONVERSION_RATE - 1)] = 0
        half_sum_pix_img[:, int(half_sum_pix_img.shape[1] // 2 + (road_width / 2 + 1) / PIXEL_CONVERSION_RATE - 1):] = 0

        bynary_half_sum_pixel_percent = np.sum(half_sum_pix_img) / (half_sum_pix_img.shape[0] * half_sum_pix_img.shape[1])
        target_half_sum_pixel_percent = full_line_target_percent * 3
        # white_tresh_lower_half_sum = white_tresh_lower_half * ((bynary_half_sum_pixel_percent + 0.5) / (target_half_sum_pixel_percent + 0.5))
        prev_error_pixel_percent_half_sum = error_pixel_percent_half_sum
        error_pixel_percent_half_sum = bynary_half_sum_pixel_percent - target_half_sum_pixel_percent
        total_error_half_sum = total_error_half_sum + error_pixel_percent_half_sum
        tresh_factor_half_sum = kp * error_pixel_percent_half_sum + ki * total_error_half_sum + kd * (error_pixel_percent_half_sum - prev_error_pixel_percent_half_sum)
        white_tresh_lower_half_sum = white_tresh_lower_half_sum * pow(2, tresh_factor_half_sum)
        if white_tresh_lower_half_sum >= 205: white_tresh_lower_half_sum = 205
        if white_tresh_lower_half_sum <= 157: white_tresh_lower_half_sum = 157

        white_tresh_lower_left = white_tresh_lower_half_sum
        white_tresh_lower_right = white_tresh_lower_half_sum

        print('white_tresh_lower_half_sum = ', white_tresh_lower_half_sum, '   bynary_half_sum_pixel_percent = ', bynary_half_sum_pixel_percent)
    # print('p1_len(left_fit) = ', len(left_fit))
    if len(left_fit) == 0:
        leftx_base, rightx_base, hist, error, inner_length = find_left_right_via_histogram(binary_all_half_sum_gray, road_width, PIXEL_CONVERSION_RATE, add_debug_image=False)  # 영상에서의 왼쪽 오른쪽 차선의 초기 위치를 뽑아내는 함수
        if inner_length <= (road_width - 1) / PIXEL_CONVERSION_RATE: stopx_base = 0

        if error == 0:
            left_fit, right_fit, debug_images1, error, following_left_lane, following_right_lane = find_ploy_fit_window_search(binary_all_half_sum_gray, leftx_base, rightx_base, following_left_lane, following_right_lane, road_width, PIXEL_CONVERSION_RATE, target_left_pixel_percent,
                                                                                                                               target_right_pixel_percent, full_line_target_percent, roi_limit_L, roi_limit_M, roi_limit_R, PT_view_height, stopx_base, nwindows=13,
                                                                                                                               margin=round(0.45 / PIXEL_CONVERSION_RATE), is_plot=False,
                                                                                                                               add_debug_image=add_debug_image1)  # 사각 영역을 만들어 차선을 탐지하고 추세선을 만드는 함수
            # if error == 0: #cv2.imshow('debug_images1', debug_images1[0])
            if (add_debug_image1) & (error == 0):
                add_debug_image1_a = 1
            if is_debug: print("stage 4 a.. done_1")

    if error == 0:
        left_fit, right_fit, lc, rc, deviation, debug_images2, error, following_left_lane, following_right_lane, top_offset, mid_offset, bot_offset, left_fit_deg_ave, center_fit_deg_ave, right_fit_deg_ave, roi_limit_L, roi_limit_M, roi_limit_R, center_fit_mid_normal_grad, roi_limit_bump = next_frame_find_poly_already_fitted(
            binary_all_left_gray, binary_all_right_gray, left_fit, right_fit, lower_offset,
            upper_offset, following_right_lane, following_left_lane,
            road_width, PIXEL_CONVERSION_RATE, error_loop, PT_view_height, stopx_base, zero_1, zero_3, line_type, bump_basex, bump_height, viewsize, ym_per_pix,
            margin=round(0.3 / PIXEL_CONVERSION_RATE), is_plot=False,
            add_debug_image=add_debug_image2)

        if error == 0:
            if add_debug_image2:
                add_debug_image2_a = 1
            if is_debug: print("stage 4 b.. done_2")

            if line_type == 3:
                binary_detect_morph_brg_right[roi_limit_bump[1]:roi_limit_bump[0]] = 0
                binary_detect_morph_brg_left[roi_limit_bump[1]:roi_limit_bump[0]] = 0
                # cv2.imshow('yellow_limit',binary_detect_morph_brg_right*255)

            bynary_window_image_left_area = cv2.bitwise_and(binary_detect_morph_brg_left, debug_images2[1])
            bynary_all_window_image_left_area = cv2.bitwise_and(binary_all_left_gray, debug_images2[1])
            bynary_all_window_image_left_area_2 = cv2.bitwise_and(binary_all_left_gray, debug_images2[3])
            bynary_window_image_right_area = cv2.bitwise_and(binary_detect_morph_brg_right, debug_images2[2])
            bynary_all_window_image_right_area = cv2.bitwise_and(binary_all_right_gray, debug_images2[2])
            bynary_all_window_image_right_area_2 = cv2.bitwise_and(binary_all_left_gray, debug_images2[4])

            # cv2.imshow('debug_images2[3]', debug_images2[3]*255)
            # cv2.imshow('debug_images2[4]', debug_images2[4]*255)

            bynary_left_pixel_percent = np.sum(bynary_window_image_left_area) / ((viewsize[0] - roi_limit_L - (roi_limit_bump[0] - roi_limit_bump[1])) * viewsize[1])
            bynary_right_pixel_percent = np.sum(bynary_window_image_right_area) / ((viewsize[0] - roi_limit_R - (roi_limit_bump[0] - roi_limit_bump[1])) * viewsize[1])

            dotted_line_target_percent = 0.0038
            full_line_target_percent = 0.01

            if left_turn == True:
                target_left_pixel_percent = dotted_line_target_percent
            else:
                target_left_pixel_percent = full_line_target_percent
            if right_turn == True:
                target_right_pixel_percent = dotted_line_target_percent
            else:
                target_right_pixel_percent = full_line_target_percent

            # left_gray, _, _ = cv2.split(bynary_window_image_left_area)
            # right_gray, _, _ = cv2.split(bynary_window_image_right_area)
            # cv2.imshow('bynary_window_image_right_area',bynary_window_image_right_area)
            left_lane_list = np.sum(bynary_all_window_image_left_area, axis=1) >= 1
            left_lane_list_blank = np.sum(bynary_all_window_image_left_area, axis=1) == 0
            right_lane_list = np.sum(bynary_all_window_image_right_area, axis=1) >= 1
            right_lane_list_blank = np.sum(bynary_all_window_image_right_area, axis=1) == 0
            # print('left_lane_list = ',left_lane_list)
            check_change = np.zeros((viewsize[1], viewsize[0], 3))

            for i in range(len(left_lane_list)):
                if left_lane_list[i] == True:
                    check_change[i, :, :] = 255

            # cv2.imshow('check_change', check_change)

            length_of_lane = 0
            length_of_lane_blank = 0
            last_lane = 0
            check_sibal = 0
            left_lane_change = 0
            right_lane_change = 0
            dotted_line_length = 2.5
            str_line = 6
            lane_storage = []

            a_L = 0
            b_L = 0
            a_R = 0
            b_R = 0

            print('len(left_lane_list) = ', len(left_lane_list))
            for i in range(len(left_lane_list)):
                length_of_lane = length_of_lane + left_lane_list[i]
                length_of_lane_blank = length_of_lane_blank + left_lane_list_blank[i]
                a_L = a_L + left_lane_list[i] * PIXEL_CONVERSION_RATE
                b_L = b_L + left_lane_list_blank[i] * PIXEL_CONVERSION_RATE
                if last_lane != left_lane_list[i]:
                    check_sibal = check_sibal + 1
                    print('sibal1 = ', check_sibal)
                last_lane = left_lane_list[i]
                if (check_sibal >= 4) & (length_of_lane_blank < 1.5 / PIXEL_CONVERSION_RATE):
                    length_of_lane_blank = 0
                    check_sibal = 0
                    print('left_noise_blank')
                if (check_sibal >= 4) & (length_of_lane < 1.5 / PIXEL_CONVERSION_RATE):
                    length_of_lane = 0
                    check_sibal = 0
                    print('left_noise_lane')
                if (check_sibal >= 1) & (left_lane_list[i] == 0) & (length_of_lane > dotted_line_length / PIXEL_CONVERSION_RATE) & (length_of_lane_blank > 2 / PIXEL_CONVERSION_RATE):
                    lane_storage.append(length_of_lane)
                    length_of_lane = 0
                    check_sibal = 0
                    print('left_have_append')
                if length_of_lane > str_line / PIXEL_CONVERSION_RATE:
                    left_turn = False
                    lane_storage = []
                    print('left_long_lane')
                    break
            if len(lane_storage) >= 1:
                if max(lane_storage) >= str_line / PIXEL_CONVERSION_RATE:
                    left_turn = False
                    print('left_long_lane2')
                    print('L_max(lane_storage) = ', max(lane_storage) * PIXEL_CONVERSION_RATE)
                elif (max(lane_storage) < str_line / PIXEL_CONVERSION_RATE):
                    left_turn = True
                    print('left_short_lane')
                    print('L_max(lane_storage) = ', max(lane_storage) * PIXEL_CONVERSION_RATE)

            print('a_L = ', a_L)
            print('b_L = ', b_L)
            check_sibal = 0
            length_of_lane = 0
            length_of_lane_blank = 0
            lane_storage = []
            for i in range(len(right_lane_list)):
                length_of_lane = length_of_lane + right_lane_list[i]
                length_of_lane_blank = length_of_lane_blank + right_lane_list_blank[i]
                a_R = a_R + left_lane_list[i] * PIXEL_CONVERSION_RATE
                b_R = b_R + left_lane_list_blank[i] * PIXEL_CONVERSION_RATE
                if last_lane != right_lane_list[i]:
                    check_sibal = check_sibal + 1
                    print('sibal2 = ', check_sibal)
                last_lane = right_lane_list[i]
                if (check_sibal >= 4) & (length_of_lane_blank < 1.5 / PIXEL_CONVERSION_RATE):
                    length_of_lane_blank = 0
                    check_sibal = 0
                    print('right_noise_blank')
                if (check_sibal >= 4) & (length_of_lane < 1.5 / PIXEL_CONVERSION_RATE):
                    length_of_lane = 0
                    check_sibal = 0
                    print('right_noise_lane')
                if (check_sibal >= 1) & (right_lane_list[i] == 0) & (length_of_lane > dotted_line_length / PIXEL_CONVERSION_RATE) & (length_of_lane_blank > 2 / PIXEL_CONVERSION_RATE):
                    lane_storage.append(length_of_lane)
                    check_sibal = 0
                    length_of_lane = 0
                    print('right_have_append')
                if length_of_lane > str_line / PIXEL_CONVERSION_RATE:
                    right_turn = False
                    lane_storage = []
                    print('right_long_lane = ', length_of_lane * PIXEL_CONVERSION_RATE)
                    break
            if len(lane_storage) >= 1:
                if max(lane_storage) >= str_line / PIXEL_CONVERSION_RATE:
                    right_turn = False
                    print('right_long_lane')
                    print('R_max(lane_storage) = ', max(lane_storage) * PIXEL_CONVERSION_RATE)
                elif (max(lane_storage) < str_line / PIXEL_CONVERSION_RATE):
                    right_turn = True
                    print('right_short_lane')
                    print('R_max(lane_storage) = ', max(lane_storage) * PIXEL_CONVERSION_RATE)

            print('a_R = ', a_R)
            print('b_R = ', b_R)

            if (roi_limit_L > (PT_view_height // 5) * 2) | (roi_limit_R > (PT_view_height // 5) * 2) | (line_type == 3):
                left_turn = False
                right_turn = False
                print('not_dotted_lane_by_roi_limit')

            # if left_lane_change >=1:left_turn = True
            # else: left_turn = False
            # if right_lane_change >=1:right_turn = True
            # else: right_turn = False

            # white_tresh_lower_left = white_tresh_lower_left * ((bynary_left_pixel_percent + 0.15) / (target_left_pixel_percent + 0.15))
            prev_error_pixel_percent_left = error_pixel_percent_left
            error_pixel_percent_left = bynary_left_pixel_percent - target_left_pixel_percent
            total_error_left = total_error_left + error_pixel_percent_left
            tresh_factor_left = kp * error_pixel_percent_left + ki * total_error_left + kd * (error_pixel_percent_left - prev_error_pixel_percent_left)
            white_tresh_lower_left = white_tresh_lower_left * pow(2, tresh_factor_left)
            if white_tresh_lower_left >= 205: white_tresh_lower_left = 205
            if white_tresh_lower_left <= 157: white_tresh_lower_left = 157

            # white_tresh_lower_right = white_tresh_lower_right * ((bynary_right_pixel_percent + 0.15) / (target_right_pixel_percent + 0.15))
            prev_error_pixel_percent_right = error_pixel_percent_right
            error_pixel_percent_right = bynary_right_pixel_percent - target_right_pixel_percent
            total_error_right = total_error_right + error_pixel_percent_right
            tresh_factor_right = kp * error_pixel_percent_right + ki * total_error_right + kd * (error_pixel_percent_right - prev_error_pixel_percent_right)
            white_tresh_lower_right = white_tresh_lower_right * pow(2, tresh_factor_right)
            if white_tresh_lower_right >= 205: white_tresh_lower_right = 205
            if white_tresh_lower_right <= 157: white_tresh_lower_right = 157

            white_tresh_lower_half_sum = (white_tresh_lower_left + white_tresh_lower_right) / 2

            print('error_pixel_percent_left =', error_pixel_percent_left, 'tresh_factor_left = ', tresh_factor_left, 'pow(2, tresh_factor_left) = ', pow(2, tresh_factor_left))
            print('error_pixel_percent_right =', error_pixel_percent_right, 'tresh_factor_right = ', tresh_factor_right, 'pow(2, tresh_factor_right) = ', pow(2, tresh_factor_right))
            add_debug_image2_b = True

            yellow_bynary_in_left_lane_area = cv2.bitwise_and(yellow_mask_area_detect_morph, debug_images2[1])
            yellow_bynary_in_right_lane_area = cv2.bitwise_and(yellow_mask_area_detect_morph, debug_images2[2])
            bynary_yellow_pixel_percent = np.sum(yellow_mask_area_detect_morph) / view_area
            yellow_bynary_in_left_lane_area_pixel_percent = np.sum(yellow_bynary_in_left_lane_area) / view_area
            yellow_bynary_in_right_lane_area_pixel_percent = np.sum(yellow_bynary_in_right_lane_area) / view_area

            # cv2.imshow('yellow_bynary_in_right_lane_area',cv2.cvtColor(yellow_bynary_in_right_lane_area*255,cv2.COLOR_GRAY2BGR))
            target_yellow_pixel_percent = 0.02

            prev_error_pixel_percent_yellow = error_pixel_percent_yellow
            error_pixel_percent_yellow = bynary_yellow_pixel_percent - target_yellow_pixel_percent
            total_error_yellow = total_error_left + error_pixel_percent_yellow
            tresh_factor_yellow = kp * error_pixel_percent_yellow + ki * total_error_yellow + kd * (error_pixel_percent_yellow - prev_error_pixel_percent_yellow)
            yellow_s_tresh_lower = yellow_s_tresh_lower * pow(2, tresh_factor_yellow)
            if yellow_s_tresh_lower >= 85: yellow_s_tresh_lower = 85
            if yellow_s_tresh_lower <= 45: yellow_s_tresh_lower = 45

            if (yellow_bynary_in_left_lane_area_pixel_percent >= target_left_pixel_percent / 4):
                left_yellow_lane = True
            else:
                left_yellow_lane = False
            if (yellow_bynary_in_right_lane_area_pixel_percent >= target_right_pixel_percent / 4):
                right_yellow_lane = True
            else:
                right_yellow_lane = False

            print('yellow_bynary_in_left_lane_area_pixel_percent = ', yellow_bynary_in_left_lane_area_pixel_percent, 'yellow_bynary_in_right_lane_area_pixel_percent = ', yellow_bynary_in_right_lane_area_pixel_percent)

            roi_limit_L_2 = roi_limit_L
            if roi_limit_L_2 >= viewsize[1] / 5 * 4 - (viewsize[1] // 5) * 3 - 1: roi_limit_L_2 = int(viewsize[1] / 5 * 4 - (viewsize[1] // 5) * 3 - 1)
            roi_limit_R_2 = roi_limit_R
            if roi_limit_R_2 >= viewsize[1] / 5 * 4 - (viewsize[1] // 5) * 3 - 1: roi_limit_R_2 = int(viewsize[1] / 5 * 4 - (viewsize[1] // 5) * 3 - 1)

            # bynary_left_upside_pixel_percent = np.sum(bynary_all_window_image_left_area_2[roi_limit_L_2 : int((viewsize[1] // 5) * 3 - 1 + roi_limit_L_2), :]) / view_area
            # bynary_right_upside_pixel_percent = np.sum(bynary_all_window_image_right_area_2[roi_limit_R_2 : int((viewsize[1] // 5) * 3 - 1 + roi_limit_R_2), :]) / view_area
            # cv2.imshow('1',bynary_window_image_right_area*255)
            # cv2.imshow('2',bynary_window_image_right_area[roi_limit_L_2 : int((viewsize[1] // 5) * 3 - 1 + roi_limit_L_2), :]*255)
            # if bynary_left_upside_pixel_percent < full_line_target_percent / 6.5 and bynary_right_upside_pixel_percent > target_right_pixel_percent / 2:
            #     following_right_lane = True
            #     following_left_lane = False
            #     print('following_right_lane = ', following_right_lane, '****', '  bynary_left_upside_pixel_percent = ', bynary_left_upside_pixel_percent)
            # elif bynary_right_upside_pixel_percent < full_line_target_percent / 6.5 and bynary_left_upside_pixel_percent > target_left_pixel_percent / 2:
            #     following_right_lane = False
            #     following_left_lane = True
            #     print('following_left_lane = ', following_left_lane, '****', '  bynary_right_upside_pixel_percent= ', bynary_right_upside_pixel_percent)
            # else:
            #     following_right_lane = False
            #     following_left_lane = False
            #     print('not_line_following  :: bynary_left_upside_pixel_percent , bynary_right_upside_pixel_percent = ', bynary_left_upside_pixel_percent, ' , ',bynary_right_upside_pixel_percent)

            # bynary_all_window_image_left_area_2[:, int(viewsize[0] // 2 - road_width / PIXEL_CONVERSION_RATE / 4 - 1):int(viewsize[0] // 2 + road_width / PIXEL_CONVERSION_RATE / 4 - 1)] = 0
            # bynary_all_window_image_left_area_2[:, :int(viewsize[0] // 2 - (road_width / 2 + 1) / PIXEL_CONVERSION_RATE - 1)] = 0
            # bynary_all_window_image_left_area_2[:, int(viewsize[0] // 2 + (road_width / 2 + 1) / PIXEL_CONVERSION_RATE - 1):] = 0

            bynary_left_upside_pixel_percent = np.sum(bynary_all_window_image_left_area_2) / view_area
            bynary_right_upside_pixel_percent = np.sum(bynary_all_window_image_right_area_2) / view_area
            if (bynary_left_upside_pixel_percent <= target_left_pixel_percent / 5 * 2) & (bynary_right_pixel_percent > target_right_pixel_percent / 3 * 2):
                following_right_lane = True
                following_left_lane = False
                print('not follow window 1')
            elif (bynary_right_upside_pixel_percent <= target_right_pixel_percent / 5 * 2) & (bynary_left_pixel_percent > target_left_pixel_percent / 3 * 2):
                following_right_lane = False
                following_left_lane = True
                print('not follow window 2')
            else:
                following_right_lane = False
                following_left_lane = False
                print('not follow window 3 bynary_left_pixel_percent, bynary_right_pixel_percent = ', bynary_left_pixel_percent, ' , ', bynary_right_pixel_percent)

    # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ1차 로스 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

    # if error == 0:
    #     flaots.left_curv = lc
    #     flaots.right_curv = rc
    #     flaots.deviation = deviation
    #     flaots.mid_point_vector = np.add(left_fit, right_fit) / 2
    #     flaots.error = error
    #     flaots.left_trun = left_turn
    #     flaots.right_trun = right_turn
    #
    #     for i in range(23):
    #         point_pixter = viewsize[1] / 23
    #         flaots.testset.data[i] = point_pixter * PIXEL_CONVERSION_RATE * i
    #         m = flaots.mid_point_vector[0] * (viewsize[1] - point_pixter * i) ** 2 + flaots.mid_point_vector[1] * (viewsize[1] - point_pixter * i) + flaots.mid_point_vector[2] - viewsize[0] / 2
    #         flaots.testset.data[23 + i] = m * PIXEL_CONVERSION_RATE
    #     self.prediction_pub.publish(flaots)

    # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ2차 로스 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

    if error == 0:
        undistored_image = np.copy(video_copy2)
        color_stop_warp, newwarp, final_image, debug_images3 = plot_lanes_unwrap(viewsize, left_fit, right_fit, left_turn, right_turn, left_yellow_lane, right_yellow_lane, undistored_image, M_INV, stopx_base, roi_limit_M, roi_limit_L, roi_limit_R, originalsize, dst_pts, is_plot=False,
                                                                                 add_debug_image=add_debug_image3)
        # binary_detect_stop_morph_brg_3 = np.uint8(np.dstack((binary_detect_stop_morph_brg, binary_detect_stop_morph_brg, binary_detect_stop_morph_brg))*255)
        base_stop_image_show = cv2.bitwise_and(color_stop_warp, binary_detect_stop_morph_brg)

        base_stop_image_yellow = cv2.bitwise_and(color_stop_warp, yellow_mask_area_detect_morph)

        # base_stop_image = cv2.cvtColor(base_stop_image_show, cv2.COLOR_BGR2GRAY)
        length_stop_histogram, line_type, stopx_base, max_stop_histogram, bump_basex, bump_height = plot_stop_peak_hist(base_stop_image_show, base_stop_image_yellow, road_width, PIXEL_CONVERSION_RATE, is_plot=False)

        if stopx_base > 20: stopx_base = stopx_base - 20

        stop_deviation = (viewsize[1] - stopx_base) * PIXEL_CONVERSION_RATE
        base_stop_image_show = cv2.cvtColor(base_stop_image_show, cv2.COLOR_GRAY2BGR)
        # cv2.imshow('base_stop_image_show', base_stop_image_show * 255)

    if add_debug_image3:
        if error == 0:
            composed_image = np.copy(final_image)
        else:
            composed_image = np.copy(video_copy2)
        # debug_images_all2 = list(itertools.chain.from_iterable(debug_images_all))
        # composed_image = compose_debug_images(debug_images_all2)

        left_pix_per_text = "lef_pix_per : " + "{:0.4f}".format(bynary_left_pixel_percent) + " in per"
        right_pix_per_text = "Right_pix_per : " + "{:0.4f}".format(bynary_right_pixel_percent) + " in per"
        L_target_pixel_percent = "L_target_pix_per : " + "{:0.4f}".format(target_left_pixel_percent) + " in per"
        R_target_pixel_percent = "R_target_pix_per : " + "{:0.4f}".format(target_right_pixel_percent) + " in per"
        deviation_text = "Deviation : " + "{:0.2f}".format(deviation) + " in m"
        error_text = "Error_type : " + "{:}".format(error) + " in type"
        # print(type(deviation),type(white_tresh_left))
        white_tresh_lower_left_text = "tresh_left : " + "{:0.2f}".format(white_tresh_lower_left) + " "
        white_tresh_lower_right_text = "tresh_right : " + "{:0.2f}".format(white_tresh_lower_right) + " "
        following_lane_text = "fallowing : " + " fair"
        # flaots.fair_state = 1
        if following_left_lane:
            following_lane_text = "fallowing : " + " left"
            # flaots.fair_state = 0

        if following_right_lane:
            following_lane_text = "fallowing : " + " right"
            # flaots.fair_state = 2

        loop_text = "loop : " + "{:}".format(loop) + " "
        half_sum_pixel_per_text = "half_sum_pix_per : " + "{:0.4f}".format(bynary_half_sum_pixel_percent) + " in per"
        target_half_sum_pixel_percent_text = "half_sum_target : " + "{:0.4f}".format(target_half_sum_pixel_percent) + " in per"
        top_offset_text = "top_offset : " + "{:0.1f}".format(top_offset) + " in m"
        mid_offset_text = "mid_offset : " + "{:0.1f}".format(mid_offset) + " in m"
        bot_offset_text = "bot_offset : " + "{:0.1f}".format(bot_offset) + " in m"
        tresh_factor_right_text = "tresh_factor_right : " + "{:0.3f}".format(tresh_factor_right) + " "
        left_fit_deg_ave_text = "L_fit_deg_ave : " + "{:0.3f}".format(abs(left_fit_deg_ave)) + "in deg"
        center_fit_deg_ave_text = "M_fit_deg_ave : " + "{:0.3f}".format(abs(center_fit_deg_ave)) + "in deg"
        right_fit_deg_ave_text = "R_fit_deg_ave : " + "{:0.3f}".format(abs(right_fit_deg_ave)) + "in deg"
        deviation_stop_text = "Stop_Deviation : " + "{:0.2f}".format(stop_deviation) + " in m"
        length_stop_histogram_text = "length_stop_histogram : " + "{:0.2f}".format(length_stop_histogram)
        line_type_text = "line_type : " + "{:}".format(line_type)
        if line_type == 0: line_type_text = "line_type : " + "{:0.2f} / clean_line".format(line_type)
        if line_type == 1: line_type_text = "line_type : " + "{:0.2f} / not_stop_line".format(line_type)
        if line_type == 2: line_type_text = "line_type : " + "{:0.2f} / stop_line".format(line_type)
        if line_type == 3: line_type_text = "line_type : " + "{:0.2f} / yellow_line".format(line_type)
        max_stop_histogram_text = "max_s_hist : " + "{:0.2f}".format(max_stop_histogram)
        center_fit_mid_normal_grad_text = "m_normal_grad : " + "{:0.2f}".format(center_fit_mid_normal_grad)
        yellow_s_tresh_lower_text = "yellow_tresh : " + "{:}".format(yellow_s_tresh_lower)

        fontScale = 1
        thickness = 2
        fontFace = cv2.FONT_ITALIC

        cv2.putText(composed_image, left_pix_per_text, (10, 50), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, right_pix_per_text, (10, 90), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, L_target_pixel_percent, (10, 130), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, R_target_pixel_percent, (10, 170), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        if error == 0:
            cv2.putText(composed_image, error_text, (10, 210), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        else:
            cv2.putText(composed_image, error_text, (10, 210), fontFace, fontScale, (0, 0, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, deviation_text, (10, 250), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, loop_text, (10, 290), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, deviation_stop_text, (10, 330), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, length_stop_histogram_text, (10, 370), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, line_type_text, (10, 410), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, max_stop_histogram_text, (10, 450), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)

        cv2.putText(composed_image, half_sum_pixel_per_text, (580, 50), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, target_half_sum_pixel_percent_text, (580, 90), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, white_tresh_lower_left_text, (580, 130), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, white_tresh_lower_right_text, (580, 170), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, following_lane_text, (580, 210), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, top_offset_text, (580, 250), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, mid_offset_text, (580, 290), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, bot_offset_text, (580, 330), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, tresh_factor_right_text, (580, 370), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, left_fit_deg_ave_text, (580, 410), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, center_fit_deg_ave_text, (580, 450), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, right_fit_deg_ave_text, (580, 490), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)
        cv2.putText(composed_image, yellow_s_tresh_lower_text, (580, 530), fontFace, fontScale, (100, 200, 255), thickness, lineType=cv2.LINE_AA)

    # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ차선 좌표 검출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

    # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡcv2.imshow 이미지 쇼 컨트롤센터 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    imshow_scale_1 = 0.3
    imshow_scale_2 = 0.6
    print('p1_ingage_imshow_control_center')
    # cv2.imshow('video_copy1_PTtrans_blur_normalize4hsv', video_copy1_PTtrans_blur_normalize4hsv)  # 투상변환 영상
    # video_copy1_PTtrans_resized = cv2.resize(video_copy1_PTtrans, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('video_copy1_PTtrans_resized', video_copy1_PTtrans_resized)

    # gray_image = cv2.cvtColor(video_copy1_PTtrans_blur_normalize4hsv, cv2.COLOR_BGR2GRAY)
    # gray_image_resized = cv2.resize(gray_image, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('gray_image_resized',gray_image_resized)

    # cv2.imshow('left', binary_detect_morph_brg_left) #왼쪽 차선 기준 임계값 이진화
    # binary_detect_morph_brg_left_resized = cv2.resize(binary_detect_morph_brg_left, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('left', binary_detect_morph_brg_left_resized)

    # cv2.imshow('right', binary_detect_morph_brg_right) #오른쪽 차선 기준 임계값 이진화
    # binary_detect_morph_brg_right_resized = cv2.resize(binary_detect_morph_brg_right, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('right', binary_detect_morph_brg_right_resized)

    # cv2.imshow('binary_detect_morph_brg_half_sum', binary_detect_morph_brg_half_sum) # 각각 기준 임계값 이진화 영상을 반반 합친 영상

    binary_detect_morph_brg_half_sum_resized = cv2.resize(binary_detect_morph_brg_half_sum * 255, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    cv2.imshow('half_sum_resized', binary_detect_morph_brg_half_sum_resized)  # 필

    # cv2.imshow('final_line_color', final_line_color) #투상변환 + 블러, 정규화 + 최종 추출 영상 컬러 버전
    final_line_color_resized = cv2.resize(final_line_color, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    cv2.imshow('color_resized', final_line_color_resized)  # 필


    # cv2.imshow('final_line_bynary_full', final_line_bynary_full) #두 차선을 구분하지 않고 임계값을 계산한 영상 (사용하지않음)
    # cv2.imshow('final_line_bynary_half_sum', final_line_bynary_half_sum)  # 투상변환 + 블러, 정규화 + 최종 추출 영상 이진화 버전 # 최종적으로 추세선 감지 함수에 사용되는 영상 # binary_detect_morph_brg_half_sum 이것과 컬러 영역 이진화 영상을 합친 것
    # final_line_bynary_half_sum_resized = cv2.resize(final_line_bynary_half_sum, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1)))) # 최종적으로 추세선 감지 함수에 사용되는 영상
    # cv2.imshow('final_line_bynary_half_sum_resized', final_line_bynary_half_sum_resized)

    # video_copy1_PTtrans_resized = cv2.resize(video_copy1_PTtrans, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('video_copy1_PTtrans_resized', video_copy1_PTtrans_resized)

    video_copy1_PTtrans_copy1_to_draw_expected_lane = np.copy(video_copy1_PTtrans)
    cv2.line(video_copy1_PTtrans_copy1_to_draw_expected_lane, (int(straght_lane_left[0][0]), int(straght_lane_left[0][1])), (int(straght_lane_left[1][0]), int(straght_lane_left[1][1])), [0, 255, 0], 2)
    cv2.line(video_copy1_PTtrans_copy1_to_draw_expected_lane, (int(straght_lane_right[0][0]), int(straght_lane_right[0][1])), (int(straght_lane_right[1][0]), int(straght_lane_right[1][1])), [0, 255, 0], 2)
    cv2.line(video_copy1_PTtrans_copy1_to_draw_expected_lane, (int(vertical_lane[0][0]), int(vertical_lane[0][1])), (int(vertical_lane[1][0]), int(vertical_lane[1][1])), [0, 255, 0], 2)
    # cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane', video_copy1_PTtrans_copy1_to_draw_expected_lane)
    video_copy1_PTtrans_copy1_to_draw_expected_lane_resized = cv2.resize(video_copy1_PTtrans_copy1_to_draw_expected_lane, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    cv2.imshow('expected_lane', video_copy1_PTtrans_copy1_to_draw_expected_lane_resized)  # 필
    draw_expected_line_reverse, draw_expected_line_reverse_window_mix = Show_Original_Window_Mix(video_copy1_PTtrans_copy1_to_draw_expected_lane, video_copy1, src_pts_yfix, dst_pts, originalsize)
    draw_expected_line_reverse_window_mix_resized = cv2.resize(draw_expected_line_reverse_window_mix, dsize=(int(round(originalsize[0] * imshow_scale_2)), int(round(originalsize[1] * imshow_scale_2))))
    cv2.imshow('reverse_expected_lane', draw_expected_line_reverse_window_mix_resized)  # 필


    # video_copy1_PTtrans_copy1_to_draw_expted_lane_resized = cv2.resize(video_copy1_PTtrans_copy1_to_draw_expted_lane, dsize=(round(video_copy1_PTtrans_copy1_to_draw_expted_lane.shape[1] * imshow_scale_1 ), round(video_copy1_PTtrans_copy1_to_draw_expted_lane.shape[0] * imshow_scale_1 )))
    # cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane_resized', video_copy1_PTtrans_copy1_to_draw_expted_lane_resized)

    # if error != 0:
    if add_debug_image1_a == 1:
        sliding_window = np.copy(debug_images1[0])  # 슬라이딩 윈도우  # = debug_images[0]  # 여기서 debug_images 는 이진화영역과 좌표 추출 windows 영역과 그 안에서 검출된 픽셀들 그리고 추세선을 시각화 한 것이다.
        sliding_window_resized = cv2.resize(sliding_window, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
        cv2.imshow('sliding_window_resized', sliding_window_resized)  # 바이너리 및 차선 영역

    if add_debug_image2_a == 1:
        bynary_and_red_blue_and_poly_area = np.copy(debug_images2[0])  # = debug_images[1] 추세선 # 0718-[0](bynary_and_red_blue)를 없애서 [0]이 추세선이됨 # 여기서 debug_images[0]은 이진화 영역과 좌표 추출 영역 안에 포함된 픽셀들을 파랑,빨강,흰색으로 보여주는 이미지이구 [1] 추세선을 시각화 한 것이다
        # cv2.imshow('bynary_and_red_blue_and_poly_area', bynary_and_red_blue_and_poly_area) # 바이너리 영역에서 추세선과 그 영역 표시 이미지
        bynary_and_red_blue_and_poly_area_resized = cv2.resize(bynary_and_red_blue_and_poly_area, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
        cv2.imshow('poly_area', bynary_and_red_blue_and_poly_area_resized)  # 필
        # video_window_PTtrans_reverse, video_original_window_mix = Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize)

        red_blue = np.copy(debug_images2[5])
        red_blue_reized = cv2.resize(red_blue, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
        cv2.imshow('red_blue_reized', red_blue_reized)

        binary_all_left_gray_resized = cv2.resize(binary_all_left_gray, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
        binary_all_right_gray_resized = cv2.resize(binary_all_right_gray, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
        cv2.imshow('binary_all_left_gray_resized', binary_all_left_gray_resized * 255)
        cv2.imshow('binary_all_right_gray_resized', binary_all_right_gray_resized * 255)

    # video_window_PTtrans_reverse, video_original_window_mix = Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize)
    # cv2.imshow('video_window_PTtrans_reverse', video_window_PTtrans_reverse)  # 차선 좌표 추세선 함수와 검출창이 포함된 이미지를 역투상변환 한 이미지
    # cv2.imshow('video_original_window_mix', video_original_window_mix)  # 차선 좌표 추세선 함수와 검출창이 포함된 이미지를 역투상변환 한 이미지와 원본 영상 모두 보이는 이미지
    # if add_debug_image2_b == True:
    # cv2.imshow('bynary_window_image_area', bynary_window_image_area)
    # bynary_window_image_area_resized = cv2.resize(bynary_window_image_area, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('bynary_window_image_area_resized', bynary_window_image_area_resized)

    # bynary_window_image_left_area_resized = cv2.resize(bynary_window_image_left_area, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # bynary_window_image_right_area_resized = cv2.resize(bynary_window_image_right_area, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
    # cv2.imshow('bynary_window_image_left_area_resized', bynary_window_image_left_area_resized)  # 차선 검출 영역 안의 픽셀들만 보여주는 이미지
    # cv2.imshow('bynary_window_image_right_area_resized', bynary_window_image_right_area_resized)

    if add_debug_image3:
        # cv2.imshow('composed_image', composed_image)  # 필 # 차선을 초록색으로 덮은 이미지에 곡률과 편차를 화면에 택스트를 표시한 이미지
        composed_image_resized = cv2.resize(composed_image, dsize=(int(round(originalsize[0] * imshow_scale_2)), int(round(originalsize[1] * imshow_scale_2))))
        cv2.imshow('composed_image_resized', composed_image_resized)  # 필

        # out.write(video_copy1_PTtrans)

        # if error == 0:
        # cv2.imshow('final_image',final_image) # 차선 영역을 초록색으로 색칠한 이미지
        # warped_new = np.copy(debug_images3[0])  # 투상변환 화면에서의 초록색으로 덮은 이미지와 점선 영역
        # warped_new_resized = cv2.resize(warped_new, dsize=(int(round(PT_view_width_2 * imshow_scale_1)), int(round(PT_view_height_2 * imshow_scale_1))))
        # cv2.imshow('warped_new', warped_new)
        # cv2.imshow('warped_new_resized', warped_new_resized)

    # stop = timeit.default_timer()
    # print('time = ', stop - start)  # 걸림

    # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡcv2.imshow 이미지 쇼 컨트롤센터 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

    # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ2차 로스 추가용 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    # out.write(composed_image)
    print('sibal~~1')
    # for i in range(10000):
    #     print('p1_done_def_aready :: ',i)
    print('sibal~~2')
    if error > 0: return_list = [error, [], [], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, total_error_left, total_error_right, total_error_half_sum, white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, yellow_s_tresh_lower, stopx_base, composed_image, 0, 0, deviation]
    else: return_list = [error, left_fit, right_fit, following_left_lane, following_right_lane, roi_limit_L, roi_limit_M, roi_limit_R, line_type, bump_basex, bump_height, left_turn, right_turn, total_error_left, total_error_right, total_error_half_sum, white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, yellow_s_tresh_lower, stopx_base, composed_image, lc, rc, deviation]
    print('sibal~~3')

    convert_list = [white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, yellow_s_tresh_lower]
    return return_list, convert_list


def get_image_convert(n, m, src_pts_yfix, dst_pts, viewsize, white_tresh_lower_stop, zero_1, zero_3, img_queue, video_queue, list_queue):
    print('p2_pid = ', os.getpid(), ' id(m) = ', id(m))
    # cam = cv2.VideoCapture(0)
    # switch_1, switch_1 = Pipe()
    # switch_2, switch_2 = Pipe()
    # img_queue, img_queue = Pipe()
    # switch_msg = 0
    while True:
        print('p2_try_video_queue.get()')
        video = video_queue.get()
        print('p2_done_video_queue.get()')
        convert_list = list_queue.get()
        white_tresh_lower_left = convert_list[0]
        white_tresh_lower_right = convert_list[1]
        white_tresh_lower_half_sum = convert_list[2]
        yellow_s_tresh_lower = convert_list[3]


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

        # video = cv2.resize(video, dsize=(1280, 720))
        print('p2_what_happen1`')
        video_copy1 = np.copy(video)  # 편집용 video copy1
        video_copy2 = np.copy(video)  # 편집용 video copy2 circle 넣을 거
        print('p2_what_happen2')
        video_copy1_PTtrans, video_copy2 = PTtrans_and_Circle(video_copy1, video_copy2, src_pts_yfix, dst_pts, viewsize)
        print('p2_what_happen3')
        video_copy1_PTtrans_blur_normalize4hsv = Blur_and_Normalize(video_copy1_PTtrans)  # 영상 블러처리 및 정규화
        print('p2_what_happen4')
        video_copy1_Gray = cv2.cvtColor(video_copy1_PTtrans_blur_normalize4hsv, cv2.COLOR_BGR2GRAY)
        print('p2_what_happen5')
        lower_yellow = (6, yellow_s_tresh_lower, 90)  # HSV 컬러 영억 검출 임계값 영역 (색상, 채도, 조도)
        upper_yellow = (21, 255, 255)
        # lower_yellow = (6, 50, 90)  # HSV 컬러 영억 검출 임계값 영역 (색상, 채도, 조도)
        # upper_yellow = (21, 255, 255)
        lower_blue = (80, 50, 180)
        upper_blue = (115, 255, 255)
        yellow_detect_morph_brg, yellow_mask_area_detect_morph = HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_yellow, upper_yellow)  # HSV 컬러 영역 이진화 + 모폴로지 연산
        blue_detect_morph_brg, blue_mask_area_detect_morph = HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_blue, upper_blue)
        # color_brg = cv2.bitwise_or(yellow_detect_morph_brg, blue_detect_morph_brg)  # 노랑 파랑 검출된 영역 합치기
        color_brg = np.copy(yellow_detect_morph_brg)  # cv2.bitwise_or(yellow_detect_morph_brg, yellow_detect_morph_brg)  # 노랑 파랑 검출된 영역 합치기
        # color_bynary = cv2.bitwise_or(yellow_mask_area_detect_morph, blue_mask_area_detect_morph)
        color_bynary = np.copy(yellow_mask_area_detect_morph)  # cv2.bitwise_or(yellow_mask_area_detect_morph, yellow_mask_area_detect_morph)
        white_tresh_left = [white_tresh_lower_left, 255]  # 흰색 차선 검출 임계값 영역(검출되는 영역이 너무 적으면 최솟값을 더 낮추면 됨)
        # white_tresh_left = [135, 255]  # 흰색 차선 검출 임계값 영역(검출되는 영역이 너무 적으면 최솟값을 더 낮추면 됨)
        white_tresh_right = [white_tresh_lower_right, 255]
        # white_tresh_right = [135, 255]
        # white_tresh_full = [white_tresh_lower_full, 255]
        white_tresh_stop = [white_tresh_lower_stop, 255]
        binary_detect_morph_brg_left = White_binary(video_copy1_Gray, white_tresh_left)
        binary_detect_morph_brg_right = White_binary(video_copy1_Gray, white_tresh_right)
        binary_detect_stop_morph_brg = White_binary(video_copy1_Gray, white_tresh_stop)
        # binary_detect_morph_brg_full = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_full)
        
        binary_detect_morph_brg_half_sum = np.copy(zero_1)

        binary_detect_morph_brg_half_sum[:, : binary_detect_morph_brg_half_sum.shape[1] // 2] = binary_detect_morph_brg_left[:, : binary_detect_morph_brg_left.shape[1] // 2]
        binary_detect_morph_brg_half_sum[:, binary_detect_morph_brg_half_sum.shape[1] // 2:] = binary_detect_morph_brg_right[:, binary_detect_morph_brg_right.shape[1] // 2:]

        # print('binary_detect_morph_brg_half_sum.shape = ',binary_detect_morph_brg_half_sum.shape)
        final_line_color = Final_Bynary_Mix(color_brg, binary_detect_morph_brg_half_sum, blue_mask_area_detect_morph, yellow_mask_area_detect_morph)  # 컬러랑 흰색 검출 영역 합치기 #추세선 탐지에 사용한는 영상의 이진화를 하기 전단계
        # final_line_bynary_left = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_left)
        # final_line_bynary_right = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_right)

        final_line_bynary_half_sum = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_half_sum)  # 추세선 탐지에 사용되는 이진화 영상
        # final_line_bynary_full = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_full)
        # print('sum bynary pixel = ', np.sum(final_line_bynary/255)/3/(final_line_bynary.shape[0]*final_line_bynary.shape[0]),'--',np.sum(final_line_bynary/255)/3)

        # cv2.imshow('final_line_bynary_half_sum', final_line_bynary_half_sum* 255)

        binary_all_left_gray = np.uint8(cv2.bitwise_or(color_bynary, binary_detect_morph_brg_left))
        binary_all_right_gray = np.uint8(cv2.bitwise_or(color_bynary, binary_detect_morph_brg_right))
        binary_all_half_sum_gray = np.uint8(cv2.bitwise_or(color_bynary, binary_detect_morph_brg_half_sum))
        # cv2.imshow('11',binary_all_left_gray* 255)
        print('p2_try_make_send_image')
        send_image = []
        send_image.append(binary_all_left_gray)  # 0
        send_image.append(binary_all_right_gray)  # 1
        send_image.append(binary_all_half_sum_gray)  # 2
        send_image.append(binary_detect_morph_brg_left)  # 3
        send_image.append(binary_detect_morph_brg_right)  # 4
        send_image.append(binary_detect_morph_brg_half_sum)  # 5
        send_image.append(binary_detect_stop_morph_brg)  # 6
        send_image.append(yellow_mask_area_detect_morph)  # 7
        send_image.append(video_copy1)  # 8
        send_image.append(video_copy2)  # 9
        send_image.append(final_line_color)  # 10 보기용
        send_image.append(video_copy1_PTtrans)  # 11
        print('p2_done_make_send_image')

        # send_image.append(video_copy2)  # 8
        # try:
        #     print('p2_try_switch_2.put(1)')
        #     switch_2.put(1)
        #     print('p2_done_switch_2.put(1)')
        # except EOFError:
        #     break

        print('p2_try_img_queue.put(send_image)')
        img_queue.put(send_image)
        print('p2_done_img_queue.put(send_image)')

        print('process_done')

# cv brigde sub,pub
class image_converter:

    def __init__(self):

        self.image_pub = rospy.Publisher("lanetraker", Image, queue_size=1000)
        self.prediction_pub = rospy.Publisher("lane", Floats, queue_size=100)
        self.lane_identify_pub = rospy.Publisher("lane_identify", Floats_for_mission, queue_size=1)

        self.bridge = CvBridge()
        self.lanetracker()

        self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def get_image(self):
        try:
            data = rospy.wait_for_message("usb_cam/image_raw", Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        return cv_image



    def lanetracker(self):

        view_scale = 0.4
        sight_height = 1.61  # 카메라 높이
        target_distance = 12  # 11.25  # 목표 거리 (현재 목표 10m전방)
        road_width = 4  # 차선 폭 (현재 3.5미터 실측 후 조정)
        margin = 12  # 추가로 투상변환할 폭

        # view_scale = 1
        # sight_height = 1.06  # 카메라 높이
        # target_distance = 19.35  # 11.25  # 목표 거리 (현재 목표 10m전방)
        # road_width = 3.5  # 차선 폭 (현재 3.5미터 실측 후 조정)
        # margin = 12  # 추가로 투상변환할 폭

        focal_length_horizontal = 1.635  # sample_video(f_l_h = 0.4, f_l_v = 0.35)  C525(f_l_h = 0.515, f_l_v = 0.31(by0.1)) C930e(f_l_h = 0.35, f_l_v = 0.648)
        focal_length_vertical = 1.635
        view_length1_h = 1.42
        view_length1_h_bot = 1.485
        view_length2_v = 0.78

        PIXEL_CONVERSION_RATE = 0.01 / view_scale  # 100픽셀당 1미터
        ym_per_pix = 0.01 / view_scale  # meters per pixel in y dimension
        xm_per_pix = 0.01 / view_scale  # meters per pixel in x dimension
        lower_offset = road_width - 1.0  # 최소 차선 감지 폭
        upper_offset = road_width + 1.5  # 최대 차선 감지 폭
        kp = 2
        ki = 0.0
        kd = 0.7

        proper_size = np.zeros([720, 1280, 3], dtype=np.uint8)
        # proper_size = np.zeros([480, 640, 3], dtype=np.uint8)
        # proper_size = np.copy(video)
        # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ좌표 자동 지정ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
        src_pts_yfix, dst_pts, PT_view_width, PT_view_height, PT_view_width_2, PT_view_height_2, vertical_lane, straght_lane_left, straght_lane_right = find_proper_transper_point(proper_size, sight_height, target_distance, road_width, margin, view_scale, focal_length_horizontal,
                                                                                                                                                                                   focal_length_vertical, view_length1_h, view_length1_h_bot, view_length2_v,
                                                                                                                                                                                   PIXEL_CONVERSION_RATE)
        M_INV = cv2.getPerspectiveTransform(dst_pts, src_pts_yfix)
        viewsize = [PT_view_width, PT_view_height]
        # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ좌표 자동 지정ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        lc = 0.0
        rc = 0.0
        left_fit = []
        right_fit = []
        error = 0
        bynary_pixel_percent = 0.02
        error_pixel_percent_left = 0
        total_error_left = 0
        error_pixel_percent_right = 0
        total_error_right = 0
        error_pixel_percent_half_sum = 0
        total_error_half_sum = 0
        left_turn = 0
        right_turn = 0
        left_yellow_lane = 0
        right_yellow_lane = 0
        following_right_lane = False
        following_left_lane = False
        error_loop = 0

        bynary_left_pixel_percent = 0
        bynary_right_pixel_percent = 0
        target_left_pixel_percent = 0.01
        target_right_pixel_percent = 0.01
        bynary_half_sum_pixel_percent = 0
        target_half_sum_pixel_percent = 0
        full_line_target_percent = 0.01
        error_pixel_percent_yellow = 0
        stopx_base = 0
        line_type = 0
        bump_basex = 0
        bump_height = 0
        view_area = PT_view_width * PT_view_height
        roi_limit_M = 0
        roi_limit_L = 0
        roi_limit_R = 0

        deviation = 0
        top_offset = 0
        mid_offset = 0
        bot_offset = 0
        tresh_factor_right = 0
        left_fit_deg_ave = 0
        center_fit_deg_ave = 0
        right_fit_deg_ave = 0
        stop_deviation = 0
        length_stop_histogram = 0
        max_stop_histogram = 0
        center_fit_mid_normal_grad = 0

        white_tresh_lower_left = 160
        white_tresh_lower_right = 160
        white_tresh_lower_half_sum = 160
        white_tresh_lower_stop = 170
        yellow_s_tresh_lower = 50
        zero_1 = np.zeros((viewsize[1], viewsize[0])).astype(np.uint8)
        zero_3 = np.zeros((viewsize[1], viewsize[0], 3)).astype(np.uint8)
        convert_list = [white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, yellow_s_tresh_lower]

        # # size = (viewsize[0],viewsize[1]) #투상변환 한 영상 저장할 때
        # # fps = cam.get(cv2.CAP_PROP_FPS)
        # fps = 30
        # fourcc = cv2.VideoWriter_fourcc(*'DIVX')  # 영상 저장 코덱 지정
        # filename = 'line_detect.avi'  # 저장될 영상 이름 지정
        # # out = cv2.VideoWriter(filename, fourcc, fps, (viewsize[0], viewsize[1])) #sizn(가로,세로) #영상 저장 설정 세팅
        # out = cv2.VideoWriter(filename, fourcc, fps, (proper_size.shape[1], proper_size.shape[0]))  # sizn(가로,세로) #영상 저장 설정 세팅
        # # plt.ion() #plot을 실시간으로 보여주게 해주는 함수

        #cam = cv2.VideoCapture(0)
        # cam = cv2.VideoCapture('C:/Users/USER/PycharmProjects/line_detect/k_city_out_2.mp4')
        # ret, video = cam.read()
        # video = cv2.resize(video, dsize=(1280, 720))


        flaots.testset.layout.dim.append(MultiArrayDimension())
        flaots.testset.layout.dim.append(MultiArrayDimension())
        flaots.testset.layout.dim[0].label = "height"
        flaots.testset.layout.dim[1].label = "width"
        flaots.testset.layout.dim[0].size = 2
        flaots.testset.layout.dim[1].size = 23
        flaots.testset.layout.dim[0].stride = 2 * 23
        flaots.testset.layout.dim[1].stride = 23
        flaots.testset.layout.data_offset = 0
        flaots.testset.data = np.zeros((46))
        

        dstride0 = flaots.testset.layout.dim[0].stride
        dstride1 = flaots.testset.layout.dim[1].stride
        offset = flaots.testset.layout.data_offset



        rospy.init_node('image_converter', anonymous=True)
        print('p1_try_video_get')
        video = np.copy(self.get_image())
        print('p1_done_video_get')

        img_queue = Queue()
        video_queue = Queue()
        list_queue = Queue()

        print('p1_done_var_declare')

        loop = 0
        p_list = []
        m = 10000


        print('p1_start_first_process')
        process1 = Process(target=get_image_convert, args=(3, m, src_pts_yfix, dst_pts, viewsize, white_tresh_lower_stop, zero_1, zero_3, img_queue, video_queue, list_queue))
        process1.start()
        print('p1_during_first_process')

        print('p1_try_filst_video_queue.put(video) ::')
        video_queue.put(video)
        print('p1_done_filst_video_queue.put(video) ::')

        print('p1_try_filst_list_queue.put(video) ::')
        list_queue.put(convert_list)
        print('p1_done_filst_list_queue.put(video) ::')

        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            loop += 1
            print('p1_loop_count : ',loop)
            # ret, video = cam.read()
            # video = cv2.resize(video, dsize=(1280, 720))
            
            print('p1_try_video_get')
            video = np.copy(self.get_image())
            print('p1_done_video_get')

            print('p1_try_new_image_set = np.copy(img_queue.get()) ::')
            new_image_set = np.copy(img_queue.get())
            print('p1_done_new_image_set = np.copy(img_queue.get()) ::')

            print('p1_try_video_queue.put(video) ::')
            video_queue.put(video)
            print('p1_done_video_queue.put(video) ::')

            print('p1_try_list_queue.put(video) ::')
            list_queue.put(convert_list)
            print('p1_done_list_queue.put(video) ::')

            print('p1_start_get_lane')
            out = 0
            return_list, convert_list = get_lane(new_image_set, error, left_fit, right_fit, following_left_lane, following_right_lane, target_left_pixel_percent, target_right_pixel_percent, full_line_target_percent, roi_limit_L, roi_limit_M, roi_limit_R, line_type, bump_basex, bump_height, left_turn, right_turn, error_pixel_percent_left, error_pixel_percent_right, total_error_left, total_error_right, total_error_half_sum, error_pixel_percent_yellow, error_pixel_percent_half_sum, white_tresh_lower_left, white_tresh_lower_right, white_tresh_lower_half_sum, yellow_s_tresh_lower, PT_view_width, PT_view_height, loop, bynary_half_sum_pixel_percent, target_half_sum_pixel_percent, out, bynary_left_pixel_percent, bynary_right_pixel_percent, deviation, top_offset, mid_offset, bot_offset, tresh_factor_right, left_fit_deg_ave, center_fit_deg_ave, right_fit_deg_ave, stop_deviation, length_stop_histogram, max_stop_histogram, center_fit_mid_normal_grad, stopx_base, road_width, PIXEL_CONVERSION_RATE, lower_offset, upper_offset, zero_1, zero_3, viewsize, ym_per_pix, kp, ki, kd, M_INV, dst_pts, PT_view_width_2, PT_view_height_2, vertical_lane, straght_lane_left, straght_lane_right, src_pts_yfix)
            print('p1_done_get_lane')
            print(process1)
            error = return_list[0]
            left_fit = return_list[1]
            right_fit = return_list[2]
            following_left_lane = return_list[3]
            following_right_lane = return_list[4]
            roi_limit_L = return_list[5]
            roi_limit_M = return_list[6]
            roi_limit_R = return_list[7]
            line_type = return_list[8]
            bump_basex = return_list[9]
            bump_height = return_list[10]
            left_turn = return_list[11]
            right_turn = return_list[12]
            total_error_left = return_list[13]
            total_error_right = return_list[14]
            total_error_half_sum = return_list[15]
            white_tresh_lower_left = return_list[16]
            white_tresh_lower_right = return_list[17]
            white_tresh_lower_half_sum = return_list[18]
            yellow_s_tresh_lower = return_list[19]
            stopx_base = return_list[20]
            composed_image = return_list[21]
            lc = return_list[22]
            rc = return_list[23]
            deviation = return_list[24]
            print('p1_done_loop')

            if error == 0:
                flaots_for_mission.left_curv = lc
                flaots_for_mission.right_curv = rc
                flaots_for_mission.deviation = deviation
                flaots_for_mission.stop_deviation = stop_deviation
                flaots_for_mission.mid_point_vector = np.add(left_fit, right_fit) / 2
                flaots_for_mission.error = error
                flaots_for_mission.left_trun = left_turn
                flaots_for_mission.right_trun = right_turn

                for i in range(23):
                    point_pixter = viewsize[1] / 23
                    flaots.testset.data[i] = point_pixter * PIXEL_CONVERSION_RATE * i
                    m = flaots_for_mission.mid_point_vector[0] * (viewsize[1] - point_pixter * i) ** 2 + flaots_for_mission.mid_point_vector[1] * (viewsize[1] - point_pixter * i) + flaots_for_mission.mid_point_vector[2] - viewsize[0] / 2
                    flaots.testset.data[23 + i] = m * PIXEL_CONVERSION_RATE
                self.prediction_pub.publish(flaots)
                self.lane_identify_pub.publish(flaots_for_mission)

                if following_left_lane:
                    flaots_for_mission.fair_state = 0

                elif following_right_lane:
                    flaots_for_mission.fair_state = 2
                else:
                    flaots_for_mission.fair_state = 1

        out.release()
        cam.release()
        cv2.destroyAllWindows()


        print('Exit')



def main():  # 메인문

    try:
        image_converter()
    except rospy.ROSInterruptException:
        # cam.release()
        # out.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
