#!/usr/bin/env python
#-*-coding:utf-8-*_

"""
패치버전 19.07.24.1
- 변수 지정을 전부 외부영역? 에서 실행
- error 변수를 global로 사용하지 않고 리턴 값으로 사용
- ㅠㅠ

패치버전 19.07.22.1
- 전체 차선으로 계산하엿던 임계값 적용을 왼쪽 오른쪽 차선 따로 각각 계산하도록 수정하였음(각각의 차선 전용으로 연산하는 이미지가 2개 늘어남)
- 단순 비례 제어였던 임계값 적용 계산 식에 PID제어를 적용하였음
- 추세선 검출 영역 margin 수정
"""




## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
##   rostopic echo color
## this demo shows some of the more advanced APIs in rospy.
import sys,time
import operator
from cv_bridge import CvBridge, CvBridgeError
from scipy.ndimage import filters

import cv2
import numpy as np
import roslib
roslib.load_manifest('macaron')

import rospy

from matplotlib import pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension

from macaron.msg import Floats
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
flaots=Floats()

#from std_msgs.msg import ColorRGBA


#sungjun's functions
def plot_peak_hist(warped_gray, is_plot=True):
    global error
    histogram = np.sum(warped_gray[warped_gray.shape[0]//2:,:], axis=0)
    if is_plot: plt.plot(histogram)

    midpoint = np.int(histogram.shape[0] / 2)
    if max(histogram[:midpoint]) <= 10 or max(histogram[midpoint:]) <= 10:
        error = 1
        print('error = 1, plot_peak_hist')
        return []
    error = 0
    return histogram
def find_left_right_via_histogram(binary_warped, is_debug=False, is_plot=False, add_debug_image=False):
    global error
    debug_images = []

    histogram = plot_peak_hist(binary_warped, is_plot=False)
    if error == 1: return [], [], []
    error = 0

    if add_debug_image: debug_images.append(histogram)
    #if add_debug_image: debug_images.append(combined_binary)

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    #if np.max(histogram[:midpoint]) <
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    if is_debug:
        print("leftx_base:" + str(leftx_base))
        print("rightx_base:" + str(rightx_base))
        print("midpoint:" + str(midpoint))

    return leftx_base, rightx_base, debug_images
def find_ploy_fit_window_search(binary_warped, leftx_base, rightx_base, nwindows=9, is_debug=False, margin=100, minpix=50, is_plot=True, add_debug_image=True):
    debug_images = []

    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    if is_debug:
        print("window_height:" + str(window_height))

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

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        #cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 3)
        #cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 3)



        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]

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


    #print(out_img.shape , '-', out_img.dtype , '-', np.max(out_img), '-', np.min(out_img))
    out_img[lefty, leftx] = [255, 0, 0]
    out_img[righty, rightx] = [0, 0, 255]

    left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, rightx, righty)
    print('left_polyfit line = ',left_fit)
    print('right_polyfit line = ',right_fit)
    try:
        #flaots.header.stamp=rospy.Time.now()

        flaots.left_curv=lc
        flaots.right_curv=rc
        flaots.deviation=deviation
        flaots.mid_point_vector=map(operator.add, left_fit/2, right_fit/2)

        for i in range(2):
            for j in range(23):
                flaots.testset.data[j] =25*j
                m=2*flaots.mid_point_vector[0]*(25*j)+flaots.mid_point_vector[1]
                flaots.testset.data[23*i+j] =25*m

 
    except CvBridgeError as e:
        print(e)


    if add_debug_image:
        out_img = plot_detected_poly(out_img, left_fit, right_fit, is_plot=is_plot)
        debug_images.append(out_img)

    return left_fit, right_fit, debug_images
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
        #print(out_img.shape, '-', out_img.dtype, '-', np.min(out_img), '-', np.max(out_img))
        #out_img_uint32 = out_img.astype(np.uint8)
        #cv2.imshow('out_img_uint32',out_img_uint32)
        #plt.xlim(0, out_img.shape[1])
        #plt.ylim(out_img.shape[0], 0)
    return out_img

def next_frame_find_poly_already_fitted(binary_warped_left, binary_warped_right, left_fit, right_fit,lower_offset, upper_offset, margin=50, is_plot=True, add_debug_image=True):
    global error
    # Assume you now have a new warped binary image
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!

    debug_images = []

    nonzero_left = binary_warped_left.nonzero()
    nonzero_right = binary_warped_right.nonzero()
    nonzeroy_left = np.array(nonzero_left[0])
    nonzerox_left = np.array(nonzero_left[1])
    nonzeroy_right = np.array(nonzero_right[0])
    nonzerox_right =  np.array(nonzero_right[1])

    left_lane_inds = ((nonzerox_left > (left_fit[0] * (nonzeroy_left ** 2) + left_fit[1] * nonzeroy_left + left_fit[2] - margin))
                      & (nonzerox_left < (left_fit[0] * (nonzeroy_left ** 2) + left_fit[1] * nonzeroy_left + left_fit[2] + margin)))
    right_lane_inds = ((nonzerox_right > (right_fit[0] * (nonzeroy_right ** 2) + right_fit[1] * nonzeroy_right + right_fit[2] - margin))
                       & (nonzerox_right < (right_fit[0] * (nonzeroy_right ** 2) + right_fit[1] * nonzeroy_right + right_fit[2] + margin)))

    # Again, extract left and right line pixel positions
    leftx = nonzerox_left[left_lane_inds]
    lefty = nonzeroy_left[left_lane_inds]
    rightx = nonzerox_right[right_lane_inds]
    righty = nonzeroy_right[right_lane_inds]

    # Fit a second order polynomial to each
    if len(leftx) == 0 or len(rightx) == 0:
        print('error = 2, next_frame_find_poly_already_fitted, cant_find_line - leftx : ', len(leftx), ', rightx : ', len(rightx))
        error = 2
        return [], [], 0, 0, 0, []
    left_fit, right_fit = fit_ploy_2_degree(leftx, lefty, rightx, righty)

    left_fitx = left_fit[0] * (binary_warped_left.shape[0] - 1) ** 2 + left_fit[1] * (binary_warped_left.shape[0] - 1) + left_fit[2]
    right_fitx = right_fit[0] * (binary_warped_right.shape[0] - 1) ** 2 + right_fit[1] * (binary_warped_right.shape[0] - 1) + right_fit[2]


    if right_fitx - left_fitx <=lower_offset or right_fitx - left_fitx >= upper_offset: #추세선이 차선범위를 이탈할 시 좌표검출 재가동
        print('error = 3, next_frame_find_poly_already_fitted, line_overlaped or line_breakaway - right_fitx - left_fitx = ', right_fitx - left_fitx, 'left_fitx = ',left_fitx, 'right_fitx = ',right_fitx)
        error = 3
        return [], [], 0, 0, 0, []
    error = 0

    # Create an image to draw on and an image to show the selection window
    out_img_left = np.dstack((binary_warped_left, binary_warped_left, binary_warped_left)) * 255
    out_img_right = np.dstack((binary_warped_right, binary_warped_right, binary_warped_right)) * 255
    # Color in left and right line pixels
    out_img_left[lefty, leftx] = [255, 0, 0]
    out_img_right[righty, rightx] = [0, 0, 255]

    out_img_half_sum = np.zeros_like(out_img_left)
    out_img_half_sum[:, : out_img_half_sum.shape[1] // 2, :] = out_img_left[:, : out_img_left.shape[1] // 2, :]
    out_img_half_sum[:, out_img_half_sum.shape[1] // 2:, :] = out_img_right[:, out_img_right.shape[1] // 2:, :]


    result = None
    if add_debug_image:
        #debug_images.append(out_img_left)
        result, window_img_left, window_img_right = get_already_fit_image_plot(out_img_half_sum, left_fit, right_fit, is_plot=is_plot, margin=margin)
        debug_images.append(result)
        debug_images.append(window_img_left)
        debug_images.append(window_img_right)

    left_curverad, right_curverad = get_radius_of_curvature(left_fit, right_fit, out_img_left)
    deviation = get_vehicle_deviation(out_img_left, left_fit, right_fit)
    return left_fit, right_fit, left_curverad, right_curverad, deviation, debug_images
def get_radius_of_curvature(left_fit_cr, right_fit_cr, image_for_size):
    plot_yyy = np.linspace(0, image_for_size.shape[0] - 1, image_for_size.shape[0])

    # Define conversions in x and y from pixels space to meters
    #ym_per_pix = 30 / 720  # meters per pixel in y dimension
    #xm_per_pix = 3.7 / 700  # meters per pixel in x dimension
    y_eval = np.max(plot_yyy)

    # Fit new polynomials to x,y in world space
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    # Now our radius of curvature is in meters
    return left_curverad, right_curverad
def get_vehicle_deviation(img, left_fit, right_fit):
    image_bottom_pixel = img.shape[0] - 1

    bottom_x_position_left_lane = left_fit[0] * (image_bottom_pixel ** 2) + left_fit[1] * (image_bottom_pixel) \
                                  + left_fit[2]
    bottom_x_position_right_lane = right_fit[0] * (image_bottom_pixel ** 2) + right_fit[1] * (image_bottom_pixel) \
                                   + right_fit[2]
    vehicle_offset = (bottom_x_position_left_lane + bottom_x_position_right_lane) / 2.0 - img.shape[1] / 2

    # convert pixels to real space
    return vehicle_offset * PIXEL_CONVERSION_RATE
def get_already_fit_image_plot(out_img_half_sum, left_fit, right_fit, margin=50, is_plot=True):
    # Generate x and y values for plotting

    ploty = np.linspace(0, out_img_half_sum.shape[0] - 1, out_img_half_sum.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    # # Generate a polygon to illustrate the search window area
    # # And recast the x and y points into usable format for cv2.fillPoly()
    # stack -> v(row), h(column), d(depth)
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    # # Draw the lane onto the warped blank image
    window_img_full = np.zeros_like(out_img_half_sum)
    window_img_left = np.zeros_like(out_img_half_sum)
    window_img_right = np.zeros_like(out_img_half_sum)
    cv2.fillPoly(window_img_full, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img_full, np.int_([right_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img_left, np.int_([left_line_pts]), (255, 255, 255))
    cv2.fillPoly(window_img_right, np.int_([right_line_pts]), (255, 255, 255))

    result = cv2.addWeighted(out_img_half_sum, 1, window_img_full, 0.3, 0)

    for pts, index in enumerate(left_fitx):
        cv2.circle(result, ((int)(left_fitx[pts]), (int)(ploty[pts])), 3, (255, 0, 255), -1)

    for pts, index in enumerate(right_fitx):
        cv2.circle(result, ((int)(right_fitx[pts]), (int)(ploty[pts])), 3, (255, 0, 255), -1)

    if is_plot:
        plt.xlim(0, 640)
        plt.ylim(480, 0)
        plt.imshow(result)

    return result, window_img_left, window_img_right
def plot_lanes_unwrap(image_for_size, left_poly, right_poly, undistorted_in_rgb, M_INV, is_plot=False, add_debug_image=True):
    debug_images = []

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(image_for_size).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    plot_yyy = np.linspace(0, image_for_size.shape[0] - 1, image_for_size.shape[0])

    #     pdb.set_trace()
    left_fitxxx = left_poly[0] * plot_yyy ** 2 + left_poly[1] * plot_yyy + left_poly[2]
    right_fitxxx = right_poly[0] * plot_yyy ** 2 + right_poly[1] * plot_yyy + right_poly[2]

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitxxx, plot_yyy]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitxxx, plot_yyy])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

    if add_debug_image: debug_images.append(color_warp)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, M_INV, (undistorted_in_rgb.shape[1], undistorted_in_rgb.shape[0]))
    #cv2.imshow('newwarp',newwarp)
    #cv2.imshow('color_warp', color_warp)
    if add_debug_image: debug_images.append(newwarp)

    # Combine the result with the original image
    result = cv2.addWeighted(undistorted_in_rgb, 1, newwarp, 0.3, 0)
    #if is_plot:
        #plot_two_images(color_warp, result, plot_diff=False)
        #plot_two_images(newwarp, result, plot_diff=False)

    if add_debug_image: debug_images.append(result)

    return color_warp, result, debug_images

#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 좌표추출 함수 선언 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def mouse_callback(event, x, y, flags, param): # 화면에서 마우스 클릭시 호출되는 함수
    global selected_pts, video_copy1
    if event == cv2.EVENT_LBUTTONUP:
        selected_pts.append([x, y])
        #cv2.circle(video_copy1, (x,y),10,(0,255,0),3)
def select_points(image, points_num):  # 로드 되는 창에서 변환할 좌표 4 곳을 클릭
    global selected_pts
    selected_pts = []
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)
    while True:
        if len(selected_pts) >= 1:        #점에 원 표시
            cv2.circle(image, tuple(selected_pts[0]), 10, (0, 0, 255), 3)
            if len(selected_pts) >= 2:
                cv2.circle(image, tuple(selected_pts[1]), 10, (0, 127, 255), 3)
                if len(selected_pts) >= 3:
                    cv2.circle(image, tuple(selected_pts[2]), 10, (0, 255, 255), 3)
        cv2.imshow('image', image)
        k = cv2.waitKey(1)
        if k == 27 or len(selected_pts) == points_num:
            break
    cv2.destroyAllWindows()
    return np.array(selected_pts, dtype=np.float32)
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 좌표추출 함수 선언 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def PTtrans_and_Circle(video_copy1, video_copy2, src_pts_yfix, dst_pts, viewsize):
    cv2.circle(video_copy2, tuple(src_pts_yfix[0]), 10, (0, 0, 255), 3)  # 변환하는 위치를 원본 이미지에 표시
    cv2.circle(video_copy2, tuple(src_pts_yfix[1]), 10, (0, 127, 255), 3)  # 빨주노초, 왼쪽 아래부터 시계방향
    cv2.circle(video_copy2, tuple(src_pts_yfix[2]), 10, (0, 255, 255), 3)
    cv2.circle(video_copy2, tuple(src_pts_yfix[3]), 10, (0, 255, 0), 3)

    perspective_m = cv2.getPerspectiveTransform(src_pts_yfix, dst_pts)
    video_copy1_PTtrans = cv2.warpPerspective(video_copy1, perspective_m, (viewsize[0], viewsize[1]), flags=cv2.INTER_LINEAR)  # 변환된 hsv 영상 출력 영역 (dst_pts 랑 똑같이 맞춰주면 됨)
    # video_copy1_PTtrans = cv2.warpPerspective(video_copy2, perspective_m, (viewsize[0],viewsize[1]))  # circle 포함
    # video_copy1_PTtrans = np.copy(video_copy1)                                        # 투상변환 적용 안하기

    # cv2.imshow('video_copy1_PTtrans', video_copy1_PTtrans)  #변환 이미지 출력
    return video_copy1_PTtrans
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ역투상변환 코드ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def PTtrans_Reverse(video_window, src_pts_yfix, dst_pts, originalsize):
    perspective_m_resvers = cv2.getPerspectiveTransform(dst_pts, src_pts_yfix)
    video_window_PTtrans_reverse =  cv2.warpPerspective(video_window, perspective_m_resvers, originalsize, flags=cv2.INTER_LINEAR)
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
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV Blue Yellow 추출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv,lower_img,upper_img):
    video_adapt = np.copy(video_copy1_PTtrans_blur_normalize4hsv) #hsv정규화 영상 적용
    ##video_adapt = np.copy(video_copy1_PTtrans_blur_normalize4yCrCv) #yCrCv정규화 영상 적용
    hsv_change = cv2.cvtColor(video_adapt, cv2.COLOR_BGR2HSV)

    img_mask = cv2.inRange(hsv_change, lower_img, upper_img)
    #print(img_mask.dtype,'-',img_mask.shape, '-', np.max(img_mask), '-', np.min(img_mask))
    kernel_img = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    opening_img = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, kernel_img, iterations=2)
    closing_img = cv2.morphologyEx(opening_img, cv2.MORPH_CLOSE, kernel_img, iterations=2)
    img_mask_area_detect_morph_rgb = cv2.cvtColor(closing_img, cv2.COLOR_GRAY2BGR)
    img_detect_morph_brg = cv2.bitwise_and(video_adapt, video_adapt, mask=closing_img)
    return img_detect_morph_brg, img_mask_area_detect_morph_rgb
    #blue_mask = cv2.inRange(hsv_change, lower_blue, upper_blue)
    #blue_mask_area_detect = cv2.bitwise_and(white, white, mask=blue_mask)
    #kernel_blue = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    #opening_blue = cv2.morphologyEx(blue_mask_area_detect, cv2.MORPH_OPEN, kernel_blue, iterations=2)
    #closing_blue = cv2.morphologyEx(opening_blue, cv2.MORPH_CLOSE, kernel_blue, iterations=2)
    #blue_mask_area_detect_morph = np.copy(closing_blue) #-간소화 가능
    #blue_mask_area_detect_morph_hsv = cv2.cvtColor(closing_blue, cv2.COLOR_BGR2HSV)
    #blue_mask_morph = cv2.inRange(blue_mask_area_detect_morph_hsv, (0,0,212), (131,255,255))
    #blue_detect_morph_brg = cv2.bitwise_and(video_adapt, video_adapt, mask=blue_mask_morph)

    #cv2.imshow('yellow_detect_morph_brg', yellow_detect_morph_brg)
    #color_brg = cv2.bitwise_or(yellow_detect_morph_brg, blue_detect_morph_brg)
    #color_brg_hsv = cv2.cvtColor(color_brg, cv2.COLOR_BGR2HSV)
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV Blue Yellow 추출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이진화ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh):
    #ㅡㅡㅡㅡㅡㅡㅡtest용ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    #video_binary_check = np.copy(gray2) # gray 정규화 영상 적용(gray)
    #ret_b, binary_detect_check = cv2.threshold(video_binary_check, 205, 255, cv2.THRESH_BINARY)
    #kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    #closing_binary_check = cv2.morphologyEx(binary_detect_check, cv2.MORPH_CLOSE, kernel, iterations=2)
    #opening_binary_check = cv2.morphologyEx(closing_binary_check, cv2.MORPH_OPEN, kernel, iterations=2)
    # ㅡㅡㅡㅡㅡㅡㅡtest용ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ #야간 도로에서 테스트 해보자

    #video_binary = np.copy(video_copy1_PTtrans_blur_normalize4yCrCv) # yCrCv 정규화 영상 적용
    video_binary = np.copy(video_copy1_PTtrans_blur_normalize4hsv) # hsv 정규화 영상 적용
    #video_binary = np.copy(gray2) # gray 정규화 영상 적용(GRAY)
    gray_change = cv2.cvtColor(video_binary, cv2.COLOR_BGR2GRAY)
    ret_b, binary_detect = cv2.threshold(gray_change, white_tresh[0], white_tresh[1], cv2.THRESH_BINARY)
    #ret_b, binary_detect = cv2.threshold(video_binary, 205, 255, cv2.THRESH_BINARY) # gray 정규화 영상 적용할 때
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
    closing_binary = cv2.morphologyEx(binary_detect, cv2.MORPH_CLOSE, kernel, iterations=2)
    opening_binary = cv2.morphologyEx(closing_binary, cv2.MORPH_OPEN, kernel, iterations=2)

    #binary_detect_morph_gray = np.copy(opening_binary) #비활성화 해도 됨
    binary_detect_morph_brg = cv2.cvtColor(opening_binary, cv2.COLOR_GRAY2BGR)
    return binary_detect_morph_brg
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이진화ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ최종 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Final_Bynary_Mix(color_brg, binary_detect_morph_brg, blue_mask_area_detect_morph, yellow_mask_area_detect_morph):
    binary_over = np.zeros_like(binary_detect_morph_brg)
    binary_over[((blue_mask_area_detect_morph == 255)|(yellow_mask_area_detect_morph == 255))&(binary_detect_morph_brg == 255)] = 255
    binary_seperate = cv2.absdiff(binary_over,binary_detect_morph_brg)
    final_line_brg = cv2.bitwise_or(color_brg, binary_seperate)
    return final_line_brg
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ최종 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV lower upper 값 추적용 히스토그램ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def HSV_Tresh_Trace(input_brg_image):
    input_hsv_image = cv2.cvtColor(input_brg_image, cv2.COLOR_BGR2HSV)

    plt.clf()
    histColor = ('b', 'g', 'r')
    binX = np.arange(32)*8
    plt.ylim(0,4000)
    for i in range(3):
        hist = cv2.calcHist(images=[input_hsv_image], channels=[i], mask=None, histSize=[256 / 8], ranges=[0, 256])
        plt.plot(binX, hist, color=histColor[i])
    plt.show()
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡHSV lower upper 값 추적용 히스토그램ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ벼열 변환 및 dtype 변환ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def List_to_Array_and_Type(out_img2, binary_detect_morph_brg):
    out_img2_array = np.array(out_img2)
    #print(out_img2_array.shape,'-',out_img2_array.dtype, '-', np.min(out_img2_array) , '-' , np.max(out_img2_array))
    out_img2_shape = out_img2_array.reshape(binary_detect_morph_brg.shape)
    #print(out_img2_shape.shape, '-', out_img2_shape.dtype, '-', np.min(out_img2_shape), '-', np.max(out_img2_shape))
    out_img2_shape_uint8 = out_img2_shape.astype(np.uint8)
    return out_img2_shape_uint8
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ벼열 변환 및 dtype 변환ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ원본 영상, 윈도우 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
def Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize):
    video_window = np.copy(bynary_and_red_blue_and_poly_area)
    video_window_PTtrans_reverse = PTtrans_Reverse(video_window, src_pts_yfix, dst_pts, originalsize)
    #cv2.imshow('video_window_PTtrans_reverse',video_window_PTtrans_reverse)
    video_window_PTtrans_reverse_gray = cv2.cvtColor(video_window_PTtrans_reverse, cv2.COLOR_BGR2GRAY)
    video_window_PTtrans_reverse_vinary = np.zeros_like(video_window_PTtrans_reverse_gray)
    video_window_PTtrans_reverse_vinary[(video_window_PTtrans_reverse_gray >= 1)] = 255
    video_reverse_diff = np.copy(video_copy1)
    video_reverse_diff[(video_window_PTtrans_reverse_vinary == 255)] = 0
    video_original_window_mix = cv2.add(video_reverse_diff,video_window_PTtrans_reverse)
    #cv2.imshow('video_original_window_mix', video_original_window_mix)
    return video_window_PTtrans_reverse, video_original_window_mix
#ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ원본 영상, 윈도우 영상 합성ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ





#ㅡㅡ영상루프 전 setupㅡㅡㅡ
def find_proper_transper_point(proper_size, sight_height, target_distance , road_width, margin, view_scale, focal_length_horizontal, focal_length_vertical, view_length1, view_length2, PIXEL_CONVERSION_RATE):

    angle_of_view_vertical_center = np.arctan(view_length2 / focal_length_vertical)
    angle_of_view_horizontal_center = np.arctan(view_length1 / focal_length_horizontal)
    angle_of_view_horizontal_bottom = np.arctan(np.cos(angle_of_view_vertical_center) * view_length1 / focal_length_horizontal)
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
    PT_view_height = np.int(round((target_distance - loss_distance) * 100 * view_scale))
    print('PT_view_width = ', PT_view_width, '   PT_view_height = ', PT_view_height)

    road_margin_length_pixel = round(proper_size.shape[1] * ratio_window_target_margin)
    print('center_view_horizontal_center_length = ', (straight_distance * np.tan(angle_of_view_horizontal_center)))
    target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)
    #target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1 - 70], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 90], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 90], [proper_size.shape[1] - 1, proper_size.shape[0] - 1 - 70]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_good.mp4
    #target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 55], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 + 55], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_v1.mp4
    #target_road_margin_srt = np.array([[0, proper_size.shape[0] - 1], [(proper_size.shape[1] - road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 205], [(proper_size.shape[1] + road_margin_length_pixel) / 2 - 1, proper_size.shape[0] / 2 - 1 - 205], [proper_size.shape[1] - 1, proper_size.shape[0] - 1]], dtype=np.float32)  # 영상 중심이 수평선 위인 영상에 임시로 적용해보기 위한 변환좌표 #video_v1.mp4
    dst_bot_pixel = np.int(round(2 * window_face_bottom_length / (road_width + margin) * PT_view_width))
    target_road_margin_dst = np.array([[(PT_view_width - dst_bot_pixel) / 2 - 1, PT_view_height - 1], [0, 0], [PT_view_width - 1, 0], [(PT_view_width + dst_bot_pixel) / 2 - 1, PT_view_height - 1]], dtype=np.float32)

    start_focus_distance = (road_width / 2) / np.tan(angle_of_view_horizontal_center)
    start_zet_distance = (straight_distance - start_focus_distance) * sight_height / target_distance
    start_distance = sight_height * np.tan(view_angle - np.arctan(start_zet_distance / start_focus_distance))
    print('start_distance = ', start_distance)

    vertical_lane = np.array([[0, round((target_distance - start_distance) / PIXEL_CONVERSION_RATE) - 1], [PT_view_width - 1, round((target_distance - start_distance) / PIXEL_CONVERSION_RATE) - 1]], dtype=np.float32)
    straght_lane_left = np.array([[round(PT_view_width / 2 - road_width / 2 / PIXEL_CONVERSION_RATE - 1), PT_view_height - 1], [round(PT_view_width / 2 - road_width / 2 / PIXEL_CONVERSION_RATE - 1), 0]])
    straght_lane_right = np.array([[round(PT_view_width / 2 + road_width / 2 / PIXEL_CONVERSION_RATE - 1), PT_view_height - 1], [round(PT_view_width / 2 + road_width / 2 / PIXEL_CONVERSION_RATE - 1), 0]])
    return target_road_margin_srt, target_road_margin_dst, PT_view_width, PT_view_height, vertical_lane, straght_lane_left, straght_lane_right





#cv brigde sub,pub
class image_converter:

    def __init__(self):
 
        self.image_pub=rospy.Publisher("lanetraker",Image,queue_size=1000)
        self.prediction_pub=rospy.Publisher("lane",Floats,queue_size=100)

        self.bridge = CvBridge()
        self.lanetracker()
        
        #self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def get_image(self):
        try:
            data=rospy.wait_for_message("usb_cam/image_raw",Image)
            cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
           
        except CvBridgeError as e:
            print(e)
       
#        (rows,cols,channels) = cv_image.shape
#  
#        try:
#            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,desired_encodeing="passthrough"))
#        except CvBridgeError as e:
        return  cv_image

    def lanetracker(self):
        global flaots,total_error_left,total_error_right,total_error_half_sum,error_pixel_percent_half_sum,error_pixel_percent_left,error_pixel_percent_right,white_tresh_lower_left,white_tresh_lower_right,white_tresh_lower_half_sum,lower_offset,upper_offset,lc,rc,deviation,left_fit,right_fit,PIXEL_CONVERSION_RATE,video_copy1,originalsize,left_curverad,right_curverad
        rospy.init_node('image_converter', anonymous=True)
        loop = 0

        #cam = cv2.VideoCapture(0)
        #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        video = self.get_image()
        video_copy1 = np.copy(video)

        #fps = cam.get(cv2.CAP_PROP_FPS)


        originalsize = (video.shape[1],video.shape[0]) #h, w, d = video.shape
        print(originalsize)

        flaots.testset.layout.dim.append(MultiArrayDimension())
        flaots.testset.layout.dim.append(MultiArrayDimension())
        flaots.testset.layout.dim[0].label = "height"
        flaots.testset.layout.dim[1].label = "width"
        flaots.testset.layout.dim[0].size = 2
        flaots.testset.layout.dim[1].size = 23
        flaots.testset.layout.dim[0].stride = 2*23
        flaots.testset.layout.dim[1].stride = 23
        flaots.testset.layout.data_offset = 0
        flaots.testset.data=np.zeros((46))
        
        dstride0 = flaots.testset.layout.dim[0].stride
        dstride1 = flaots.testset.layout.dim[1].stride
        offset = flaots.testset.layout.data_offset

        # ㅡㅡ영상루프 전 setupㅡㅡㅡ

        while True:  # @@@@@@@@@@@@@@@@@@@@영상출력루프 시작@@@@@@@@@@@@@@@@@@@@@@
            video = self.get_image()
            video_copy1 = np.copy(video)
            
            video_copy1 = np.copy(video)  # 편집용 video copy1
            video_copy2 = np.copy(video)  # 편집용 video copy2 circle 넣을 거
            loop = loop + 1
            print('loop count : ', loop)

            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ참고용 코드(astype, dtype, shape 등)ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            # binary_all_gray = binary_all_gray.astype(bool)
            # print(binary_all_gray.shape,'-',binary_all_gray.dtype, '-', np.min(binary_all_gray) , '-' , np.max(binary_all_gray))
            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ참고용 코드(astype, dtype, shape 등)ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

            video_copy1_PTtrans = PTtrans_and_Circle(video_copy1, video_copy2, src_pts_yfix, dst_pts, viewsize)
            video_copy1_PTtrans_blur_normalize4hsv = Blur_and_Normalize(video_copy1_PTtrans)  # 영상 블러처리 및 정규화
            lower_yellow = (15, yellow_s_tresh_lower, 90)  # HSV 컬러 영억 검출 임계값 영역 (색상, 채도, 조도)
            upper_yellow = (32, 255, 255)
            lower_blue = (80, 50, 180)
            upper_blue = (115, 255, 255)
            yellow_detect_morph_brg, yellow_mask_area_detect_morph = HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_yellow, upper_yellow)  # HSV 컬러 영역 이진화 + 모폴로지 연산
            blue_detect_morph_brg, blue_mask_area_detect_morph = HSV_Detect(video_copy1_PTtrans_blur_normalize4hsv, lower_blue, upper_blue)
            color_brg = cv2.bitwise_or(yellow_detect_morph_brg, blue_detect_morph_brg)  # 노랑 파랑 검출된 영역 합치기
            color_bynary = cv2.bitwise_or(yellow_mask_area_detect_morph, blue_mask_area_detect_morph)
            white_tresh_left = [white_tresh_lower_left, 255]  # 흰색 차선 검출 임계값 영역(검출되는 영역이 너무 적으면 최솟값을 더 낮추면 됨)
            white_tresh_right = [white_tresh_lower_right], 255]
            # white_tresh_full = [white_tresh_lower_full, 255]
            binary_detect_morph_brg_left = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_left)
            binary_detect_morph_brg_right = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_right)
            # binary_detect_morph_brg_full = White_binary(video_copy1_PTtrans_blur_normalize4hsv, white_tresh_full)
            binary_detect_morph_brg_half_sum = np.zeros_like(video_copy1_PTtrans_blur_normalize4hsv)
            binary_detect_morph_brg_half_sum[:, : binary_detect_morph_brg_half_sum.shape[1] // 2, :] = binary_detect_morph_brg_left[:, : binary_detect_morph_brg_left.shape[1] // 2, :]
            binary_detect_morph_brg_half_sum[:, binary_detect_morph_brg_half_sum.shape[1] // 2:, :] = binary_detect_morph_brg_right[:, binary_detect_morph_brg_right.shape[1] // 2:, :]

            # print('binary_detect_morph_brg_half_sum.shape = ',binary_detect_morph_brg_half_sum.shape)
            final_line_brg = Final_Bynary_Mix(color_brg, binary_detect_morph_brg_half_sum, blue_mask_area_detect_morph, yellow_mask_area_detect_morph)  # 컬러랑 흰색 검출 영역 합치기
            final_line_bynary_left = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_left)
            final_line_bynary_right = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_right)
            final_line_bynary_half_sum = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_half_sum)
            # final_line_bynary_full = cv2.bitwise_or(color_bynary, binary_detect_morph_brg_full)
            # print('sum bynary pixel = ', np.sum(final_line_bynary/255)/3/(final_line_bynary.shape[0]*final_line_bynary.shape[0]),'--',np.sum(final_line_bynary/255)/3)

            if error != 0:
                bynary_half_sum_pixel_percent = np.sum(final_line_bynary_half_sum / 255) / 3 / (final_line_bynary_half_sum.shape[0] * final_line_bynary_half_sum.shape[0])
                target_half_sum_pixel_percent = 0.025
                # white_tresh_lower_half_sum = white_tresh_lower_half * ((bynary_half_sum_pixel_percent + 0.5) / (target_half_sum_pixel_percent + 0.5))
                prev_error_pixel_percent_half_sum = error_pixel_percent_half_sum
                error_pixel_percent_half_sum = bynary_half_sum_pixel_percent - target_half_sum_pixel_percent
                total_error_half_sum = total_error_half_sum + error_pixel_percent_half_sum
                tresh_factor_half_sum = kp * error_pixel_percent_half_sum + ki * total_error_half_sum + kd * (error_pixel_percent_half_sum - prev_error_pixel_percent_half_sum)
                white_tresh_lower_half_sum = white_tresh_lower_half_sum * pow(2, tresh_factor_half_sum)
                if white_tresh_lower_half_sum >= 255: white_tresh_lower_half_sum = 255
                if white_tresh_lower_half_sum <= 0: white_tresh_lower_half_sum = 0

                white_tresh_lower_left = white_tresh_lower_half_sum
                white_tresh_lower_right = white_tresh_lower_half_sum

                print('white_tresh_lower_half_sum = ', white_tresh_lower_half_sum, '   bynary_half_sum_pixel_percent = ', bynary_half_sum_pixel_percent)

            # HSV_Tresh_Trace(input_brg_image) #노랑, 파랑 임계값 추적용 코드

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ차선 좌표 검출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            binary_all_left = np.zeros_like(binary_detect_morph_brg_left)
            binary_all_right = np.zeros_like(binary_detect_morph_brg_right)
            binary_all_half_sum = np.zeros_like(binary_detect_morph_brg_half_sum)
            binary_all_left[(color_bynary == 255) | (binary_detect_morph_brg_left == 255)] = 1
            binary_all_right[(color_bynary == 255) | (binary_detect_morph_brg_right == 255)] = 1
            binary_all_half_sum[(color_bynary == 255) | (binary_detect_morph_brg_half_sum == 255)] = 1
            binary_all_left_gray = cv2.cvtColor(binary_all_left, cv2.COLOR_BGR2GRAY)  # 1채널의 0, 1 boolean data_type
            binary_all_right_gray = cv2.cvtColor(binary_all_right, cv2.COLOR_BGR2GRAY)
            binary_all_half_sum_gray = cv2.cvtColor(binary_all_half_sum, cv2.COLOR_BGR2GRAY)

            # plt.clf() #plot초기화
            add_debug_image1 = True
            add_debug_image2 = True
            add_debug_image2_a = False
            add_debug_image3 = True
            add_debug_image4 = False
            is_debug = True
            debug_images_all = []
            # plt.clf()
            # plot_peak_hist(binary_all_gray, is_plot=True)
            # plt.show()

            if len(left_fit) == 0:
                leftx_base, rightx_base, hist = find_left_right_via_histogram(binary_all_half_sum_gray, add_debug_image=False)  # 영상에서의 왼쪽 오른쪽 차선의 초기 위치를 뽑아내는 함수
                if error == 0:
                    left_fit, right_fit, debug_images1 = find_ploy_fit_window_search(binary_all_half_sum_gray, leftx_base, rightx_base, nwindows=13, margin=round(0.65 / PIXEL_CONVERSION_RATE), is_plot=False, add_debug_image=add_debug_image1)  # 사각 영역을 만들어 차선을 탐지하고 추세선을 만드는 함수
                    if add_debug_image1:
                        add_debug_image1_a = 1
                    if is_debug: print("stage 4 a.. done_1")
            else:
                left_fit, right_fit, lc, rc, deviation, debug_images2 = next_frame_find_poly_already_fitted(binary_all_left_gray, binary_all_right_gray, left_fit, right_fit, lower_offset, upper_offset, margin=round(0.3 / PIXEL_CONVERSION_RATE), is_plot=False,
                                                                                                                                               add_debug_image=add_debug_image2)
                if error == 0:
                    if add_debug_image2:
                        add_debug_image2_a = 1
                    if is_debug: print("stage 4 b.. done_2")
                    window_img_left_gray = cv2.cvtColor(debug_images2[1], cv2.COLOR_BGR2GRAY)
                    window_img_right_gray = cv2.cvtColor(debug_images2[2], cv2.COLOR_BGR2GRAY)
                    bynary_window_image_left_area = cv2.bitwise_and(binary_detect_morph_brg_left, binary_detect_morph_brg_left, mask=window_img_left_gray)
                    bynary_window_image_right_area = cv2.bitwise_and(binary_detect_morph_brg_right, binary_detect_morph_brg_right, mask=window_img_right_gray)
                    bynary_window_image_area = cv2.bitwise_or(bynary_window_image_left_area, bynary_window_image_right_area)
                    bynary_left_pixel_percent = np.sum(bynary_window_image_left_area / 255) / 3 / (bynary_window_image_left_area.shape[0] * bynary_window_image_left_area.shape[0])
                    bynary_right_pixel_percent = np.sum(bynary_window_image_right_area / 255) / 3 / (bynary_window_image_right_area.shape[0] * bynary_window_image_right_area.shape[0])
                    print('white_tresh_lower_left = ', white_tresh_lower_left, '   bynary_left_pixel_percent = ', bynary_left_pixel_percent, 'white_tresh_lower_right = ', white_tresh_lower_right, '   bynary_right_pixel_percent = ', bynary_right_pixel_percent)
                    target_left_pixel_percent = 0.0028
                    target_right_pixel_percent = 0.0028

                    # white_tresh_lower_left = white_tresh_lower_left * ((bynary_left_pixel_percent + 0.15) / (target_left_pixel_percent + 0.15))
                    prev_error_pixel_percent_left = error_pixel_percent_left
                    error_pixel_percent_left = bynary_left_pixel_percent - target_left_pixel_percent
                    total_error_left = total_error_left + error_pixel_percent_left
                    tresh_factor_left = kp * error_pixel_percent_left + ki * total_error_left + kd * (error_pixel_percent_left - prev_error_pixel_percent_left)
                    white_tresh_lower_left = white_tresh_lower_left * pow(2, tresh_factor_left)
                    if white_tresh_lower_left >= 255: white_tresh_lower_left = 255
                    if white_tresh_lower_left <= 0: white_tresh_lower_left = 0

                    # white_tresh_lower_right = white_tresh_lower_right * ((bynary_right_pixel_percent + 0.15) / (target_right_pixel_percent + 0.15))
                    prev_error_pixel_percent_right = error_pixel_percent_right
                    error_pixel_percent_right = bynary_right_pixel_percent - target_right_pixel_percent
                    total_error_right = total_error_right + error_pixel_percent_right
                    tresh_factor_right = kp * error_pixel_percent_right + ki * total_error_right + kd * (error_pixel_percent_right - prev_error_pixel_percent_right)
                    white_tresh_lower_right = white_tresh_lower_right * pow(2, tresh_factor_right)
                    if white_tresh_lower_right >= 255: white_tresh_lower_right = 255
                    if white_tresh_lower_right <= 0: white_tresh_lower_right = 0

                    print('error_pixel_percent_left =', error_pixel_percent_left, 'tresh_factor_left = ', tresh_factor_left, 'pow(2, tresh_factor_left) = ', pow(2, tresh_factor_left))
                    print('error_pixel_percent_right =', error_pixel_percent_right, 'tresh_factor_right = ', tresh_factor_right, 'pow(2, tresh_factor_right) = ', pow(2, tresh_factor_right))
                    add_debug_image4 = True
            # if error == 1 or error == 2 or error == 3: continue
            if error == 0:
                undistored_image = np.copy(video_copy2)

                # debug_images3[0] : 투상변환에서 차선 사이 영역 확인, debug_images3[1] : [0] 역투상 , debug_images3[2] : [1]를 원본 영상 위에 겹쳐서 보여줌 = final_image
                newwarp, final_image, debug_images3 = plot_lanes_unwrap(binary_all_left_gray, left_fit, right_fit, undistored_image, M_INV, is_plot=False, add_debug_image=add_debug_image3)

                if add_debug_image3:
                    #debug_images_all2 = list(itertools.chain.from_iterable(debug_images_all))
                    #composed_image = compose_debug_images(debug_images_all2)
                    composed_image = np.copy(final_image)
                    left_cur_text = "Curvature left: " + "{:0.2f}".format(lc) + " in m"
                    right_cur_text = "Curvature Right : " + "{:0.2f}".format(rc) + " in m"
                    deviation_text = "Deviation : " + "{:0.2f}".format(deviation) + " in m"
                    fontScale = 1
                    thickness = 2
                    fontFace = cv2.FONT_ITALIC

                    cv2.putText(composed_image, left_cur_text, (10, 50), fontFace, fontScale, (255, 255, 255), thickness, lineType=cv2.LINE_AA)
                    cv2.putText(composed_image, right_cur_text, (10, 90), fontFace, fontScale, (255, 255, 255), thickness, lineType=cv2.LINE_AA)
                    cv2.putText(composed_image, deviation_text, (10, 130), fontFace, fontScale, (255, 255, 255), thickness, lineType=cv2.LINE_AA)
                    out.write(composed_image)
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(composed_image,encoding="bgr8"))

            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ차선 좌표 검출ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

            # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡcv2.imshow 이미지 쇼 컨트롤센터 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            imshow_scale = 0.265 / view_scale
            # cv2.imshow('video_copy1_PTtrans',video_copy1_PTtrans) #투상변환 영상
            #video_copy1_PTtrans_resized = cv2.resize(video_copy1_PTtrans, dsize=(round(video_copy1_PTtrans.shape[1] * imshow_scale), round(video_copy1_PTtrans.shape[0] * imshow_scale)))
            #cv2.imshow('video_copy1_PTtrans_resized', video_copy1_PTtrans_resized)

            # cv2.imshow('left', binary_detect_morph_brg_left) #왼쪽 차선 기준 임계값 이진화
            #binary_detect_morph_brg_left_resized = cv2.resize(binary_detect_morph_brg_left, dsize=(round(binary_detect_morph_brg_left.shape[1] * imshow_scale), round(binary_detect_morph_brg_left.shape[0] * imshow_scale)))
            #cv2.imshow('left', binary_detect_morph_brg_left_resized)

            # cv2.imshow('right', binary_detect_morph_brg_right) #오른쪽 차선 기준 임계값 이진화
            #binary_detect_morph_brg_right_resized = cv2.resize(binary_detect_morph_brg_right, dsize=(round(binary_detect_morph_brg_right.shape[1] * imshow_scale), round(binary_detect_morph_brg_right.shape[0] * imshow_scale)))
            #cv2.imshow('right', binary_detect_morph_brg_right_resized)

            # cv2.imshow('final_line_brg', final_line_brg) #투상변환 + 블러, 정규화 + 최종 추출 영상 컬러 버전
            #final_line_brg_resized = cv2.resize(final_line_brg, dsize=(round(final_line_brg.shape[1] * imshow_scale), round(final_line_brg.shape[0] * imshow_scale)))
            #cv2.imshow('final_line_brg_resized', final_line_brg_resized)

            # cv2.imshow('final_line_bynary_full', final_line_bynary_full) #투상변환 + 블러, 정규화 + 최종 추출 영상 이진화 버전
            # cv2.imshow('final_line_bynary_half_sum', final_line_bynary_half_sum)  # 투상변환 + 블러, 정규화 + 최종 추출 영상 이진화 버전
            #final_line_bynary_half_sum_resized = cv2.resize(final_line_bynary_half_sum, dsize=(round(final_line_bynary_half_sum.shape[1] * imshow_scale), round(final_line_bynary_half_sum.shape[0] * imshow_scale)))
            #cv2.imshow('final_line_bynary_half_sum_resized', final_line_bynary_half_sum_resized)

            # video_copy1_PTtrans_resized = cv2.resize(video_copy1_PTtrans, dsize=(round(video_copy1_PTtrans.shape[1] * imshow_scale ), round(video_copy1_PTtrans.shape[0] * imshow_scale )))
            # cv2.imshow('video_copy1_PTtrans_resized', video_copy1_PTtrans_resized)

            #video_copy1_PTtrans_copy1_to_draw_expted_lane = np.copy(video_copy1_PTtrans)
            #cv2.line(video_copy1_PTtrans_copy1_to_draw_expted_lane, (straght_lane_left[0][0], straght_lane_left[0][1]), (straght_lane_left[1][0], straght_lane_left[1][1]), [0, 255, 0], 2)
            #cv2.line(video_copy1_PTtrans_copy1_to_draw_expted_lane, (straght_lane_right[0][0], straght_lane_right[0][1]), (straght_lane_right[1][0], straght_lane_right[1][1]), [0, 255, 0], 2)
            #cv2.line(video_copy1_PTtrans_copy1_to_draw_expted_lane, (vertical_lane[0][0], vertical_lane[0][1]), (vertical_lane[1][0], vertical_lane[1][1]), [0, 255, 0], 2)
            # cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane', video_copy1_PTtrans_copy1_to_draw_expted_lane)
            #video_copy1_PTtrans_copy1_to_draw_expted_lane_resized = cv2.resize(video_copy1_PTtrans_copy1_to_draw_expted_lane, dsize=(round(final_line_bynary_half_sum.shape[1] * imshow_scale), round(final_line_bynary_half_sum.shape[0] * imshow_scale)))
            #cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane_resized', video_copy1_PTtrans_copy1_to_draw_expted_lane_resized)


            # video_copy1_PTtrans_copy1_to_draw_expted_lane_resized = cv2.resize(video_copy1_PTtrans_copy1_to_draw_expted_lane, dsize=(round(video_copy1_PTtrans_copy1_to_draw_expted_lane.shape[1] * imshow_scale ), round(video_copy1_PTtrans_copy1_to_draw_expted_lane.shape[0] * imshow_scale )))
            # cv2.imshow('video_copy1_PTtrans_copy1_to_draw_expted_lane_resized', video_copy1_PTtrans_copy1_to_draw_expted_lane_resized)

            if error == 0:
                # if add_debug_image1_a == 1:
                # sliding_window = np.copy(debug_images1[0])  # 슬라이딩 윈도우  # = debug_images[0]  # 여기서 debug_images 는 이진화영역과 좌표 추출 windows 영역과 그 안에서 검출된 픽셀들 그리고 추세선을 시각화 한 것이다.
                # cv2.imshow('sliding_window', sliding_window) # 바이너리 및 차선 영역

                if add_debug_image2_a == 1:
                    bynary_and_red_blue_and_poly_area = np.copy(debug_images2[0])  # = debug_images[1] 추세선 # 0718-[0](bynary_and_red_blue)를 없애서 [0]이 추세선이됨 # 여기서 debug_images[0]은 이진화 영역과 좌표 추출 영역 안에 포함된 픽셀들을 파랑,빨강,흰색으로 보여주는 이미지이구 [1] 추세선을 시각화 한 것이다
                    #cv2.imshow('bynary_and_red_blue_and_poly_area', bynary_and_red_blue_and_poly_area) # 바이너리 영역에서 추세선과 그 영역 표시 이미지
                    # video_window_PTtrans_reverse, video_original_window_mix = Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize)

                    #bynary_and_red_blue_and_poly_area_resized = cv2.resize(bynary_and_red_blue_and_poly_area, dsize=(round(bynary_and_red_blue_and_poly_area.shape[1] * imshow_scale), round(bynary_and_red_blue_and_poly_area.shape[0] * imshow_scale)))
                    #cv2.imshow('bynary_and_red_blue_and_poly_area_resized', bynary_and_red_blue_and_poly_area_resized)

                    # video_window_PTtrans_reverse, video_original_window_mix = Show_Original_Window_Mix(bynary_and_red_blue_and_poly_area, video_copy1, src_pts_yfix, dst_pts, originalsize)
                    # cv2.imshow('video_window_PTtrans_reverse', video_window_PTtrans_reverse)  # 차선 좌표 추세선 함수와 검출창이 포함된 이미지를 역투상변환 한 이미지
                    # cv2.imshow('video_original_window_mix', video_original_window_mix)  # 차선 좌표 추세선 함수와 검출창이 포함된 이미지를 역투상변환 한 이미지와 원본 영상 모두 보이는 이미지
                #if add_debug_image4 == True:
                    # cv2.imshow('bynary_window_image_area', bynary_window_image_area)
                    #bynary_window_image_area_resized = cv2.resize(bynary_window_image_area, dsize=(round(bynary_window_image_area.shape[1] * imshow_scale), round(bynary_window_image_area.shape[0] * imshow_scale)))
                    #cv2.imshow('bynary_window_image_area_resized', bynary_window_image_area_resized)

                    # bynary_window_image_left_area_resized = cv2.resize(bynary_window_image_left_area, dsize=(round(bynary_window_image_left_area.shape[1] * imshow_scale ), round(bynary_window_image_left_area.shape[0] * imshow_scale )))
                    # bynary_window_image_right_area_resized = cv2.resize(bynary_window_image_right_area, dsize=(round(bynary_window_image_right_area.shape[1] * imshow_scale ), round(bynary_window_image_right_area.shape[0] * imshow_scale )))
                    # cv2.imshow('bynary_window_image_left_area_resized', bynary_window_image_left_area_resized)  # 차선 검출 영역 안의 픽셀들만 보여주는 이미지
                    # cv2.imshow('bynary_window_image_right_area_resized', bynary_window_image_right_area_resized)

                # cv2.imshow('final_image',final_image) # 차선 영역을 초록색으로 색칠한 이미지

                #cv2.imshow('composed_image', composed_image)  # 차선을 초록색으로 덮은 이미지에 곡률과 편차를 화면에 택스트를 표시한 이미지
                # out.write(bynary_window_image_area_resized)
                #out.write(video_copy1_PTtrans)
                # ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡcv2.imshow 이미지 쇼 컨트롤센터 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


            #print('white_tresh_lower_left = ',white_tresh_lower_left, '   bynary_pixel_percent = ',bynary_pixel_percent)
            self.prediction_pub.publish(flaots)



#        except CvBridgeError as e:
#            print(e)

    #   except KeyboardInterrupt:
    #        print("Shutting down")
    #   cv2.destroyAllWindows()


def main():#메인문

    try:
        image_converter()
    except rospy.ROSInterruptException:
#       cam.release()
         out.release()
         cv2.destroyAllWindows()


selected_pts = [] #tf function location array

view_scale = 1.0
sight_height = 1  # 카메라 높이
target_distance = 5  # 목표 거리 (현재 목표 10m전방)
road_width = 3.5  # 차선 폭 (현재 3.5미터 실측 후 조정)
margin = 3.5  # 추가로 투상변환할 폭
focal_length_horizontal = 0.35  # sample_video(f_l_h = 0.4, f_l_v = 0.35)  C525(f_l_h = 0.515, f_l_v = 0.31)
focal_length_vertical = 0.648
view_length1 = 0.3
view_length2 = 0.3
PIXEL_CONVERSION_RATE = 0.01 / view_scale  # 100픽셀당 1미터
ym_per_pix = 0.01 / view_scale  # meters per pixel in y dimension
xm_per_pix = 0.01 / view_scale  # meters per pixel in x dimension
lower_offset = 2 / PIXEL_CONVERSION_RATE  # 최소 차선 감지 폭
upper_offset = 5 / PIXEL_CONVERSION_RATE  # 최대 차선 감지 폭
kp = 2
ki = 0.05
kd = 0.3

proper_size = np.zeros([720, 1280, 3], dtype=np.uint8)
src_pts_yfix, dst_pts, PT_view_width, PT_view_height, vertical_lane, straght_lane_left, straght_lane_right = find_proper_transper_point(proper_size, sight_height, target_distance, road_width, margin, view_scale, focal_length_horizontal, focal_length_vertical, view_length1, view_length2,
                                                                                                                                        PIXEL_CONVERSION_RATE)
M_INV = cv2.getPerspectiveTransform(dst_pts, src_pts_yfix)
viewsize = [PT_view_width, PT_view_height]

lc = 0.0
rc = 0.0
deviation = 0.0
left_fit = []
right_fit = []
white_tresh_lower_left = 145
white_tresh_lower_right = 145
white_tresh_lower_half_sum = 145
yellow_s_tresh_lower = 90
error = 0
bynary_pixel_percent = 0.02
error_pixel_percent_left = 0
total_error_left = 0
error_pixel_percent_right = 0
total_error_right = 0
error_pixel_percent_half_sum = 0
total_error_half_sum = 0


# size = (viewsize[0],viewsize[1]) #투상변환 한 영상 저장할 때
# fps = cam.get(cv2.CAP_PROP_FPS)
fps = 30
fourcc = cv2.VideoWriter_fourcc(*'DIVX')  # 영상 저장 코덱 지정
filename = 'line_detect.avi'  # 저장될 영상 이름 지정
out = cv2.VideoWriter(filename, fourcc, fps, (viewsize[0], viewsize[1]))  # originalsize) #sizn(가로,세로) #영상 저장 설정 세팅
# plt.ion() #plot을 실시간으로 보여주게 해주는 함수



if __name__ == '__main__':
    main()