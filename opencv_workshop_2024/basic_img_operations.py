#! /usr/bin/env python3

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

def main():
    
    # read the image from directory
    img = cv.imread('images/image1.jpg')
    # error handling for reading file
    assert img is not None, "file could not be read"

    #################################################################################
    # INSPECT THE OBJECT PROPERTIES
    #################################################################################

    # # display the image
    # # cv.imshow("Image", img)

    # # Inspect data type
    # print('OpenCV image data type:', type(img))

    # # # Image data shape
    # print(f'Image data shape: {img.shape}')

    # # crop image [rows, columns]
    # cropped_img = img[0:250, :]

    # # display the cropped image
    # cv.imshow("Image", cropped_img)

    # # save cropped image
    # cv.imwrite('cropped_image.jpg', cropped_img)


    #################################################################################
    # SPLIT IMAGE INTO RESPECTIVE CHANNELS
    #################################################################################

    # # convert from BGR -> RGB for sake of matplotlib
    # img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

    # channels = cv.split(img)
    # print(len(channels))
    # titles = ['Original Image', 'Blue', 'Green', 'Red']

    # # # plot the channels
    # plt.subplot(1,4,1)
    # plt.imshow(img_rgb), plt.title(titles[0])

    # for i in range(len(channels)):
    #     plt.subplot(1,4,i+2)
    #     plt.imshow(channels[i])
    #     plt.title(titles[i+1])

    # plt.show()


    #################################################################################
    # COLOR SPACES AND IMAGE THRESHOLDING
    #################################################################################

    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    cv.imshow('Original image', img)
    cv.imshow('HSV color space', img_hsv)

    # thresholding
    COLOR_MIN = (000, 130, 78)
    COLOR_MAX = (27, 255, 255)
    thresh_img = cv.inRange(img_hsv, COLOR_MIN, COLOR_MAX)

    cv.imshow("Thresholded HSV", thresh_img)

    # Bitwise-AND mask and original image
    res = cv.bitwise_and(img, img, mask=thresh_img)

    cv.imshow("Masked image", res)

    #################################################################################



    # hold script till terminate key is pressed
    key = cv.waitKey(0)
    if key == ord("s"):
        cv.destroyAllWindows()


if __name__=='__main__':
    main()