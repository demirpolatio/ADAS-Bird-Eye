# -*- coding: utf-8 -*-
"""
Created on Sun Apr 17 17:14:25 2022

@author: DenizD
"""

import cv2
import numpy as np


MAXFRME = 300

def extract_frame_from_videos(path1, path2, path3, path4):
    
    vidcap = cv2.VideoCapture(path1)
    success,image = vidcap.read()
    count = 0
    success = True
    framesF = []
    while success and count <MAXFRME:
        framesF.append(image)
        success,image = vidcap.read()
        count += 1
    framesF = np.array(framesF)



    vidcap = cv2.VideoCapture(path2)
    success,image = vidcap.read()
    count = 0
    success = True
    framesB = []
    while success and count <MAXFRME:
        framesB.append(image)
        success,image = vidcap.read()
        count += 1
    framesB = np.array(framesB)



    vidcap = cv2.VideoCapture(path3)
    success,image = vidcap.read()
    count = 0
    success = True
    framesL = []
    while success and count <MAXFRME:
        framesL.append(image)
        success,image = vidcap.read()
        count += 1
    framesL = np.array(framesL)


    vidcap = cv2.VideoCapture(path4)
    success,image = vidcap.read()
    count = 0
    success = True
    framesR = []
    while success and count <MAXFRME:
        framesR.append(image)
        success,image = vidcap.read()
        count += 1
    print(count, " frames extracted")
    framesR = np.array(framesR)
    print("data shape =\t", framesR.shape)

    framesArray = [framesF, framesB, framesL, framesR]
    
    
    return framesArray


carImg = cv2.imread('araba/koluman.png')
(carH, carW) = carImg.shape[:2]
carImg.shape




def inputframes_to_finalframe(imgF, imgB, imgL, imgR):


 #-------------------------------------------------------------------------------------------------------------------
    
    # Pixel values in original image
    red_pointF = [280,210]
    green_pointF = [370,210]
    black_pointF = [70,350]
    blue_pointF = [630,354]

    # Create point matrix
    point_matrixF = np.float32([red_pointF,green_pointF,black_pointF, blue_pointF])

    
    # Output image size
    widthF, heightF = 600,400

    # Desired points value in output images

    converted_red_pixel_valueF = [0,0]
    converted_green_pixel_valueF = [widthF,0]
    converted_black_pixel_valueF = [0,heightF]
    converted_blue_pixel_valueF = [widthF,heightF]

    # Convert points
    converted_pointsF = np.float32([converted_red_pixel_valueF,converted_green_pixel_valueF,
                                   converted_black_pixel_valueF,converted_blue_pixel_valueF,
                                   ])

    # perspective transform
    perspective_transformF = cv2.getPerspectiveTransform(point_matrixF,converted_pointsF)
    img_OutputF = cv2.warpPerspective(imgF,perspective_transformF,(widthF,heightF))
    
    
#-------------------------------------------------------------------------------------------------------------------


 # Pixel values in original image
    red_pointB = [0,277]
    green_pointB = [284,200]
    black_pointB = [640,250]
    blue_pointB = [345,197]

    # Create point matrix
    point_matrixB = np.float32([red_pointB,green_pointB,black_pointB, blue_pointB])

    # Output image size
    widthB, heightB = 400,600

    # Desired points value in output images

    converted_red_pixel_valueB = [0,heightB]
    converted_green_pixel_valueB = [widthB,heightB]
    converted_black_pixel_valueB = [0,0]
    converted_blue_pixel_valueB = [widthB,0]


    # Convert points
    converted_pointsB = np.float32([converted_black_pixel_valueB,converted_blue_pixel_valueB,
                                   converted_red_pixel_valueB,converted_green_pixel_valueB
                                   ])

    # perspective transform
    perspective_transformB = cv2.getPerspectiveTransform(point_matrixB,converted_pointsB)
    img_OutputB = cv2.warpPerspective(imgB,perspective_transformB,(widthB,heightB))
    #img_Output = cv2.rotate(img_Output, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img_OutputB = cv2.rotate(img_OutputB, cv2.ROTATE_90_CLOCKWISE)
      

#-------------------------------------------------------------------------------------------------------------------


    red_pointL = [80,330]
    green_pointL = [160,204]
    black_pointL = [500,330]
    blue_pointL = [400,210]

    # Create point matrix
    point_matrixL = np.float32([red_pointL,green_pointL,black_pointL, blue_pointL])


    widthL, heightL = 400,500

    # Desired points value in output images

    converted_red_pixel_valueL = [0,0]
    converted_green_pixel_valueL = [widthL,0]
    converted_black_pixel_valueL = [0,heightL]
    converted_blue_pixel_valueL = [widthL,heightL]


    # Convert points
    converted_pointsL = np.float32([converted_red_pixel_valueL,converted_green_pixel_valueL,
                                   converted_black_pixel_valueL,converted_blue_pixel_valueL])

    # perspective transform
    perspective_transformL = cv2.getPerspectiveTransform(point_matrixL,converted_pointsL)
    img_OutputL = cv2.warpPerspective(imgL,perspective_transformL,(widthL,heightL))
    img_OutputL = cv2.rotate(img_OutputL, cv2.ROTATE_180)



#-------------------------------------------------------------------------------------------------------------------


    red_pointR = [120,360]
    green_pointR = [240,195]
    black_pointR = [640,370]
    blue_pointR = [400,195]

    # Create point matrix
    point_matrixR = np.float32([red_pointR,green_pointR,black_pointR, blue_pointR])

    widthR, heightR = 400,500

    # Desired points value in output images
    converted_red_pixel_valueR = [0,heightR]
    converted_green_pixel_valueR = [widthR,heightR]
    converted_black_pixel_valueR = [0,0]
    converted_blue_pixel_valueR = [widthR,0]


    # Convert points
    converted_pointsR = np.float32([converted_black_pixel_valueR,converted_blue_pixel_valueR,
                                   converted_red_pixel_valueR,converted_green_pixel_valueR
                                   ])

    # perspective transform
    perspective_transformR = cv2.getPerspectiveTransform(point_matrixR,converted_pointsR)
    img_OutputR = cv2.warpPerspective(imgR,perspective_transformR,(widthR,heightR))


#-------------------------------------------------------------------------------------------------------------    
    
    
    final_Frame = np.zeros(shape=(1300,870,3),dtype=np.int16)

    final_Frame[0:400, 100:700] = img_OutputF[:,:]
    final_Frame[900:1300, 100:700] = img_OutputB[:,:]
    final_Frame[400:900, 0:400] = img_OutputL[:,:]
    final_Frame[400:900, 470:870] = img_OutputR[:,:]


    final_Frame[400:400+carH, 320:320+carW] = carImg[:,:]
    final_Frame = final_Frame[0:1300, 100:700]


    cv2.line(final_Frame,(0,400),(870,400),(0,0,0),10)
    cv2.line(final_Frame,(0,900),(870,900),(0,0,0),10)
    cv2.line(final_Frame,(384,400),(384,900),(0,0,0),10)
    cv2.line(final_Frame,(216,400),(216,900),(0,0,0),10)
    
    return final_Frame
    
    
    
def main():
    
    
    
    framelistesi = extract_frame_from_videos("front_record.mp4", "back_record.mp4", "left_record.mp4", "right_record.mp4")
    
    
    finalframes = []
    
    
#     size = 1300, 600
#     out = cv2.VideoWriter('./finalframes/output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (size[1], size[0]), False)
#     for i in range(MAXFRME):
#         finalframes.append(inputframes_to_finalframe(framelistesi[0][i], framelistesi[1][i], framelistesi[2][i], framelistesi[3][i]))
#         finalframes[i] = finalframes[i][:,:,0]
#         out.write(finalframes[i])
#     out.release()  
    
    
#     size = 1300, 600
#     out = cv2.VideoWriter('./ADASoutput.mp4', cv2.VideoWriter_fourcc('M','J','P','G'), 30, (size[1], size[0]), True)
    
#     for i in range(MAXFRME):
#         finalframes.append(inputframes_to_finalframe(framelistesi[0][i], framelistesi[1][i], framelistesi[2][i], framelistesi[3][i]))
#         out.write(finalframes[i].astype('uint8'))
#     out.release()  
    
    
    size = 1300, 600
    out = cv2.VideoWriter('./ADASoutput.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (size[1], size[0]), True)
    
    for i in range(MAXFRME):
        finalframes.append(inputframes_to_finalframe(framelistesi[0][i], framelistesi[1][i], framelistesi[2][i], framelistesi[3][i]))
        out.write(finalframes[i].astype('uint8'))
    out.release()  
    
    
    

    print("done")

                                      
                          
if __name__ == '__main__':
    main()