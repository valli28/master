import cv2 as cv2
import numpy as np

d65_whitepoint =  (0.3127, 0.3290, 0.3583)

f = 3446.79393
k1 = 0.115801
k2 = -0.253555
k3 = 0.33047
p1 = -0.0522358
p2 = -0.00423041

distortion_coefficients = (k1, k2, p1, p2, k3)

fx = f
fy = -f
cx = -884.651
cy = -68.3818

camera_parameters = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]])

def find_3_channel_stats(img):
    # 3-channel descriptive statistics (mean and variance): 
    blueVals = np.reshape(img[:,:,0], -1)
    greenVals = np.reshape(img[:,:,1], -1)
    redVals = np.reshape(img[:,:,2], -1)
    stats3 = np.array([[np.mean(blueVals), np.std(blueVals)],
                          [np.mean(greenVals), np.std(greenVals)], 
                          [np.mean(redVals), np.std(redVals)]])
    return stats3

def mahal_dist(img, ref_img, BGR_stats):     # 4. (Mahalanobis)Distance in RGB space to a reference color
    # We start by calculating the covariance matrix of the BGR test_image
    blueVals = np.reshape(ref_img[:,:,0], -1)
    greenVals = np.reshape(ref_img[:,:,1], -1)
    redVals = np.reshape(ref_img[:,:,2], -1)
    x = np.array([blueVals, greenVals, redVals]) # nx3 array of all channels
    cov_matrix = np.cov(x)

    # white point instead of reference image
    ref_color = np.array([BGR_stats[0][0], BGR_stats[1][0], BGR_stats[2][0]])
    #ref_color = np.array([d65_whitepoint[0], d65_whitepoint[1], d65_whitepoint[2]]) # Try printing the original


    reshaped_img = np.array([np.reshape(img[:,:,0], -1), np.reshape(img[:,:,1], -1), np.reshape(img[:,:,2], -1)])

    diff = (reshaped_img.T - ref_color)
    boi = np.dot((np.linalg.inv(cov_matrix)), diff.T)

    d = np.multiply(boi.T, diff)
    # d is now a distance vector with 250.000 number of elements (500x500)
    d = np.sqrt(np.sum(d,1))
    d = np.reshape(d, (img.shape[0], img.shape[1]))

    ret, binary_img = cv2.threshold(d, 4, 255, cv2.THRESH_BINARY_INV) # The threshold value is found by trial and error
    # The convert functions in compare_images had difficulties with the resulting binary_img due to depth problems
    # .astype solves that
    final_Mahal_img = binary_img.astype('uint8')
    #compare_images(img, binary_img, 'Hello', 'binary')
    #cv2.imwrite('output/14_Mahal_unsegmented_img.png', d)
    #cv2.imwrite('output/14_Mahal_segmented_img.png', final_Mahal_img)
    return final_Mahal_img

# Function to extract frames 
def FrameCapture(path): 
      
    # Path to video file 
    vidObj = cv2.VideoCapture(path)
    # Read reference image
    ref_img = cv2.imread("ref_img.png")
    #ref_img = cv2.cvtColor(ref_img, cv2.COLOR_XYZ2RGB)
    kernel = np.ones((3,3),np.uint8)
  
    # checks whether frames were extracted 
    ret = 1
  
    while ret: 
  
        # "Huske" ting som bliver ved med at have en hoej varmestraaling i billedet, saadan saa at reflektioner og midten af billedet (som altid er mere exposed appearently) bliver subtraheret.
        # Derefter estimere punkternes poisioner og samle dem i "blobs". 
        # Disse blobs har saa koordinater som kan projekteres paa en 3D model bagefter.
        # The halo-effect is appearently called a vignetting effect, and is often prominent in RGB images with bad lenses. 
        
        
        # vidObj object calls read 
        # function extract frames 
        ret, img = vidObj.read() 
        #img = cv2.undistort(img, camera_parameters, distortion_coefficients)
        
        #Resize the image for easier calculation and prototyping
        img = cv2.resize(img,(960,540))
        
        cv2.imshow("Original", img)

        #img = cv2.cvtColor(img, cv2.COLOR_XYZ2RGB)

        stats = find_3_channel_stats(ref_img)
        mahal_img = mahal_dist(img, ref_img, stats)

        #Do some closing
        
        mahal_img = cv2.morphologyEx(mahal_img, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("Segmented image", mahal_img)

        if cv2.waitKey(1) == 27:
            break

        
    # When everything done, release the capture
    cv2.cap.release()
    cv2.destroyAllWindows()
  
# Driver Code 
if __name__ == '__main__': 
  
    # Calling the function 
    FrameCapture("iron.mp4") 