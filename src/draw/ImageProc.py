'''
File name: ImageProc.py
Author: 肖镇龙（Zhenlong Xiao）
Description: Image processing
'''

import numpy as np
import cv2
from skimage import morphology
from Common.DrKDtree import KDtree
from geomdl import NURBS
from geomdl import BSpline
from geomdl import utilities
from geomdl import exchange
from geomdl import operations

def skeletonize(image: np.ndarray):
    """
    This is a function called skeletonize that takes in an image as a numpy array and returns the skeletonized version of that image.

    Args:
        image (np.ndarray): A 2D numpy array representing the image to be skeletonized.

    Returns:
        np.ndarray: A 2D numpy array representing the skeletonized version of the input image.
    """
    return morphology.skeletonize(image)


def binaryFilter(image: np.ndarray):
    """
    This function applies a binary threshold to an input image using OpenCV's cv2.threshold() method with Otsu's thresholding.

    Args:
        image: A numpy array representing the input image.

    It multiplies each pixel value in the input image by 255 and converts it to an 8-bit unsigned integer.
    Then, it applies the binary threshold using a combination of cv2.THRESH_BINARY and cv2.THRESH_OTSU flags.
    The threshold value is automatically determined by Otsu's algorithm.

    Finally, the thresholded image is returned.
    """
    
    # Multiply each pixel value by 255 and convert to 8-bit unsigned integer
    gray = image*255
    gray = gray.astype(np.uint8)
    # Apply binary threshold using cv2.threshold() with Otsu's algorithm
    _, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # Return the thresholded image
    return dst

def getPointFromImage(skeleton_image, pForcolor=True):
    """
    This function retrieves the coordinates of all points on a line from an input binary image.

    Args:
        skeleton_image: A binary image containing the line skeleton.
        pForcolor: A boolean parameter indicating whether to retrieve foreground (True) or background (False) pixel coordinates. 
                   Default value is True.

    Returns:
        list: A list containing the coordinates of all the points on the line as [x, y] pairs.
    """
    
    # Return a list containing the coordinates of all the points as [x, y] pairs
    return np.argwhere(skeleton_image == pForcolor).astype(np.float)

def getBSplineCurve(Points: np.ndarray):
    """
    Creates a BSpline curve using the input point set by creating a NURBS model.

    Args:
        Points (ndarray): An array containing the control points used to create the BSpline curve.

    Returns:
        NURBS.Curve: A curve object containing information about the generated BSpline curve.

    This function takes in an array of control points used to generate a BSpline curve. It first converts the NumPy array of points to a list and then uses the KDTree algorithm to sort the points in order on the curve.

    With the sorted list of points, a new NURBS curve object is created which has the following properties:
        - `degree`: Degree of the curve
        - `knotvector`: Knot vector of the curve
        - `ctrlpts`: Control points of the curve
        - `delta`: Evaluation delta for the curve

    The degree of the curve is set to 5, and the control points are set to the sorted list of points. The knot vector is generated automatically based on the degree and number of control points.

    Finally, the evaluation delta for the curve is set and the NURBS curve object is returned.
    """

    # Convert NumPy array of points to list
    point_list = Points.tolist()

    # Use KDTree to sort points in order on the curve
    kdtree = KDtree(point_list)
    pt_list = kdtree.sort()

    # Create new NURBS curve object
    curve = NURBS.Curve()

    # Set degree of the curve
    curve.degree = 5

    # Set control points of the curve to the sorted list of points
    curve.ctrlpts = pt_list

    # Generate knot vector automatically based on degree and number of control points
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlpts))

    # Set evaluation delta for the curve
    curve.delta = 0.01

    # Return the NURBS curve object
    return curve

def getNurbsValue(curve: NURBS.Curve, value: float):
    """
    Returns the point on the provided NURBS curve corresponding to the given value.

    Args:
        curve (NURBS.Curve): An object of type NURBS.Curve representing a NURBS curve.
        value (float): A float between 0 and 1. This parameter determines where on the curve
                       the point is found.

    Returns:
        list: A list containing two values. The first value is the y-coordinate of the point,
              and the second value is the x-coordinate.
    """

    # Evaluate the NURBS curve at the specified value (i.e. point on the curve)
    curve.evaluate(start=value, stop=value)

    # Return the evaluated point as a list containing two values:
    # the y-coordinate and the x-coordinate respectively.
    return [curve.evalpts[0][1], curve.evalpts[0][0]]



def concentration(x: np.ndarray, coefficients, Q, time: float):
    '''
    Calculates the concentration of a diffusing substance at a distance x from its source.

    Args:
        x (np.ndarray): 到释放源的距离 (distance from the release point)
        coefficients (float): 扩散系数 (diffusion coefficient)
        Q (float): 释放的分子两 (amount of substance released)
        time (float): 扩散时间 (time elapsed since release)

    Returns:
        The concentration of the substance at distance x.
    '''
    return (Q / (2 * np.sqrt(np.pi * coefficients * time))) * np.exp(-np.power(x, 2)/(4 * coefficients * time))

def laplacian(image: np.ndarray):
    """
    Calculates the Laplacian of the input image.

    Args:
        image (np.ndarray): The input image.

    Returns:
        np.ndarray: The Laplacian of the input image, which is the sum of second-order partial derivatives with respect to x and y.
    """
    
    # Initializing arrays to store partial derivatives
    image_x = np.zeros_like(image)
    image_y = np.zeros_like(image)
    image_x2 = np.zeros_like(image)
    image_y2 = np.zeros_like(image)
    
    # Calculating partial derivatives using central difference formula
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            if row-1 < 0:
                image_y[row, col] = image[row+1, col]-image[row, col]
            elif row+1 >= image.shape[0]:
                image_y[row, col] = image[row, col]-image[row-1, col]
            else:
                image_y[row, col] = (image[row+1, col]-image[row-1, col])/2
            if col-1 < 0:
                image_x[row, col] = image[row, col+1]-image[row, col]
            elif col+1 >= image.shape[1]:
                image_x[row, col] = image[row, col]-image[row, col-1]
            else:
                image_x[row, col] = (image[row, col+1]-image[row, col-1])/2
    
    # Calculating second-order partial derivatives using same logic as above
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            if row-1 < 0:
                image_y2[row, col] = image_y[row+1, col]-image_y[row, col]
            elif row+1 >= image.shape[0]:
                image_y2[row, col] = image_y[row, col]-image_y[row-1, col]
            else:
                image_y2[row, col] = (image_y[row+1, col]-image_y[row-1, col])/2
            if col-1 < 0:
                image_x2[row, col] = image_x[row, col+1]-image_x[row, col]
            elif col+1 >= image.shape[1]:
                image_x2[row, col] = image_x[row, col]-image_x[row, col-1]
            else:
                image_x2[row, col] = (image_x[row, col+1]-image_x[row, col-1])/2
    
    # Returning the Laplacian of the input image
    return image_x2 + image_y2


def laplacian1D(image: np.ndarray):
    """
    applies Laplacian 1D filter on a given image 

    Args:
        image (np.ndarray): the input image to apply the filter on.

    Returns:
        np.ndarray: the filtered output image.
    """    
    
    # Creating two arrays of zeros with the same shape as the input image
    image_x = np.zeros_like(image) 
    image_x2 = np.zeros_like(image)
    
    # Looping over each column of the image and computing the gradient using the central difference method
    for col in range(image.shape[0]):
        if col - 1 < 0: # edge case where the leftmost column is being processed
            image_x[col] = image[col + 1] - image[col]
        elif col + 1 >= image.shape[0]: # edge case where the rightmost column is being processed
            image_x[col] = image[col] - image[col - 1]
        else: # general case where a column somewhere in the middle is being processed
            image_x[col] = (image[col+1] - image[col - 1])/2
    
    # Looping over each column of the gradient image and computing the second order gradient using central difference method
    for col in range(image.shape[0]):
        if col - 1 < 0:
            image_x2[col] = image_x[col + 1] - image_x[col]
        elif col + 1 >= image.shape[0]:
            image_x2[col] = image_x[col] - image_x[col - 1]
        else:
            image_x2[col] = (image_x[col+1] - image_x[col - 1])/2
    
    # Returning the final filtered image after applying laplacian filter
    return image_x2


def rotateImage(image: np.ndarray, degree: float):
    """
    Rotates an input image by a specified angle (in degrees) counterclockwise around its center.

    Args:
        image (np.ndarray): The input image to be rotated.
        degree (float): The amount of rotation in degrees (positive values indicate counterclockwise rotation).

    Returns:
        np.ndarray: The rotated output image.
    """

    # Extracting the number of rows and columns of the input image
    rows = image.shape[0]
    cols = image.shape[1]

    # Computing the rotation matrix using built-in OpenCV function cv2.getRotationMatrix2D
    # This matrix specifies the characteristics of the rotation transformation
    M = cv2.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), degree, 1)

    # Applying the transformation on the input image using built-in OpenCV function cv2.warpAffine
    dst = cv2.warpAffine(image, M, (cols, rows))

    # Returning the rotated output image
    return dst

def readImage(dir):
    """
    Reads an image from a specified file path.

    Args:
        dir (str): The file path of the image to be read.

    Returns:
        np.ndarray: The image read from the specified file path.
    """

    # Reading an image from the specified file path using openCV library's built-in imread function
    image = cv2.imread(dir)

    # Returning the read image
    return image



def resizeImage(image, sideLen=100):
    """
    Resizes an image to a specified length using openCV library's built-in resize function.

    Args:
        image (np.ndarray): The image to be resized.
        sideLen (int): The desired length for each side of the image. Default is 100.

    Returns:
        np.ndarray: The resized image as a numpy array.
    """

    # Using openCV library's built-in resize function to resize the input image to the specified length
    return cv2.resize(image, (sideLen, sideLen))


def getPatternFromImage(dir, sideLen=100):
    """
    Reads an image file and extracts a binary pattern from the image. 

    Args:
        dir (str): The directory containing the image file.
        sideLen (int): The desired length for each side of the image. Default is 100.

    Returns:
        tuple: A tuple containing:
            - a binary pattern extracted from the image as a numpy array
            - an empty numpy array to define a mask (used in other parts of the code)

    This function defines a new numpy array to store the binary pattern extracted from the image. 
    It loops over every pixel in the resized image array and sets the corresponding value in the pattern array to 1 
    if the pixel at that location is black (has RGB values of less than 1). 
    Finally, it returns both the extracted pattern and an empty mask.
    """

    # Reading the image from the specified directory using the readImage() function
    image = readImage(dir)

    # Resizing the image using the resizeImage() function
    image = resizeImage(image, sideLen)

    # Creating a new numpy array with zeros of size (sideLen, sideLen)
    # This will be used to store the binary pattern extracted from the image later 
    pattern = np.zeros((sideLen, sideLen), dtype=np.uint8)

    # Creating another numpy array with zeros of size (sideLen, sideLen)
    # This will be used as an empty mask (masking is done in other parts of the code)
    mask = np.zeros((sideLen, sideLen), dtype=np.uint8)

    # Looping over every row and column in the resized image array
    for row in range(sideLen):
        for col in range(sideLen):
            # If the pixel at the current row and column has RGB values of less than 1 (indicating black)
            # set the corresponding value in the pattern array to 1
            if image[row, col, 0] < 1 and image[row, col, 1] < 1 and image[row, col, 2] < 1:
                pattern[row, col] = 1

    # Return both the extracted pattern and an empty mask to define it
    return pattern, mask

