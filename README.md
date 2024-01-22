# Adaptive-Thermal-Camouflage
ADD INTRODUCTION HERE FOR THE ABSTRACT OF PROJECT

TERMS and their definition
  i. camou-pixel: Hexagonal-shaped geometric structure designed on which the thermal gradient of the background scenario is transformed on it with the aid of Active heating/cooling elements                   - termed as ThermoElectric Coolers (TECs). 

1.  Prior to the experimental setup, I have created the pseudo-simulation environment in the Finite Element Machine (FEM) software i.e COMSOL Multiphysics. And written the sample code in Python, for viewing the thermal gradient developed on the surface of camou-pixel.
    cam_plate_thermal_pattern_visualization.py -----> PYTHON file  
    hex_grids_data(20 sec).txt -----> sample file exported from FEM simulation software
   ![img1](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/298e3e8b-9a45-4ec2-9da4-b0baaa691ffb)

2.  camou-pixel_I GUI.py  ----->   further amendments are made in the GUI code to grabbing the contour of pixel, determining the average temperature of      the camou-pixel and its background.
    Testing code for acquiring the contour of camou-pixel-I by implementing Machine Vision Techniques (MVT) for model training, which   
    includes:
    i. Identity Linear transformation Method (ILTM) ----> for transformation of spatial thermal frame to gray scale (0-255) domain
    ii. Canny Edge Detection Method (CEDM) ----> for determining the contour of camou-pixel-I.
    Below figure depicts the detected contour of camou-pixel from the thermal frame  
    ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/7ea353eb-63ac-4222-b923-17d799f07d43)

    iii. Harris Corner Detection Method (HCDM) ----> for determining the corner coordinates of camou-pixel-I.
    iv. Dilation Morphological Operation(DMO) ----> The fact that HCDM does not determine the single corner point of the camou-pixel-I            geometry due to the low resolution of the captured thermal frame. Thus, it is required to enhance the corner points of the       
        geometry i.e. cluster of pixel at the corners of thermal frame, and determining its centroid in order to obtain a single corner   
        pixel of the camou-pixel-I.
        Below figure depicts the detected corner points using HCDM and the enhance corners using DMO. It is visualized that the edges are
        also dilated, to suppress the edges the entire frame is transformed to gray scale frame.  
    ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/199c37b7-4641-4314-9530-190bd5699dcf)

   v. Sub-Pixel Accuracy (SPA) algorithm ----> This algorithm is based on Orthogonal Vector Theory (OVT) and is utilized for determining the precise corners of camou-pixel-I from the cluster of pixels, obtained from DMO.


shape_detection_v6.py   -----> code for
