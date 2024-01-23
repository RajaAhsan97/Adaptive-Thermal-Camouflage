# Adaptive-Thermal-Camouflage
ADD INTRODUCTION HERE FOR THE ABSTRACT OF PROJECT

TERMS and their definition
  i. camou-pixel: Hexagonal-shaped geometric structure designed on which the thermal gradient of the background scenario is transformed on it with the aid of Active heating/cooling elements- termed as ThermoElectric Coolers (TECs). 

1.  Prior to the experimental setup, I have created the pseudo-simulation environment in the Finite Element Machine (FEM) software i.e COMSOL Multiphysics. And written the sample code in Python, for viewing the thermal gradient developed on the surface of camou-pixel.
    cam_plate_thermal_pattern_visualization.py -----> PYTHON file  
    hex_grids_data(20 sec).txt -----> sample file exported from FEM simulation software
   ![img1](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/298e3e8b-9a45-4ec2-9da4-b0baaa691ffb)

CAMOU-PIXEL-I:

2.  camou-pixel_I GUI.py  ----->   further amendments are made in the GUI code to grabbing the contour of pixel, determining the average temperature of the camou-pixel and its background.

3.  The thermal frame saved from the above GUI code is used for the testing code (shape_detection_v6.py) for acquiring the contour of camou-pixel-I by implementing Machine Vision Techniques (MVT) for model training, which   
    includes:
    i. Identity Linear transformation Method (ILTM) ----> for transformation of spatial thermal frame to gray scale (0-255) domain
    ii. Canny Edge Detection Method (CEDM) ----> for determining the contour of camou-pixel-I.
    Below figure depicts the detected contour of camou-pixel from the thermal frame.  
    ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/7ea353eb-63ac-4222-b923-17d799f07d43)

    iii. Harris Corner Detection Method (HCDM) ----> for determining the corner coordinates of camou-pixel-I.
    iv. Dilation Morphological Operation(DMO) ----> The fact that HCDM does not determine the single corner point of the camou-pixel-I geometry due to the low resolution of the captured thermal frame. Thus, it is required to enhance the corner points of the       
        geometry i.e. cluster of pixel at the corners of thermal frame, and determining its centroid in order to obtain a single corner   
        pixel of the camou-pixel-I.
        Below figure depicts the detected corner points using HCDM and the enhance corners using DMO. It is visualized that the edges are
        also dilated, to suppress the edges the entire frame is transformed to the gray scale frame as shown in the the figure labelled   
        (c).  
        ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/199c37b7-4641-4314-9530-190bd5699dcf)

   v. Sub-Pixel Accuracy (SPA) algorithm ----> This algorithm is based on Orthogonal Vector Theory (OVT) and is utilized for determining the precise corners of camou-pixel-I from the cluster of pixels, obtained from DMO. 
      Below figure depicts the determined corner points by using SPA aalgorithm. Where the blue-colored pixels represents the corners of camou-pixel-I in thermal frame and green-colored pixels represents the centroid of the clusters of pixels at the corners.
      ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/0f0e823d-d45e-4cea-8fbc-925d57ad765c)

      Below figure depicts the regions of camou-pixel-I and its surrounding. The coordinates of these regions are grabbed into an array in the GUI code and average temperature of both the regions are calculated. Which will be transmitted serially to the micro-controller as the set-point and process-variable. The average temperature of the background is termed as "set-point" and the average temperature of camou-pixel-I is termed as "process-variable".
  ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/3683afbc-0835-4391-8ae7-729546566372)


      Below figure depicts the the view of camou-pixel-I in the real-time thermal frame with uniform background region (located on left) and the real thermal background region (located on right).
  ![image](https://github.com/RajaAhsan97/Adaptive-Thermal-Camouflage/assets/155144523/3ccc6a0a-48bf-478f-8cb6-b5bb0ae48b1d)

4. xthermpyshowpyqt_(average_temps)v6.py ----> is the actual code in which all the above discussed techniques of machine vision for trained model is implemented and also connection of python with micro-controller is initiated. Which is used to blend the camou-pixel-I in the surrounding thermal background region by processing the acquired set-point and process-variable with the aid of Proportional Integrative and Derivative (PID) control algorithm.  

5. TEC_PID_ctrl_single_pic_camouflage_v4.ino  ----> is the PID control algorithm code implemented on micro-controller for minimizing the error between the set-point (background temperature) and the process-variable (camou-pixel-I temperature).

CAMOU-PIXEL-II:

6. Pixelated_CV90.emf  ----> array of camou-pixel-II for creating the structure of Object

7. ATC system II- CO SP extraction.py  ----> for grabbing set-points for every TECs on camou-pixel-II array forming the structure of CO
8. ATC system II- CO pixels IQA compute.py  ----> for computation of IQA metrics of camou-pixels-II with the background regions 



6.  ATC_cam_model2_v4.py  ----> code for dividing the camou-pixel-II geometry into six sub-regions and defining its surrounding regions for the purpose to acquire the average temperatures. Thus the total of six set-points and six process-variables are obtained for processing it with PID control algorithm.   

7. ATC_systemII_GUI.py   ----> Final code 

8. CTGP-II Locations Temperatures.txt ----> file in which set-points of every TECs are stored, which is then used for the PID controller 
