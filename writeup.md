## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  


### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.


What I changed in the notebook is as follows.
	I added a lower and upper boundary to the color_thresh function. This allows me to provide a range of acceptable values when looking for objects. The lower and upper bounds for each item was (160,160,160) - (255,255,255) for navigable terrain. (0,0,0) - (100,100,100) for obstacles, and (110, 110,5) - (210, 210, 145) for rocks/objective. From there I modified the rotate and translate function to be one function, mostly for ease of use. 
	Finally we get to the pandas portion, from here I just ran it using data provided as well as some recorded data I saved. All I really had to do here was make sure my delimiter was set to the correct ';' instead of ','.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And another! 
	In the process_image() function I set up source and destination points, as well as a destination size and offset. 

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
	For this portion of the code I mostly just followed along with the steps done in the notebook but applied them to data in real time. I changed the color_thresh function to allow for a lower and upper bound. I also combined the rotate and translate pixel functions into one function for ease of use. From here rather than reading in images from recordings or sample data I just read it in real time from Rover.img. 
	I applied my values that I found worked for rocks, obstacles, and navigable terrain from the notebook to this same process. The extra step I had to do here was output the results of these thresholded arrays to the output video feed on the Rover. This was done by assigning the Rover.vision_image to the appropriate thresholded images. 
	From there I had to convert the image pixel values to coordinates that my rover could understand. I did this just by calling the rover_coords function provided. From here all that was left was to change these rover coordinates to world coordinates. All you need to know for this is how big you want your world map to be as well as an appropirate scale to go from rover to world values. I chose 10 and 200 respectively. 
	The last few steps here were to use my pix_to_world function which was just a combination of the rotate and translate functions for ease of use. Finally we update the world map with these array/image values and let the rover know of the new angle and distance values. 

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: Resolution: 1920x1200 and FPS was anywhere from 30-50 but it averaged 45 as far as I can see**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  
So in terms of what is broken there are a few things, one my Rover gets stuck on rocks occasionally, I would add some sort of timeout function that tests how long I"m at a specific location and would trigger a 180 degree turn if I were near an x/y rover position for longer than like 30 seconds. Past that my fidelity wasn't super great, I think this could have been improved by both better "wall crawling" as well as 
accounting for the fact that perspective_transform will break if roll or pitch is a higher value. I'd probably add some kind of safety there to make it so it only mapped for reasonable roll/pitch values.  


