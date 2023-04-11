# OpenCV-Trackers_Eval
Three separate Computer Vision Trackers developed using OpenCV 4.5.2 are run and evaluated together.

Input Source can be a video or image frames. The choice of Source is selectable in the Command Console upon code execution.

## 1. MOSSE TRACKER
### Based on: 
Bolme, David. Beveridge, J. Drapper, Bruce and Lui Yui. Visual Object Tracking using Adaptive Correlatio Filters. CVPR, 2010. 

Code used is an edited variation of the one from https://github.com/albertoQD/tracking-mosse

## 2. OpenCV Template Matching (NCC) TRACKER
### Based on: 
OpenCV's built-in "Template Matching Library". The Similiarity Index used here is Normalized Cross Correlation(NCC).

Algorithm for reliable Occlusion Handling has been introduced as well as blending of the present template with its previous copies.

## 3. STAPLE TRACKER
### Based on: 
CVPR16 paper "Staple: Complementary Learners for Real-Time Tracking" by Bertinetto et al

Code used is a heavily edited variation of the one from https://github.com/xuduo35/STAPLE. Algorithm based on Peak-to-Sidelobe Ratio (PSR) for Occlusion Handling has been introduced.

![Untitled](https://user-images.githubusercontent.com/124782488/231141595-1af443ea-ce52-42cc-a14d-ab307cd70ded.png)

The Three Tracker are titled 'M/N/S' as can be seen in the above image. 'M_O/N_O/S_O' with a white annotated rectangle shows that the tracker is in a state of occlusion.  
