IEEE2015 Localization
=====================

Principal Author: *Matt Feldman*
All work contained in this folder, regardless of the git blame, was based on the work of the esteemed and lord-like Matthew Feldman.

# Method
Boltzman Affine or Linear Logical Sample Consensus (BALLSAC), a development on RANdom SAmple Consensus (RANSAC)



# Pipeline

Image --> Perspective Transform --> Feature detection/Correspondence Estimation --> Output transform

# TO-TRY
Transforming endpoints of a hough-transform instead of feature points (which are non-descriptive for black/white images)
Least-squared-difference or a similar approach
Convolutional template matching?

# TODO

Add a perspective transform based on angle of camera, without requiring someone to input 4 points. This should not be hard to do, but will require some serious math