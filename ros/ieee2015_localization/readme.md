IEEE2015 Localization
=====================

Principal Author: *Matt Feldman*
Add'l Authors: Jacob Panikulam, Brandon Peterson
All work contained in this folder, regardless of the git blame, was based on the work of the esteemed and lord-like Matthew Feldman.

# Method
Boltzman Affine or Linear Logical Sample Consensus (BALLSAC), a development on RANdom SAmple Consensus (RANSAC)



## Pipeline

Image --> Perspective Transform --> Feature detection/Correspondence Estimation --> Output transform

## Design

1. Take in a new image
2. If the translation from the previous image is greater than something (close) to zero, use this as the new root image
3. Use those translations to change root position

Run at ~10Hz or optimize


# TO-TRY
Transforming endpoints of a hough-transform instead of feature points (which are non-descriptive for black/white images)
Least-squared-difference or a similar approach
Convolutional template matching?

# TODO

Do comparison only to the previous image, add current scale tracking

Move things out of pixel coordinates, into metric coords

Add a "match-quality" metric for testing output

Use match-quality to assess certainty. If too low (or we think we've moved very far), start scanning the image

Keep a history of images (just a few) incase everything goes wrong, and we fail to get a good match. (Keyframe chain)

Track transforms only along the keyframe tree

Be able to jump along the key-frame tree

Dismiss sudden massive changes


Add a perspective transform based on angle of camera, without requiring someone to input 4 points. This should not be hard to do, but will require some serious math

Make fourier registration work in real-time: 
    - Remove creation of the 2nd image; don't do redundant fft/ifft's
    - Speed up logpolar estimation (linspace, ndii.map_coordinates)

Benchmarks:
    (150 x 150) -> 0.1 sec
    (250 x 250) -> 0.29 sec

Where (Y x Y) is the image size


# ISSUES

- We stop generating new seed images if we lose track of the map
- Doesn't handle scale