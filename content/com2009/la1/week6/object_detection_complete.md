+++  
title = "Week 6 Object Detection Node: Complete"  
hidden = "true"
+++

Here's a full example of the `object_detection.py` node that you should have developed during [The Object Detection Exercise](../week6/#ex2).  Also included here is an illustration of how to use the `cv2.circle()` method to create a marker on an image illustrating the centroid of the detected feature, as discussed [here](../week6/#image-moments).

#### The Code {#code}

{{< include file="object_detection_complete.py" code="true" lang="python" >}}

#### The Code Explained {#explainer}

Everything here should be familiar to you from [the previous exercise](../week6/#ex2), except for this new bit:

```python
m = cv2.moments(img_mask)
cy = m['m10'] / (m['m00'] + 1e-5)
cz = m['m01'] / (m['m00'] + 1e-5)
cv2.circle(filtered_img, (int(cy), int(cz)), 10, (0, 0, 255), 2)
```

Here, we are obtaining the moments of our colour blob by providing the boolean representation of it (i.e. the `img_mask`) to the `cv2.moments()` function.

Then, we are determining *where* the central point of this colour blob is located by calculating the `cy` and `cz` coordinates of it.  This provides us with pixel coordinates relative to the top left-hand corner of the image.

Finally, We can use the `cv2.circle()` function to draw a circle on our image at the centroid location so that we can visualise it.  Into this function we pass:
1. The image that we want the circle to be drawn on.  In this case: `filtered_img`.
1. The *location* that we want the circle to be placed, specifying the horizontal and vertical pixel coordinates respectively: `(int(cy), int(cz))`.
1. How *big* we want the circle to be: here we specify a radius of 10 pixels.
1. The *colour* of the circle, specifying this using a Blue-Green-Red colour space: `(0, 0, 255)` (i.e.: pure red in this case)
1. Finally, the thickness of the line that will be used to draw the circle, in pixels.