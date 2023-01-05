+++  
title = "Week 6 Object Detection Node"  
hidden = "true"
+++

#### The Code {#code}

Copy **all** the code below into your `object_detection.py` file.  Then, review [the explainer below](#explainer) to understand how this all works.

{{< include file="object_detection.py" code="true" lang="python" >}}

#### The Code Explained {#explainer}

Before we do anything else (do I really need to say it by now?!)... [DFTS](../../week1/subscriber/#dfts)!

1. First, let's have a look at the Python libraries that we are importing for this node:

    ```python
    import rospy
    from pathlib import Path

    import cv2
    from cv_bridge import CvBridge, CvBridgeError

    from sensor_msgs.msg import Image
    ```

    Of course, we always need to import `rospy` so that Python can work with ROS. What we're also importing here is the Python `Path` class from [the `pathlib` module](https://docs.python.org/3/library/pathlib.html), which will be used to do a few file operations. Then, we're importing the OpenCV library for Python (remember the Python API [that we talked about earlier](../week6/#opencv)), which is called `cv2`, and *also* that ROS-to-OpenCV bridge interface that we talked about earlier too: `cv_bridge`.  From `cv_bridge` we're importing the `CvBridge` and `CvBridgeError` classes from the `cv_bridge` library specifically.

    Finally, we need to subscribe to an image topic in order to obtain the messages being published to it. You should've already identified the *type* of message that is published to the `/camera/rgb/image_raw` topic, so we import that message type here (from the `sensor_msgs` package) so that we can build a subscriber to the topic later.

1. Next, we're doing a number of initialisations that should be very familiar to you by now:
    1. Giving our node a name.
    1. Initialising the node (i.e. registering it on the ROS network using `rospy.init_node()`).
    1. Specifying a rate at which we want the node to run:

        ```python
        node_name = "object_detection_node"
        rospy.init_node(node_name)
        print(f"Launched the '{node_name}' node. Currently waiting for an image...")
        rate = rospy.Rate(5)
        ```

1. Then, we define a filesystem location that we'll use to save images to. We know that there's a directory in the home directory of the WSL-ROS filesystem called "myrosdata", so we can use Pathlib's `Path.home().joinpath(...)` to define it (without necessary needing to know the name of the home directory itself). Then, we use the Pathlib `Path.mkdir()` method to create this directory, if it doesn't exist already.

    ```python
    base_image_path = Path.home().joinpath("myrosdata/week6_images/")
    base_image_path.mkdir(parents=True, exist_ok=True)
    ```

1. Next, we create an instance of the `CvBridge` class that we imported earlier, and which we'll use later on to convert ROS image data into a format that OpenCV can understand.  We're then also creating a flag to indicate whether the node has obtained an image yet or not. For this exercise, we only want to obtain a single image, so we will set the `waiting_for_image` flag to `False` in our camera callback function once an image has been obtained, to avoid capturing any more:

    ```python
    cvbridge_interface = CvBridge()

    waiting_for_image = True
    ```

1. After that, we're creating a function called `show_and_save_image()`, within which we contain some image operations that we will need to repeat multiple times (this will become apparent later in this exercise when you start adding further functionality to the camera callback function):

    ```python
    def show_and_save_image(img, img_name):
        ...
    ```

1. We'll come back to this in a minute, but for now let's skip past this function and on to the next one:

    ```python
    def camera_cb(img_data):
        ...
    ```

    Here, we're defining a callback function for a `rospy.Subscriber()`.

1. Let's skip past this function for now as well, and first keep moving down the code to where this subscriber is actually defined:

    ```python
    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)

    while waiting_for_image:
        rate.sleep()

    cv2.destroyAllWindows()
    ```

    This is in fact the end of the code, but establishes the main behaviour that the `object_detection.py` node will enact:
    1. As discussed above, we create subscriber to the `/camera/rgb/image_raw` topic first, telling the `rospy.Subscriber()` function the message *type* that is used by this topic (`sensor_msgs/Image` - as imported above), and we point it to a callback function (`camera_cb`, in this case), to define the processes that should be performed every time a message is obtained on this topic (in this case, the messages will be our camera images)
    1. Then we go into a `while` loop, and use the `rate.sleep()` method to maintain this loop at a speed of 5 Hz (as defined earlier) whilst checking the `waiting_for_image` flag to see if an image has been obtained by our subscriber yet.  We only really want to obtain a single image here, so once the `waiting_for_image` flag changes to `False`, the `while` loop will stop.
    1. Finally, before terminating the node, we're running the `cv2.destroyAllWindows()` method to make sure that any OpenCV image pop-up windows that may still be active or in memory are destroyed before the node shuts down. 

1. Now let's go back to the camera callback function, and see what we are telling the node to do every time an image message is received:

    ```python
    def camera_cb(img_data):
        global waiting_for_image  
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        if waiting_for_image == True:
            height, width, channels = cv_img.shape

            print(f"Obtained an image of height {height}px and width {width}px.")

            show_and_save_image(cv_img, img_name = "step1_original")

            waiting_for_image = False
    ```

    Let's talk through this step-by-step:
    1. First we want to make changes to the `waiting_for_image` flag *inside* this function, but make sure that these changes are also observed *outside* the function too (i.e. by the `while` loop that we talked about above).  So, we change the *scope* of the variable to *global* here using the `global` statement.
    1. Next we're using the CvBridge interface to take our ROS image data and convert it to a format that OpenCV will be able to understand.  In this case we are specifying conversion (or *"encoding"*) to an 8-bit BGR (Blue-Green-Red) image format: `"bgr8"`.
        
        We contain this within a `try-except` block though, which is the [recommended procedure when doing this](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython).  Here we *try* to convert an image using the desired encoding, and if a `CvBridgeError` is raised then we print this error to the terminal.  Should this happen, this particular execution of the camera callback function will stop.
    1. Then we check the `waiting_for_image` flag to see if this is the first image that has been received by the node.  If so, then we:
        1. Obtain the height and width of the image (in pixels), as well as the number of colour channels,
        1. Print the image dimensions to the terminal,
        1. Pass the image data to the `show_and_save_image()` function, which we mentioned earlier and which is explained in more detail below.  Here we also pass a descriptive name for the image to this function too (`img_name`).
        1. Finally, we set the `waiting_for_image` flag to `False` so that we only ever perform these processing steps once (we only want to capture one image remember!).  This will then trigger the main `while` loop to stop, thus causing the overall execution of the node to stop too.

1. Finally, let's talk about the `show_and_save_image()` function, and see what is happening inside this:

    ```python
    def show_and_save_image(img, img_name):
        full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

        print("Opening the image in a new window...")
        cv2.imshow(img_name, img)
        print(f"Saving the image to '{full_image_path}'...")
        cv2.imwrite(str(full_image_path), img)
        print(f"Saved an image to '{full_image_path}'\n"
            f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
            f"file size = {full_image_path.stat().st_size} bytes")
        print("Please close down the image pop-up window to continue...")
        cv2.waitKey(0)
    ```

    Essentially, what we are doing here is:
    1. Constructing a full file path for an image (using the `Path.joinpath()` method) from:
        1. The `base_image_path` that we defined earlier and 
        1. An image name that is passed into this function via the `img_name` argument.

        We'll use this to save the file to our filesystem later on.
    1. We then use the `cv2.imshow()` function to display the actual image in a pop-up window:
        1. The image data is passed into the function via the `img` argument,
        1. We need to give the pop-up window a name, so in this case we are using the `img_name` argument that has also been passed into the function.
    1. Next, we are using the `cv2.imwrite()` function to save the image to a `.jpg` file.  We're supplying the `full_image_path` that was created above, and also the actual image data (`img`) so that the function knows what image we want to save.
    1. Then, we're printing a message to the terminal to inform us of (a) where the image has been saved to, (b) how big the image was (in terms of its pixel dimensions) and (c) how big the image *file* is (in bytes).
    1. And to finish off, we're using the `cv2.waitKey()` function.  We're supplying a value of `0` here, which tells this function to wait indefinitely before allowing our `show_and_save_image()` function to end. If we had supplied a value here (say: `1`) then the function would simply wait 1 millisecond and then close the pop-up window down. In our case however, we want some time to actually look at the image and then close the window down ourselves, manually. Once the window has been closed, the execution of our code is able to continue...
