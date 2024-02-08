Here is how I suggest you proceed:

* Create a directory called `data`. Create sub-directories `calibration_images` and `calibration_results` inside `data`.
* Run the example notebook ["How to print tags and create templates"](https://github.com/tbretl/ae598-3dv/blob/cbd382c001f3f2491709962e7fd93cbfd6548bff/examples/01_tags_and_templates/tags_and_templates.ipynb). Print the image `tag36_11_grid_5x8-image.png` that is produced by this notebook, being careful to print it at 100% scale. Move the file `tag36_11_grid_5x8-template.json` that is produced by this notebook to the `data` directory.
* Rigidly attach your printout of the tag grid to a flat surface. Capture four images of this tag grid from different positions and orientations with some camera (e.g., your phone) to which you will have access throughout the rest of the semester. Convert these images to `PNG` format and put them in the `calibration_images` directory.
* Run both the "Set up notebook" and "Parse images" sections of your calibration notebook.
* Complete `FIXME (1)` and `FIXME (2)` and run the "Estimate intrinsic and extrinsic parameters by inspection" part of your calibration notebook.
* Complete `FIXME (3)` and run both the "Estimate intrinsic and extrinsic parameters by optimization" and "Visualize results" sections of your calibration notebook. (Did the optimizer run successfully? Did the distribution of projection errors improve — i.e., "get smaller" — after optimization? Do the images in `calibration_results` show a match between the predicted (blue) and actual (red) corners of tags?)
* Complete `FIXME (4)`, `FIXME (5)`, and `FIXME (6)`. Run the "Estimate intrinsic and extrinsic parameters by analysis" part of your calibration notebook. Then, re-run the "Estimate... by optimzation" and "Visualize results" sections of your calibration notebook. (Did the optimizer run successfully? Did the distribution of projection errors improve — i.e., "get smaller" — after optimization? Do the images in `calibration_results` show a match between the predicted (blue) and actual (red) corners of tags?) The section "Estimate... by inspection" is, of course, made obsolete by the section "Estimate... by analysis" and can be removed after you finish your implementation.

As a next step, I suggest you apply your code (unchanged) to a larger set of images. Rather than capture these images one by one, it may be easier to capture a single video and to modify your code to grab frames from this video rather than to load images from `calibration_images`. Keep in mind that most phones will use a different camera (or at least different camera settings) when capturing video than when capturing images, so your intrinsic parameter estimates may change if you switch from images to a video.