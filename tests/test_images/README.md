# Image fixtures

Automated smoke tests generate synthetic grayscale/RGB arrays and native tests
use synthetic detection records. No labeled Fire/non-Fire photographs were
supplied, so recognition accuracy and comparison to Edge Impulse Studio are not
asserted.

Place representative JPEG/PNG files here if desired, including Fire, no Fire
and multiple Fire instances. Use `test_single_image(path)` and
`test_intermitent_images(files)` to exercise them. Record Studio reference
detections with matching preprocessing and compare the selected
highest-confidence detection. Commit only images you intend to share publicly.
