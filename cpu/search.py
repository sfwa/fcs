import sys
import numpy
import subprocess
import skimage.feature
import skimage.color
import skimage.exposure
import skimage.io
import skimage.draw


def array_from_pgm(data, byteorder='>'):
    header, _, image = data.partition("\n")
    width, height, maxval = [int(item) for item in header.split()[1:]]
    return numpy.fromstring(
                image,
                dtype='u1' if maxval < 256 else byteorder + 'u2'
            ).reshape((height, width, 3))


def detect(pgm_path, threshold=0.2, min_size=1, max_size=5):
    with open(pgm_path, "r") as img_in:
        # Debayer the image
        debayer = subprocess.Popen(
            ["./debayer", "GBRG"], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True)
        img_out, stderr = debayer.communicate(img_in.read())

        # Convert the output to a numpy array
        img_arr = skimage.img_as_float(array_from_pgm(img_out))

        # Crop the useless bits of the frame, convert to LAB, and extract the
        # L channel as a float in [0.0, 1.0]
        img_lab = skimage.img_as_float(
            skimage.color.rgb2lab(img_arr)[..., 0] * 0.01)
        img_hsv = skimage.img_as_float(
            skimage.color.rgb2hsv(img_arr)[..., 0])
        img_hsv2 = skimage.img_as_float(
            skimage.color.rgb2hsv(img_arr)[..., 0])

        # Stretch the contrast so that the 0.2th percentile brightness becomes
        # 0.0, and the 99.8th percentile brightness becomes 1.0.
        pmin, pmax = numpy.percentile(img_lab, (90.0, 99.8))
        img_exp = skimage.exposure.rescale_intensity(img_lab,
                                                     in_range=(pmin, pmax))

        # Detect blue blobs
        img_hsv = numpy.mod(1.0 + img_hsv - 0.65, 1.0)
        img_hsv = numpy.abs(img_hsv - 1.0)
        img_hsv[img_hsv > 0.5] = 0.5
        img_hsv = 1.0 - img_hsv * 2.0
        img_hsv *= img_exp

        out_path = pgm_path.rpartition(".")[0] + "-h-blue.png"
        skimage.io.imsave(out_path, img_hsv)

        blue_blobs = skimage.feature.blob_dog(img_hsv, min_sigma=min_size,
                                              max_sigma=max_size,
                                              sigma_ratio=1.8,
                                              threshold=threshold)

        # Detect yellow blobs
        img_hsv2 = numpy.mod(1.0 + img_hsv2 - 0.15, 1.0)
        img_hsv2 = numpy.abs(img_hsv2 - 1.0)
        img_hsv2[img_hsv2 > 0.5] = 0.5
        img_hsv2 = 1.0 - img_hsv2 * 2.0
        img_hsv2 *= img_exp

        out_path = pgm_path.rpartition(".")[0] + "-h-yellow.png"
        skimage.io.imsave(out_path, img_hsv2)

        yellow_blobs = skimage.feature.blob_dog(img_hsv2, min_sigma=min_size,
                                                max_sigma=max_size,
                                                sigma_ratio=1.8,
                                                threshold=threshold)
        # Make the image a bit darker so the highlight stands out more
        img_arr *= 0.7

        h, w, d = img_arr.shape

        target_blobs = []
        for b1 in yellow_blobs:
            for b2 in blue_blobs:
                if abs(b1[0] - b2[0]) < 8 and abs(b1[1] - b2[1]) < 8 and abs(b1[2] - b2[2]) < 4:
                    target_blobs.append((int((b1[0] + b2[0]) * 0.5), int((b1[1] + b2[1]) * 0.5), b1[2]))

        # Draw a red line around the features by subtracting a circle
        # pattern from the G and B channels, and adding it to R.
        for blob in target_blobs:
            if blob[1] < 10 or blob[1] > w - 10 or blob[0] < 10 or blob[0] > h - 10:
                continue

            rr, cc, val = skimage.draw.circle_perimeter_aa(
                blob[0], blob[1], blob[2] + 4)
            img_arr[rr, cc, 0] = val

        print "%s: %d Y blobs, %d B blobs, %d targets" % (pgm_path, len(yellow_blobs), len(blue_blobs), len(target_blobs))

        # If blobs were found, write an image out showing where
        if len(target_blobs):
            out_path = pgm_path.rpartition(".")[0] + "-blobs.png"
            skimage.io.imsave(out_path, img_arr)


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        try:
            detect(arg)
        except Exception:
            print "Couldn't process %s" % arg
