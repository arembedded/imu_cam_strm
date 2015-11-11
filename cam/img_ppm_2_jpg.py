from PIL import Image
import sys

name = sys.argv[1]
im = Image.open(name + '.ppm')
im.save(name + ".jpg")
