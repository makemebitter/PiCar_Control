import imageio
import glob, os
from wand.image import Image
filenames=[file for file in glob.glob("*.pdf")]
filenames.sort()
filenames.sort(key=len, reverse=False)



# Converting first page into JPG

for filename in filnames:
	with Image(filename=filename) as img:
	     img.save(filename=filename+".jpg")
	# Resizing this image
	# with Image(filename=filename+".jpg") as img:
	#      img.resize(200, 150)
	#      img.save(filename="/thumbnail_resize.jpg")
	    
images = []

for filename in filenames:
	print filename
    images.append(imageio.imread('./'+filename))
imageio.mimsave('../ani.gif', images)