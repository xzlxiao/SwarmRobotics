import imageio
import cv2

resize = 0.5
image_name = 'DiffusionWithSource'
image_num = 171

img_paths = []
for i in range(image_num):
    img_paths.append('/Users/xzlxiao/Downloads/tmp/{}/{:0>4d}.png'.format(image_name, i))


gif_images = []

for path in img_paths:
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gif_images.append(cv2.resize(image, (int(image.shape[1]*resize), int(image.shape[0]*resize))))
imageio.mimsave("/Users/xzlxiao/Downloads/tmp/{}.gif".format(image_name),gif_images, 'GIF',duration=0.1)