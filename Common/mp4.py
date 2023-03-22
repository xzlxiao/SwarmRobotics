import imageio
import cv2

resize = 1
image_name = 'move_performance5'
image_num = 345
fps = 20

img_paths = []
for i in range(1, image_num, 1):
    img_paths.append('/Users/xzlxiao/Downloads/tmp/{}/image{:0>5d}.png'.format(image_name, i))

image = cv2.imread('/Users/xzlxiao/Downloads/tmp/{}/image00001.png'.format(image_name))
video_out = cv2.VideoWriter("/Users/xzlxiao/Downloads/tmp/{}.avi".format(image_name), cv2.VideoWriter_fourcc(*'XVID'), fps, (int(image.shape[1]*resize), int(image.shape[0]*resize)))

for path in img_paths:
    print(path)
    image = cv2.imread(path)
    image = cv2.resize(image, (int(image.shape[1]*resize), int(image.shape[0]*resize)))
    video_out.write(image)

video_out.release()