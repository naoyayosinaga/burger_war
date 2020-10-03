import learn_image_maker as lim
import cv2
from PIL import Image

if __name__=='__main__':

    path = 'log.txt'

    f = open(path)
    line_count = 0

    for line in f:

        i = 0

        if line[0] =='(':

            while(line[i] != ','):
                x_end = i
                i+=1
            x_end+=1
            i+=1

            while(line[i] != ','):
                y_end = i
                i+=1
            y_end += 1
            x=''
            y=''
            for j in range(1,x_end):
                x = x+line[j]

            for j in range(x_end+1,y_end):
                y = y+line[j]

            theta = int(line[y_end+2])
            line_count+=1

            map = lim.learn_image_maker(int(x),int(y),theta,23,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
            cv2.imwrite('tempim.png',map)
            
            if line_count==1:
                im = Image.open('tempim.png')
                im_list = []
            else:
                im_temp = Image.open('tempim.png')
                im_list.append(im_temp)

    im.save('simu.gif',save_all=True,append_images=im_list,duaration=500,loop=100)
