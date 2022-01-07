import os
import cv2

def generate_negative_description_file():
    with open('neg.txt', 'w') as f:
        for filename in os.listdir('./src/data/negatifs/'):
            f.write('negatifs/' + filename + '\n')

