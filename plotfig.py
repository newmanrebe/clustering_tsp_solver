import numpy as np
import matplotlib.pyplot as plt
import random
import csv

def get_random_color(pastel_factor = 0.5):
    return [(x+pastel_factor)/(1.0+pastel_factor) for x in [random.uniform(0,1.0)]]

def color_distance(c1,c2):
    return sum([abs(x[0]-x[1]) for x in zip(c1,c2)])

def generate_new_color(existing_colors,pastel_factor = 0.5):
    max_distance = None
    best_color = None
    for i in range(0,100):
        color = get_random_color(pastel_factor = pastel_factor)
        if not existing_colors:
            return color
        best_distance = min([color_distance(color,c) for c in existing_colors])
        if not max_distance or best_distance > max_distance:
            max_distance = best_distance
            best_color = color
    return best_color

def pngkmeans(xvals,yvals,labels,filename):
    fig = plt.figure(figsize=(7, 7))
    #print(len(coords))
    num_clusters  = max(labels)
    colors = []
    for i in range(0,num_clusters+1):
        colors.append(generate_new_color(colors,pastel_factor=0.5))
    print(colors)
    plt.scatter(xvals,yvals, c=[colors[x][0] for x in labels],s=80)
    save_string = filename + '_kmeans_'+str(num_clusters)+'.png'
    fig.savefig(save_string)


xvals =[]
yvals =[]
labels =[]
with open('/home/rebecca/Downloads/Telegram Desktop/clusters/k_means_a280_2.csv') as f:
    reader = csv.reader(f)
    f.readline()
    for row in reader:
        xvals.append(float(row[1]))
        yvals.append(float(row[2]))
        labels.append(int(row[3]))
#coords = [[1,2],[3,4],[5,6],[7,8]]
print(len(xvals))
print(len(labels))
pngkmeans(xvals,yvals,labels,'test')
