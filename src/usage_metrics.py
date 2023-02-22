import sys
import numpy as np
import os

files = [x for x in os.listdir('.') if x.__contains__('res_')]

csv_f = open(sys.argv[1],'r')

pres = []
posts = []

for f in files:
    pre_start = []
    post = []
    csv_f = open(f,'r')
    for i in csv_f.readlines():
        u = i[:-1]
        t = u.split(',')
        if(t[0]=='_'):
            break
        pre_start.append(float(t[2]))

    csv_f = open(f,'r')
    for j in csv_f.readlines():
        j = j[:-1]
        t = j.split(',')
        if(t[0]=='_'):
            continue
        else:
            post.append(float(t[2]))


    pre_start = np.array(pre_start)
    pres.append(np.mean(pre_start))
    post = np.array(post)
    posts.append(np.mean(post))

print(pres,posts)
print(np.mean(np.array(pres)))
print(np.mean(np.array(posts)))

