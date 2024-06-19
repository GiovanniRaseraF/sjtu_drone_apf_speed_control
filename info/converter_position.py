import math as Math

#gazebo world
gmaxx = 9.42472
gmaxy = 9.27499
gminx = -9.26194
gminy = -9.39923
#python world
pmaxx = 50
pmaxy = 50
pminx = 0
pminy = 0

def gazebo_to_python(gx, gy):
    
    px = Math.floor(((gx - gminx) * (pmaxx - pminx) / (gmaxx - gminx)) + pminx)
    py = Math.floor(((gy - gminy) * (pmaxy - pminy) / (gmaxy - gminy)) + pminy)

    return px, py

def python_to_gazebo(px, py):
    
    gx = Math.floor(((px - pminx) * (gmaxx - gminx) / (pmaxx - pminx)) + gminx)
    gy = Math.floor(((py - pminy) * (gmaxy - gminy) / (pmaxy - pminy)) + gminy)

    return gx, gy
