import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import math


if False:
    lenghts = np.array([57.07961788358283, 18.969408090846706, 3.4743086909175087, 51.67796707585578, 29.121386522992545, 2.923892658296705, 22.817861220659516, 10.478694652825162, 10.432142050191208, 38.16608209995694, 12.887339138463414, 18.95583779367343])
    hor_error = np.array([29,175,13,48,54,32,117,65,203,210,140,0])
    vert_error = np.array([167,201,73,0,0,15,168,19,336,217,333,24])
    building_height = 14


    lengths_p_h = abs(np.array([216-1814, 670-1322, 873-1035, 216-1803, 290-1576, 870-1033, 437-1393, 775-1222, 827-1283, 4-1920, 661-1143, 598-1415]))
    lengths_p_v = abs(np.array([752-328, 206-854, 209-819, 228-747, 85-854, 23-868, 115-833, 879, 82-830, 46-869, 177-811, 916]))

    hori_error_m = np.multiply(np.divide(hor_error, lengths_p_h), lenghts)  # or the other way around?
    veri_error_m = np.divide(vert_error, lengths_p_v) * building_height

    total_error = (np.divide(hor_error, lengths_p_h)/2 + np.divide(vert_error, lengths_p_v)/2) * 100
    
    print(hori_error_m)
    print(veri_error_m)
    print(total_error)



if True:

    aovs = [20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176]
    perc = [15.670981357036576, 15.114707226701157, 15.05441177900746, 15.0417691851362, 14.800587394361402, 14.580800762445662, 14.40574946268976, 14.519532807531096, 14.708199208379122, 14.999951374638956, 15.255720773726743, 15.553307983311775, 15.97537611716767, 16.44315209040427, 16.941075787487723, 17.531387670553453, 18.0954418586558, 18.679918698396335, 19.328581014714032, 19.964600737160474, 20.643410777325023, 21.328055860814764, 21.844457195094673, 22.443521643148202, 22.962840499090706, 23.381018604063136, 23.774884028513913, 24.094838904178864, 24.365195911579644, 24.52954963190602, 24.672508193373336, 24.760033843251286, 24.72405107607924, 24.53149464634775, 24.26794518949303, 23.939237748840288, 23.53759226662258, 22.99396073015842, 22.5038170908419, 22.14885195522577]

    plt.figure(1, figsize=(10,7))
    plt.plot(aovs, perc, '-o', color='black')
    plt.axis([16, 180, 10, 30])
    plt.title("Validity with increasing AoV's")
    plt.xlabel('Camera horizontal angle of view [°]')
    plt.ylabel('Percentage of buildings with valid impression poses [%]')
    #plt.clf()

    print(min(perc))
    print(max(perc))
    cgo3 = 115
    flirduopror = 56
    hasselblad = 96
    zenmuseXT2 = 90
    zenmusex4s = 84
    zenmuseZ30 = 63.7

    plt.text(cgo3+2.5,22.3, 'CGO3+', bbox=dict(facecolor='b', alpha=0.2), verticalalignment='top')
    plt.scatter(cgo3, 22.8, s=100, zorder=10, color='b')

    plt.text(flirduopror-3,16,'Flir Duo Pro R', bbox=dict(facecolor='r', alpha=0.2), verticalalignment='top', horizontalalignment='right')
    plt.scatter(flirduopror, 15, s=100, zorder=10, color='r')

    plt.text(zenmuseXT2+4, 18.8,'DJI Zenmuse XT2', bbox=dict(facecolor='g', alpha=0.2), verticalalignment='top', horizontalalignment='left')
    plt.scatter(zenmuseXT2+3,  19.42, s=100, zorder=10, color='g')

    plt.text(hasselblad+3, 19.9, 'Hasselblad A6D', bbox=dict(facecolor='c', alpha=0.2), verticalalignment='top', horizontalalignment='left')
    plt.scatter(hasselblad, 19.91, s=100, zorder=10, color='c')

    plt.text(zenmusex4s+3, 17.6, 'DJI Zenmuse X4S', bbox=dict(facecolor='m', alpha=0.2), verticalalignment='top', horizontalalignment='left')
    plt.scatter(zenmusex4s, 18.05, s=100, zorder=10, color='m')

    plt.text(zenmuseZ30+3, 15.4, 'DJI Zenmuse Z30', bbox=dict(facecolor='tab:blue', alpha=0.2), verticalalignment='top', horizontalalignment='left')
    plt.scatter(zenmuseZ30, 15.54, s=100, zorder=10, color='tab:blue')

    plt.savefig("aovsandcameras.pdf")
    plt.show()

'''
Ways: 102827
Generating local coordinates from lat, lon to UTM then to origin |################################| 102827/102827
20.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 16114
Number of houses that are invalid: 86713
Percentage of houses that this approach can cover: 15.670981357036576
24.000000000000004
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15542
Number of houses that are invalid: 87285
Percentage of houses that this approach can cover: 15.114707226701157
28.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15480
Number of houses that are invalid: 87347
Percentage of houses that this approach can cover: 15.05441177900746
32.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15467
Number of houses that are invalid: 87360
Percentage of houses that this approach can cover: 15.0417691851362
36.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15219
Number of houses that are invalid: 87608
Percentage of houses that this approach can cover: 14.800587394361402
40.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 14993
Number of houses that are invalid: 87834
Percentage of houses that this approach can cover: 14.580800762445662
44.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 14813
Number of houses that are invalid: 88014
Percentage of houses that this approach can cover: 14.40574946268976
48.00000000000001
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 14930
Number of houses that are invalid: 87897
Percentage of houses that this approach can cover: 14.519532807531096
52.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15124
Number of houses that are invalid: 87703
Percentage of houses that this approach can cover: 14.708199208379122
56.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15424
Number of houses that are invalid: 87403
Percentage of houses that this approach can cover: 14.999951374638956
59.99999999999999
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15687
Number of houses that are invalid: 87140
Percentage of houses that this approach can cover: 15.255720773726743
64.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 15993
Number of houses that are invalid: 86834
Percentage of houses that this approach can cover: 15.553307983311775
68.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 16427
Number of houses that are invalid: 86400
Percentage of houses that this approach can cover: 15.97537611716767
72.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 16908
Number of houses that are invalid: 85919
Percentage of houses that this approach can cover: 16.44315209040427
76.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 17420
Number of houses that are invalid: 85407
Percentage of houses that this approach can cover: 16.941075787487723
80.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 18027
Number of houses that are invalid: 84800
Percentage of houses that this approach can cover: 17.531387670553453
84.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 18607
Number of houses that are invalid: 84220
Percentage of houses that this approach can cover: 18.0954418586558
88.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 19208
Number of houses that are invalid: 83619
Percentage of houses that this approach can cover: 18.679918698396335
92.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 19875
Number of houses that are invalid: 82952
Percentage of houses that this approach can cover: 19.328581014714032
96.00000000000001
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 20529
Number of houses that are invalid: 82298
Percentage of houses that this approach can cover: 19.964600737160474
100.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 21227
Number of houses that are invalid: 81600
Percentage of houses that this approach can cover: 20.643410777325023
104.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 21931
Number of houses that are invalid: 80896
Percentage of houses that this approach can cover: 21.328055860814764
108.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 22462
Number of houses that are invalid: 80365
Percentage of houses that this approach can cover: 21.844457195094673
112.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 23078
Number of houses that are invalid: 79749
Percentage of houses that this approach can cover: 22.443521643148202
116.00000000000001
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 23612
Number of houses that are invalid: 79215
Percentage of houses that this approach can cover: 22.962840499090706
119.99999999999999
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 24042
Number of houses that are invalid: 78785
Percentage of houses that this approach can cover: 23.381018604063136
124.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 24447
Number of houses that are invalid: 78380
Percentage of houses that this approach can cover: 23.774884028513913
128.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 24776
Number of houses that are invalid: 78051
Percentage of houses that this approach can cover: 24.094838904178864
132.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 25054
Number of houses that are invalid: 77773
Percentage of houses that this approach can cover: 24.365195911579644
136.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 25223
Number of houses that are invalid: 77604
Percentage of houses that this approach can cover: 24.52954963190602
140.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 25370
Number of houses that are invalid: 77457
Percentage of houses that this approach can cover: 24.672508193373336
144.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 25460
Number of houses that are invalid: 77367
Percentage of houses that this approach can cover: 24.760033843251286
148.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 25423
Number of houses that are invalid: 77404
Percentage of houses that this approach can cover: 24.72405107607924
152.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 25225
Number of houses that are invalid: 77602
Percentage of houses that this approach can cover: 24.53149464634775
156.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 24954
Number of houses that are invalid: 77873
Percentage of houses that this approach can cover: 24.26794518949303
160.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 24616
Number of houses that are invalid: 78211
Percentage of houses that this approach can cover: 23.939237748840288
164.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 24203
Number of houses that are invalid: 78624
Percentage of houses that this approach can cover: 23.53759226662258
168.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 23644
Number of houses that are invalid: 79183
Percentage of houses that this approach can cover: 22.99396073015842
172.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 23140
Number of houses that are invalid: 79687
Percentage of houses that this approach can cover: 22.5038170908419
176.0
Generating impression poses for current AoV |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Total number of buildings: 102827
Number of houses that are valid: 22775
Number of houses that are invalid: 80052
Percentage of houses that this approach can cover: 22.14885195522577
['15.670981357036576', '15.114707226701157', '15.05441177900746', '15.0417691851362', '14.800587394361402', '14.580800762445662', '14.40574946268976', '14.519532807531096', '14.708199208379122', '14.999951374638956', '15.255720773726743', '15.553307983311775', '15.97537611716767', '16.44315209040427', '16.941075787487723', '17.531387670553453', '18.0954418586558', '18.679918698396335', '19.328581014714032', '19.964600737160474', '20.643410777325023', '21.328055860814764', '21.844457195094673', '22.443521643148202', '22.962840499090706', '23.381018604063136', '23.774884028513913', '24.094838904178864', '24.365195911579644', '24.52954963190602', '24.672508193373336', '24.760033843251286', '24.72405107607924', '24.53149464634775', '24.26794518949303', '23.939237748840288', '23.53759226662258', '22.99396073015842', '22.5038170908419', '22.14885195522577']
'''