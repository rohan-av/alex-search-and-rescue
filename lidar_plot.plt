set size square 1,1     				#set the aspect ratio to 1:1
plot [0:320] [0:320] "lidar_reading.dat" using 1:2 with points pointtype 4
