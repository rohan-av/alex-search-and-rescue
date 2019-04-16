set size square 1,1     				#set the aspect ratio to 1:1
set object circle at 160,160 size 80  	#draw 3 concentric circle centered on 160,160
set object circle at 160,160 size 50 
set object circle at 160,160 size 20 
set label "53.3 cm" at 150,130 boxed		#label the 3 circles
set label "133.3 cm" at 150,100 boxed		#find out the actual distance
set label "233.3 cm" at 150,70 boxed
plot [0:320] [0:320] "lidar_reading.dat" using 1:2 with points pointtype 4
pause 1
reread
