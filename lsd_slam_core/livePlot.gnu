set terminal x11 noraise
set view equal xyz

sp '< tail -2000 slamPath.txt' u 1:2:3 w l
pause 0.2
reread
