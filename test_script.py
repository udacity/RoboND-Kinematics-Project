from sympy import *
import time

start_time = time.time()
x = Matrix([[1,2,3],[4,5,6],[7,8,9]])
print(x)
p = Matrix([[1],[2],[3]])
y = Matrix([[0],[0],[1]])
#mult_start = time.time()
wc = p - 0.303*x*y
lc = p - 0.303*x[:,2]
#mult_time = time.time()-mult_start
#print('matrix takes '+str(mult_time))
print(wc)
print(lc)
#yc = p - x*y
#print(yc)
#print(time.time()-start_time)
