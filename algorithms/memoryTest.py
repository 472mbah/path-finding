# importing the module
import tracemalloc
 
# code or function for which memory
# has to be monitored
def app():
    lt = []
    # j = [0] * 10000
    c = [1] * 2
    j = set()
    for i in range(0, 100000):
        lt.append(i)
 
# starting the monitoring
tracemalloc.start()
tracemalloc.take_snapshot()
# function call
app()
 
# displaying the memory
print(tracemalloc.get_traced_memory())
 
# stopping the library
tracemalloc.stop()