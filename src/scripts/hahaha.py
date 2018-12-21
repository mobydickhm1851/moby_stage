import numpy as np

def this(arr):
    arr[1]=[5,6]
    return arr

test =  np.array([[1,2],[3,4]])
print("test before called 1 : {0}".format(test))
lol = this(test)
print("test after called 1 : {0}".format(test))
haha = this(test)
print("test after called 2 : {0}".format(test))

print(test, lol, haha)