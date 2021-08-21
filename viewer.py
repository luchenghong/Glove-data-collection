import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt("icm_test.txt",delimiter=',',skip_header=2)



end = -1
sec = data[:end,1]
usec = data[:end,2]

hist = []
s = sec[0]
counter  = 0
for ss in sec:
    if ss == s:
        counter += 1
    else:
        hist.append(counter)
        counter = 1
        s = ss

hist = np.array(hist)
timestamp = sec+(usec*10e-7) 

time_diff = np.diff(timestamp)
print(np.argmax(time_diff))
fig = plt.figure(figsize=(6,4))
plt.plot(range(timestamp.shape[0]),timestamp)
plt.title("timestamp")

fig = plt.figure(figsize=(6,4))
plt.plot(range(time_diff.shape[0]),time_diff)
plt.title("time_diff")

fig = plt.figure(figsize=(6,4))
plt.plot(range(hist.shape[0]),hist)
plt.title("hist")

plt.show()