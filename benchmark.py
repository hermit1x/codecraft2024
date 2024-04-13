import os
import json
import subprocess
import numpy as np
from tqdm import tqdm
import threading

# os.chdir('./bin')
# os.system('cmake ..')
# os.system('make .')
# os.chdir('../mac')

os.chdir('./MacOSRelease')

iters = 10

s = [[], [], [], []]
mutex = threading.Lock()

def fun(i):
    result = subprocess.run(
        [f'./SemiFinalJudge ../bin/main -m ./maps/map{i}.txt -l NONE 2> /dev/null'],
        shell=True,
        capture_output=True
    )
    res = result.stdout.decode()
    score = json.loads(res)['score']
    mutex.acquire()
    s[i].append(score)
    mutex.release()

for i in tqdm(range(iters//2), desc="iter"):
    t1 = threading.Thread(target=fun, args=[1])
    t1.start()
    t2 = threading.Thread(target=fun, args=[2])
    t2.start()
    t4 = threading.Thread(target=fun, args=[1])
    t4.start()
    t5 = threading.Thread(target=fun, args=[2])
    t5.start()

    t1.join()
    t2.join()
    t4.join()
    t5.join()


for i in range(1, 3):
    s[i] = np.array(s[i])
    print(f"map{i}: ave:{s[i].mean()} max:{s[i].max()} min:{s[i].min()}")

print(f'sum: {s[1].mean() + s[2].mean() + s[3].mean()}')
print(f'best: {s[1].max() + s[2].max() + s[3].max()}')