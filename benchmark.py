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

os.chdir('./mac')

iters = 10

s = [[], [], [], []]
mutex = threading.Lock()

def fun(i):
    result = subprocess.run(
        [f'./SemiFinalJudge ../bin/main -m ./maps/map{i}.txt -l NONE'],
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
    t3 = threading.Thread(target=fun, args=[3])
    t3.start()
    t4 = threading.Thread(target=fun, args=[1])
    t4.start()
    t5 = threading.Thread(target=fun, args=[2])
    t5.start()
    t6 = threading.Thread(target=fun, args=[3])
    t6.start()

    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
    t6.join()


for i in range(1, 4):
    s[i] = np.array(s[i])
    print(f"map{i}: ave:{s[i].mean()} max:{s[i].max()} min:{s[i].min()}")

print(f'sum: {s[1].mean() + s[2].mean() + s[3].mean()}')