import os
import json
import subprocess
import numpy as np
from tqdm import tqdm

os.chdir('./bin')
os.system('cmake ..')
os.system('make .')
os.chdir('../mac')
iters = 1

s1 = []
for i in tqdm(range(iters), desc="Map1"):
    result = subprocess.run(
                ['./SemiFinalJudge ../bin/main -m ./maps/map1.txt -l NONE'],
                shell=True, 
                capture_output=True
            )
    res = result.stdout.decode()
    score = json.loads(res)['score']
    s1.append(score)

s2 = []
for i in tqdm(range(iters), desc="Map2"):
    result = subprocess.run(
                ['./SemiFinalJudge ../bin/main -m ./maps/map2.txt -l NONE'],
                shell=True, 
                capture_output=True
            )
    res = result.stdout.decode()
    score = json.loads(res)['score']
    s2.append(score)

s3 = []
for i in tqdm(range(iters), desc="Map3"):
    result = subprocess.run(
        ['./SemiFinalJudge ../bin/main -m ./maps/map3.txt -l NONE'],
        shell=True,
        capture_output=True
    )
    res = result.stdout.decode()
    score = json.loads(res)['score']
    s3.append(score)

s1 = np.array(s1)
s2 = np.array(s2)
s3 = np.array(s3)

print("map1: ave:", s1.mean(), "max:", s1.max())
print("map2: ave:", s2.mean(), "max:", s2.max())
print("map3: ave:", s3.mean(), "max:", s3.max())
print(f'sum: {s1.mean() + s2.mean() + s3.mean()}')