import os
import json
import subprocess
import numpy as np
from tqdm import tqdm

os.chdir('./MacOSReleasev2.0')
os.system('g++ ../main.cpp -o ./main')

iters = 20

s1 = []
for i in tqdm(range(iters), desc="Map1"):
    result = subprocess.run(
                ['./PreliminaryJudge ./main -m ./maps/map1.txt -l NONE'], 
                shell=True, 
                capture_output=True
            )
    res = result.stdout.decode()
    score = json.loads(res)['score']
    s1.append(score)

s2 = []
for i in tqdm(range(iters), desc="Map2"):
    result = subprocess.run(
                ['./PreliminaryJudge ./main -m ./maps/map2.txt -l NONE'], 
                shell=True, 
                capture_output=True
            )
    res = result.stdout.decode()
    score = json.loads(res)['score']
    s2.append(score)

s1 = np.array(s1)
s2 = np.array(s2)

print("map1: ave:", s1.mean(), "max:", s1.max())
print("map2: ave:", s2.mean(), "max:", s2.max())