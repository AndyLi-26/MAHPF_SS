import os,random
from itertools import chain
N=900

def getStarts(h,w):
    return random.choices(list(chain(*[[(1,i),(h-2,i)] for i in range(1,w-1)]+[[(i,1),(i,w-2)] for i in range(1,h-1)])),k=N)

def getGoals(h,w,m):
    return random.sample([(i,j) for i in range(h) for j in range(w) if m[i][j]=="."],N)

r=lambda f: f.readline().split()
def read_map(fn):
    with open(fn) as f:
        _,h,w,_,m=r(f),int(r(f)[1]),int(r(f)[1]),r(f),[list(l.strip())for l in f]
    return h,w,m

root= os.getcwd()
for mf in [item for item in os.listdir(root) if os.path.isdir(os.path.join(root, item))]:
    h,w,m=read_map(os.path.join(mf, mf+".map"))
    starts=getStarts(h,w)
    [print(m[i[0]][i[1]],end=" ") for i in starts]
    goals=getGoals(h,w,m)
    [print(m[i[0]][i[1]],end=" ") for i in goals]
    with open(os.path.join(mf,"scen","human","scen.scen"), "w") as f:
        print("version 1", file=f)
        [print(*[-1,mf+".map",w,h,*starts[i][::-1],*goals[i][::-1],-1], file=f, sep="\t") for i in range(N)]

