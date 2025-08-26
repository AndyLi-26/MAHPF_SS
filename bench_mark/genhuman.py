import os,random
from itertools import chain
N=900

def getStarts(h,w,m):
    def d2e(r,c):
        return min(r,c,h-1-r,w-1-c)

    traversable=[(r,c,d2e(r,c))
                 for r in range(h)
                 for c in range(w)
                 if m[r][c]=="."]

    if not traversable: return []

    min_d = min(d for _,_,d in traversable)

    candidate = [(r,c) for (r,c,d) in traversable if d==min_d]
    return random.choices(candidate,k=N)

    #return random.choices(list(chain(*[[(1,i),(h-2,i)] for i in range(1,w-1)]+[[(i,1),(i,w-2)] for i in range(1,h-1)])),k=N)

def getGoals(h,w,m):
    return random.choices([(i,j) for i in range(h) for j in range(w) if m[i][j]=="."],k=N)

r=lambda f: f.readline().split()
def read_map(fn):
    with open(fn) as f:
        _,h,w,_,m=r(f),int(r(f)[1]),int(r(f)[1]),r(f),[list(l.strip())for l in f]
    return h,w,m

root= os.getcwd()
for mf in [item for item in os.listdir(root) if os.path.isdir(os.path.join(root, item))]:
    #print(mf)
    h,w,m=read_map(os.path.join(mf, "map.map"))
    #print(h,w)
    #print(m)
    for i in range(1):
        starts=getStarts(h,w,m)
        goals=getGoals(h,w,m)
        #print(starts)
        #exit()
        #with open(os.path.join(mf,"scen","human",f"scen-{i}.scen"), "w") as f:
        with open(os.path.join(mf,f"human-{i}.scen"), "w") as f:
            print("version 1", file=f)
            [print(*[-1,mf+".map",w,h,*starts[i][::-1],*goals[i][::-1],-1], file=f, sep="\t") for i in range(N)]
