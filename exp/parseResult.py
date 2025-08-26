import matplotlib.pyplot as plt
import matplotlib.cm as cm
import sys,random
def conv(s):
    s=s.strip()
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s

def filterItem(L, cond):
    return [l for l in L if cond(l)]

def byID(idx):
    return lambda l: l[0:4]==list(idx)

def grabInsIDX(L):
    retval=set()
    for l in L:
        retval.add((l[0],l[1],l[2],l[3]))

    return retval

#def findItem(L,idx):
#    return [l for l in L if l[0:4]==list(idx)]

with open(sys.argv[1]) as f:
    motherload=[list(map(conv,l.strip().split(",")[:-1])) for l in f]

print("total ins: ",len(motherload))
withConf=filterItem(motherload,lambda l: l[8]!=0)
'''
print("with init conf",len(withConf))
mergeFail=filterItem(withConf,lambda l: l[-2]==-1)
print("with merge fail",len(mergeFail))
temp=grabInsIDX(mergeFail)
m=dict()
for l in mergeFail:
    if l[0] in m:
        m[l[0]]+=1
    else:
        m[l[0]]=1
print(m)
exit()
'''

def sanityCheck(l):
    print(l)
    assert len(l)==len(mergeAlgo)
    return min(l)==l[-1]

    for i in range(len(l)-1):
        if l[i] and l[i+1]:
            if (not l[i+1]>=l[i]):
                print(l)
                return False
    return True

def findDif(l):
    if l[3]!=9999999 and l[-1]!=9999999:
        return l[3]-l[-1]
    return None


def genColor(n):
    colormap=cm.get_cmap("gist_rainbow",n)
    temp=[colormap(i) for i in range(n)]
    random.shuffle(temp)
    return temp

def genMarkers(n):
    marker_styles = ['o', 's', '^', 'v', '<', '>', 'p', '*', 'h', 'H', 'D', 'd', 'X', '|', '_']
    random.shuffle(marker_styles)
    return marker_styles[:n]


maps=["empty-8-8", "empty-16-16", "random-32-32-10", "warehouse-10-20-10-2-1"]
#mergeAlgo=["stop", "superMCP","MCP","Sub-OPTIMAL-P1","Sub-OPTIMAL","OPTIMAL"]
mergeAlgo=["stop","Sub-OPTIMAL-P1","Sub-OPTIMAL","OPTIMAL"]
colormap=genColor(len(mergeAlgo))
markers=genMarkers(len(mergeAlgo))

merge2i=lambda i:mergeAlgo.index(i)
maxDIF=-1
maxDIFID=-1
for mi,m in enumerate(maps):
    #plt.subplot(1,4,mi+1)
    plt.figure()
    localL = filterItem(withConf,lambda l: m in l[0])
    IDXS=grabInsIDX(localL)
    x=dict()
    for idx in IDXS:
        x[idx[2]]=[]
        for i in range(len(mergeAlgo)):
            x[idx[2]].append([])

    for idx in IDXS:
        tmp=filterItem(localL, lambda l: l[0:4]==list(idx))
        #[print(i) for i in tmp]
        assert len(tmp)==len(mergeAlgo)

        sanity=[9999999 for _ in mergeAlgo ]
        for l in tmp:
            i=merge2i(l[-3])
            if l[-1]:
                sanity[i]=l[-1]
            x[idx[2]][i].append(l[-1])
        print(idx)
        assert sanityCheck(sanity)

        dif=findDif(sanity)
        if dif and dif>maxDIF:
            maxDIF=dif
            maxDIFID=idx


    x_idx=[k for k in x]
    finalx=[[] for _ in mergeAlgo]
    finaly=[[] for _ in mergeAlgo]
    for xi in x_idx:
        assert len(x[xi])==len(mergeAlgo)
        for yi in range(len(x[xi])):
            tmpl=[i for i in x[xi][yi] if i]
            for y in tmpl:
                finalx[yi].append(xi)
                finaly[yi].append(y)


    for i in range(len(finaly)):
        plt.scatter(finalx[i],finaly[i],marker=markers[i],facecolors="none",edgecolors=colormap[i],label=mergeAlgo[i])

    plt.xticks(x_idx)
    plt.title(m)
    plt.legend()
#plt.show()
#plt.tight_layout()
#plt.show()

print(maxDIFID,maxDIF)
temp=filterItem(withConf,byID(maxDIFID))
[print(i) for i in temp]
