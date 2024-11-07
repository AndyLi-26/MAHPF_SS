class Agent:
    def __init__(self,role,id,cell_size):
        self.role=role
        self.id=id
        self.cell_size=cell_size

    def setPath(self,path):
        self.path=path

    def setIDX(self,IDX):
        self.IDX=IDX

    def setColor(self,c):
        self.color=c

    def getStart(self):
        return self.path[0]

    def getGoal(self):
        return self.path[-1]

    def getCoord(self,t):
        if t>=len(self.path)-1:
            pos=self.coord2pos(*self.path[-1])
        elif t==int(t):
            pos=self.coord2pos(*self.path[int(t)])
        else:
            pos1=self.coord2pos(*self.path[int(t)])
            pos2=self.coord2pos(*self.path[int(t)+1])
            pos=self.shift(pos1,pos2,t)
        return pos

    def shift(self,pos1,pos2,t):
        shift_t=t-int(t)
        pos=[(pos2[i]-pos1[i])*shift_t+pos1[i] for i in range(4)]
        return pos


    def coord2pos(self,i,j):
        x0 = j * self.cell_size
        y0 = i * self.cell_size
        x1 = x0 + self.cell_size
        y1 = y0 + self.cell_size
        return x0,y0,x1,y1


