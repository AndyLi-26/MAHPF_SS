import tkinter as tk
import re,math,sys
from agent import Agent

PRT_PATH=False

class AnimationGenerator:
    def __init__(self, root,fn_map, fn_log):
        self.root = root
        self.root.title("Animation Generator")
        self.canvas_size=(1500,1000)

        # Set up the canvas
        self.canvas = tk.Canvas(root, width=self.canvas_size[0], height=self.canvas_size[1], bg="white")
        self.canvas.pack()
        # Create an object to animate (e.g., a circle)
        self.paused = False  # Animation paused state
        self.t=0

        # Bind the space key to toggle pause
        self.root.bind("<space>", self.toggle_pause)

        self.map=[]
        self.read_map(fn_map)
        self.read_log(fn_log)
        self.draw_map()
        if PRT_PATH:
            self.draw_path()
        self.create_image()
        self.inc=20

        #print(len(self.agent))
        print(self.make_span)
        self.animate()

    def animate(self):
        if not self.paused and self.t<self.make_span-1:
            # Update the position of the circle
            self.t+=(self.inc/1000)
            self.canvas.itemconfig(self.text_item, text=f"Time Step: {round(self.t,3)}")

            for a in self.agents:
                pos=a.getCoord(self.t)
                self.canvas.coords(a.IDX["circle"],*pos)

        # Schedule the next frame
        self.root.after(self.inc, self.animate)  # 20 ms delay for each frame

    def toggle_pause(self, event):
        self.paused = not self.paused  # Toggle the paused state

    def read_map(self, fn_map):
        with open(fn_map) as f:
            [f.readline() for i in range(4)]
            for l in f:
                l=l.strip()
                l=re.sub('[.GS]','1',l)
                l=re.sub('[@TO]','0',l)
                self.map.append(l)
        self.cell_size=min(self.canvas_size[0]/len(self.map[0]), self.canvas_size[1]/len(self.map))

    def read_log(self, fn_log):
        self.make_span=-1
        self.agents=[]
        #self.human_path=[]
        #self.robot_path=[]
        with open(fn_log) as f:
            for l in f:
                if "human" in l:
                    role="H"
                    continue
                if "robot" in l:
                    role="R"
                    continue
                l=[tuple(map(int,i.split(","))) for i in l.strip("\n;").split(";")]
                a=Agent(role,len(self.agents),self.cell_size)
                a.setPath(l)
                a.setColor("red" if role=="H" else "blue")
                self.agents.append(a)
                self.make_span=max(self.make_span,len(l))


    def draw_map(self):
        def generate_colors(n):
            colors = []
            for i in range(n):
                r = int(255 * (i / n)) % 256
                g = int(255 * ((i * 2) / n)) % 256
                b = int(255 * ((i * 3) / n)) % 256
                hex_color = f'#{r:02x}{g:02x}{b:02x}'
                colors.append(hex_color)
            return colors
        for i, row in enumerate(self.map):
            for j, value in enumerate(row):
                coord=self.coord2pos(i,j)
                color = "white" if value == '1' else "black"
                self.canvas.create_rectangle(*coord, fill=color, outline="gray")

    def draw_path(self):
        for a in self.agents:
            s=a.getStart()
            for e in i[1:]:
                pos1=self.coord2center_pos(*s)
                pos2=self.coord2center_pos(*e)
                self.canvas.create_line(*pos1,*pos2, fill=a.color, width=2)
                s=e

    def create_image(self):
        self.text_item = self.canvas.create_text(1300, 150, text=f"Time Step: {self.t}", font=("Helvetica", 24), fill="black")

        #temp=generate_colors(len(self.human_path)+len(self.robot_path))

        for a in self.agents:
            s=a.getStart()
            pos=a.coord2pos(*s)
            cir = self.canvas.create_oval(*pos, fill=a.color)
            start = self.create_star(self.coord2center_pos(*s), a.color, 3)
            end = self.create_star(self.coord2center_pos(*a.getGoal()), a.color, 5)
            a.setIDX({"circle":cir,"start":start,"end":end})
            #self.humans.append((agent_cir,"red"))

    def coord2center_pos(self,i,j):
        p=self.coord2pos(i,j)
        return (p[0]+p[2])/2, (p[1]+p[3])/2

    def coord2pos(self,i,j):
        x0 = j * self.cell_size
        y0 = i * self.cell_size
        x1 = x0 + self.cell_size
        y1 = y0 + self.cell_size
        return x0,y0,x1,y1

    def create_star(self, center, c, p):
        outer_angle = 2 * math.pi / p  # Full circle divided by the number of points
        inner_angle = outer_angle / 2       # Angle between outer and inner points
        coordinates = []

        for i in range(p * 2):  # 5 outer and 5 inner points, so total 10
            angle = i * outer_angle / 2  # Alternate between outer and inner points
            radius = self.cell_size/3 if i % 2 == 0 else self.cell_size / 6
            # Calculate coordinates
            x = center[0] + radius * math.cos(angle)
            y = center[1] - radius * math.sin(angle)
            coordinates.append((x, y))
        flattened_coordinates = [coord for p in coordinates for coord in p]
        return self.canvas.create_polygon(flattened_coordinates, outline=c, fill=c, width=2)


if __name__ == "__main__":
    root = tk.Tk()
    app = AnimationGenerator(root, sys.argv[1],sys.argv[2])
    root.mainloop()

