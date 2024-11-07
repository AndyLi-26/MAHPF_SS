import tkinter as tk
import re

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
        self.create_image()
        self.inc=20

        print(len(self.human_path[0]))
        print(self.make_span)
        self.animate()

    def animate(self):
        if not self.paused and self.t<self.make_span-1:
            # Update the position of the circle
            self.t+=(self.inc/1000)
            self.canvas.itemconfig(self.text_item, text=f"Time Step: {round(self.t,3)}")

            for i in range(len(self.human_path)):
                if self.t>=len(self.human_path[i])-1:
                    pos=self.coord2pos(*self.human_path[i][-1])
                elif self.t==int(self.t):
                    pos=self.coord2pos(*self.human_path[i][int(self.t)])
                else:
                    pos1=self.coord2pos(*self.human_path[i][int(self.t)])
                    pos2=self.coord2pos(*self.human_path[i][int(self.t)+1])
                    pos=self.shift_agent(pos1,pos2)
                self.canvas.coords(self.humans[i][0],*pos)

            for i in range(len(self.robot_path)):
                if self.t>=len(self.robot_path[i])-1:
                    pos=self.coord2pos(*self.robot_path[i][-1])
                elif self.t==int(self.t):
                    pos=self.coord2pos(*self.robot_path[i][int(self.t)])
                else:
                    pos1=self.coord2pos(*self.robot_path[i][int(self.t)])
                    pos2=self.coord2pos(*self.robot_path[i][int(self.t)+1])
                    pos=self.shift_agent(pos1,pos2)
                self.canvas.coords(self.robots[i][0],*pos)
            '''
            self.canvas.move(self.circle, self.dx, self.dy)

            # Get the current position of the circle
            x0, y0, x1, y1 = self.canvas.coords(self.circle)

            # Check for collision with canvas boundaries
            if x0 <= 0 or x1 >= self.canvas.winfo_width():
                self.dx = -self.dx
            if y0 <= 0 or y1 >= self.canvas.winfo_height():
                self.dy = -self.dy
            '''

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
        self.human_path=[]
        self.robot_path=[]
        with open(fn_log) as f:
            for l in f:
                if "human" in l:
                    curr=self.human_path
                    continue
                if "robot" in l:
                    curr=self.robot_path
                    continue
                l=[tuple(map(int,i.split(","))) for i in l.strip("\n;").split(";")]
                curr.append(l)
                self.make_span=max(self.make_span,len(l))

    def draw_map(self):
        for i, row in enumerate(self.map):
            for j, value in enumerate(row):
                coord=self.coord2pos(i,j)
                color = "white" if value == '1' else "black"
                self.canvas.create_rectangle(*coord, fill=color, outline="gray")

    def create_image(self):
        self.text_item = self.canvas.create_text(1300, 150, text=f"Time Step: {self.t}", font=("Helvetica", 24), fill="black")
        def generate_colors(n):
            colors = []
            for i in range(n):
                r = int(255 * (i / n)) % 256
                g = int(255 * ((i * 2) / n)) % 256
                b = int(255 * ((i * 3) / n)) % 256
                hex_color = f'#{r:02x}{g:02x}{b:02x}'
                colors.append(hex_color)
            return colors

        temp=generate_colors(len(self.human_path)+len(self.robot_path))
        self.humans=[]
        for i in self.human_path:
            s=i[0]
            a_color="red"
            pos=self.coord2pos(*s)
            agent_cir = self.canvas.create_oval(*pos, fill=a_color)
            self.humans.append((agent_cir,a_color))
            for e in i[1:]:
                pos1=self.coord2center_pos(*s)
                pos2=self.coord2center_pos(*e)
                self.canvas.create_line(*pos1,*pos2, fill=a_color, width=2)
                s=e

        self.robots=[]
        for i in self.robot_path:
            s=i[0]
            a_color=temp.pop()
            pos=self.coord2pos(*s)
            agent_cir = self.canvas.create_oval(*pos, fill=a_color)
            self.robots.append((agent_cir,a_color))
            for e in i[1:]:
                pos1=self.coord2center_pos(*s)
                pos2=self.coord2center_pos(*e)
                self.canvas.create_line(*pos1,*pos2, fill=a_color, width=2)
                s=e

    def coord2center_pos(self,i,j):
        p=self.coord2pos(i,j)
        return (p[0]+p[2])/2, (p[1]+p[3])/2

    def coord2pos(self,i,j):
        x0 = j * self.cell_size
        y0 = i * self.cell_size
        x1 = x0 + self.cell_size
        y1 = y0 + self.cell_size
        return x0,y0,x1,y1

    def shift_agent(self,pos1,pos2):
        shift_t=self.t-int(self.t)
        pos=[(pos2[i]-pos1[i])*shift_t+pos1[i] for i in range(4)]
        return pos





if __name__ == "__main__":
    root = tk.Tk()
    app = AnimationGenerator(root, "../bench_mark/empty-8-8/map.map","path.log")
    root.mainloop()

