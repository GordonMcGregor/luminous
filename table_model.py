
from Tkinter import *
import colorsys
import time

WIDTH = 10
HEIGHT = 10

UPDATE_DELAY = 0.02
REPEAT_COUNT = 25

RGB_NORM = 255.0

ANGLE_NORM = 360.0

rb = ( (255,0,0), (255, 127, 0), (255,255,0), (0, 255, 0), (0, 0, 255), (102, 0, 255), (139, 0, 255))

class Colour:

    def __init__(self, rb_index=0):
        self.rb_index = rb_index % len(rb)
        self.R = rb[self.rb_index][0]
        self.G = rb[self.rb_index][1]
        self.B = rb[self.rb_index][2]

    def __str__(self):
        str = "#%02x%02x%02x" % (self.R, self.G, self.B)
        return str 

    def next_rb(self, direction = 1):
        self.rb_index = (self.rb_index+(1*direction)) % len(rb)
        self.R = rb[self.rb_index][0]
        self.G = rb[self.rb_index][1]
        self.B = rb[self.rb_index][2]

    def increment_hue(self, increment=30):
        hsv = colorsys.rgb_to_hsv(self.R/RGB_NORM, self.G/RGB_NORM, self.B/RGB_NORM)

        new_rgb = colorsys.hsv_to_rgb(hsv[0] + (increment/ANGLE_NORM), hsv[1], hsv[2])

        self.R = new_rgb[0] * RGB_NORM
        self.G = new_rgb[1] * RGB_NORM
        self.B = new_rgb[2] * RGB_NORM


class Application(Frame):

    labels = []
    colours = []
    next_colours = []

    def createWidgets(self):
        for x in xrange(WIDTH+1):
            self.labels.append( [] )
            self.colours.append( [] )
            self.next_colours.append( [] )
            for y in xrange(HEIGHT+1):
                self.labels[x].append( Label(self) )
                self.labels[x][y]["text"] = "    %2d,%2d    " % (x,y)
                self.colours[x].append(Colour(rb_index=y))
                self.next_colours[x].append(Colour(rb_index=y))
                self.labels[x][y].grid(row=y, column=x)

        self.update_colours()
        self.control = Button(self)
        self.control["command"] = self.next_seq
        self.control["text"] = 'cycle'
        self.control.grid(row = y+1, column = 0, columnspan = 3)

        self.quiter = Button(self)
        self.quiter["command"] = self.quit
        self.quiter["text"] = ' quit'
        self.quiter.grid(row = y+1, column = 4, columnspan = 3)

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()


    def update_colours(self):
        for x in xrange(WIDTH+1):
            for y in xrange(HEIGHT+1):
                self.labels[x][y]["bg"] = self.next_colours[x][y]

        self.colours = self.next_colours[:]


    def cycle(self):
        for x in xrange(WIDTH+1):
            for y in xrange(HEIGHT+1):
                self.colours[x][y].next_rb(-1)
        self.next_colours = self.colours[:]
        self.update_colours()

    def multi_cycle(self):
        for x in xrange(REPEAT_COUNT):
            self.cycle()
            time.sleep(UPDATE_DELAY)
            self.master.update()

    def scroll(self, x_dir=1, y_dir=1):
        for x in xrange(WIDTH+1):
            for y in xrange(HEIGHT+1):
                self.next_colours[x][y] = self.colours[((x+(1*x_dir)) % WIDTH)][((y+(1*y_dir))%HEIGHT)]
        self.update_colours()

    def multi_scroll(self):
        for x in xrange(REPEAT_COUNT):
            self.scroll(-1,0)
            time.sleep(UPDATE_DELAY)
            self.master.update()

    def next_seq(self):
        self.multi_cycle()
        for x in xrange(WIDTH+1):
            for y in xrange(HEIGHT+1):
                self.colours[x][y]=Colour(rb_index=x)
        self.update_colours()
        self.multi_cycle()

        for x in xrange(WIDTH+1):
            for y in xrange(HEIGHT+1):
                self.colours[x][y] = Colour(rb_index=1)
                self.colours[4][y] = Colour(rb_index=5)
                self.colours[x][5] = Colour(rb_index=5)

        self.update_colours()
        self.multi_cycle()
        self.multi_scroll()

root = Tk()
app = Application(master=root)
root.lift()
app.mainloop()


