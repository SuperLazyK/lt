from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import pygame.locals as pl
import numpy as np
import sys

#-------------------------
# Draw
#-------------------------

def draw_circle_cmd(p, r, color=None, width=1, name="-"):
    return [{"type": "circle", "color":color, "origin":(p[0], p[1]), "r":r, "width":width, "name":name}]

def draw_lineseg_cmd(p0, p1, color=None, width=1, name="-"):
    return [{"type": "lineseg", "color":color, "start":(p0[0], p0[1]), "end":(p1[0], p1[1]), "width":width, "name":name}]

def draw_arcseg_cmd(p, r, th0, th1, color=None, width=1, name="-"):
    return [{"type": "arcseg", "color":color, "origin":(p[0], p[1]), "r":r, "start":th0, "end": th1, "width":width, "name":name}]

def draw_geo(geo, color=None, width=1, name="-"):
    if geo["type"] == "lineseg":
        return draw_lineseg_cmd(geo["start"], geo["end"], color, width, name)
    elif geo["type"] == "arcseg":
        return draw_arcseg_cmd(geo["origin"], geo["r"], geo["start"], geo["end"], color, width, name)
    elif geo["type"] == "circle":
        return draw_circle_cmd(geo["origin"], geo["r"], color, width, name)
    elif geo["type"] == "eqtri":
        c = geo["origin"]
        r = geo["r"]
        th = geo["th"]
        p0 = (c[0] + r * np.cos(th),             c[1] + r * np.sin(th))
        p1 = (c[0] + r * np.cos(th + 2*np.pi/3), c[1] + r * np.sin(th + 2*np.pi/3))
        p2 = (c[0] + r * np.cos(th + 4*np.pi/3), c[1] + r * np.sin(th + 4*np.pi/3))
        return draw_lineseg_cmd(p0, p1) + draw_lineseg_cmd(p1, p2) + draw_lineseg_cmd(p2, p0)

def draw_geos(geos, color=None, width=1, name="-"):
    ret = []
    for g in geos:
        ret = ret + draw_geo(g)
    return ret

def arr2txt(a, title=""):
    a2 = a.reshape(-1)
    return " ".join([f"{title}[{i}]: {a2[i]:.03f}" for i in range(a2.shape[0])])

SCREEN_SIZE=(1300, 500)

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

def default_event_handler(key, shifted):
    if key == 'q':
        sys.exit()

def s2color(s):
    if s == "white":
        return (255, 255, 255)
    elif s == "red":
        return (170, 0, 0)
    elif s == "green":
        return (0, 170, 0)
    elif s == "blue":
        return (0, 0, 170)
    elif s is None or s == "black":
        return (0, 0, 0)
    else:
        return s

class Viewer():
    def __init__(self, scale=1, screen_size=SCREEN_SIZE, offset=[0, 0]):
        pygame.init()
        pygame.event.clear()
        self.scale = scale
        self.screen_size = screen_size
        self.screen = pygame.display.set_mode(self.screen_size)
        self.offset = offset
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Calibri', 15, True, False)

    def text(self, ss, color=None):
        for i, s in enumerate(ss):
            text = self.font.render(s, True, s2color(color))
            self.screen.blit(text, [100, i*30+100])

    def conv_pos(self, p):
        ret = self.scale * np.array([p[0], -p[1]]) + np.array(self.screen_size)/2 +  np.array([self.screen_size[0] * self.offset[0], self.screen_size[1] * self.offset[1]])
        return ret

    def clear(self):
        self.screen.fill(WHITE)

    def handle_event(self, handler = None):
        for event in pygame.event.get():
            if event.type == pl.QUIT:
                pygame.quit()
                sys.exit()
            if handler is None:
                continue
            if event.type == pl.KEYDOWN or event.type == pl.KEYUP:
                keyname = pygame.key.name(event.key)
                mods = pygame.key.get_mods()
                handler(keyname, "DOWN" if event.type == pl.KEYDOWN else "UP", mods & pl.KMOD_LSHIFT)

    def draw_horizon(self, y):
        pygame.draw.line(self.screen, BLACK, self.conv_pos((-1000,y)), self.conv_pos((1000,y)), width=int(2))

    def draw(self, cmds):
        for cmd in cmds:
            if cmd["type"] == "lineseg":
                pygame.draw.line(self.screen, s2color(cmd.get("color")), self.conv_pos(cmd["start"]), self.conv_pos(cmd["end"]), width=int(1))
            elif cmd["type"] == "circle":
                pygame.draw.circle(self.screen, s2color(cmd.get("color")), self.conv_pos(cmd["origin"]), self.scale * cmd["r"], width=int(1))
            elif cmd["type"] == "arcseg":
                c = self.conv_pos(cmd["origin"])
                r = self.scale * cmd["r"]
                rect = pygame.Rect(c[0] - r, c[1] - r, 2 * r, 2 * r)
                pygame.draw.arc(self.screen, s2color(cmd.get("color")), rect, cmd["start"], cmd["end"], width=int(1))

    def flush(self, Hz):
        pygame.display.flip()
        self.clock.tick(Hz)


if __name__ == '__main__':
    viewer = Viewer()

    def event_handler(key, type, shifted):
        if key == 'q':
            sys.exit()

    while True:
        viewer.handle_event(event_handler)
        viewer.clear()
        viewer.text("test")
       # viewer.draw(draw_arcseg_cmd((100, 100), 100, np.deg2rad(0), np.deg2rad(359)))
       # viewer.draw(draw_lineseg_cmd((0, 0), (100, 100)))
        viewer.draw(draw_geo({"type":"eqtri", "origin":(100, 100), "r":100, "th":np.deg2rad(60)}))
        viewer.draw_horizon(0)
        viewer.flush(30)



