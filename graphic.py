from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import pygame.locals as pl
import numpy as np
import sys
from pprint import pprint

#-------------------------
# Draw
#-------------------------

def draw_circle_cmd(p, r, color=None, width=0, name="-"):
    return [{"type": "circle", "color":color, "origin":(p[0], p[1]), "r":r, "width":width, "name":name}]

def draw_lineseg_cmd(p0, p1, color=None, width=1, name="-"):
    return [{"type": "lineseg", "color":color, "start":(p0[0], p0[1]), "end":(p1[0], p1[1]), "width":width, "name":name}]

def draw_arcseg_cmd(p, r, th0, th1, color=None, width=1, name="-"):
    return [{"type": "arcseg", "color":color, "origin":(p[0], p[1]), "r":r, "start":th0, "end": th1, "width":width, "name":name}]

def draw_poly_cmd(ps, color=None, width=1, name="-"):
    return [{"type": "poly", "color":color, "points":ps, "width":width, "name":name}]

def draw_eqtri_cmd(p, r, th, color=None, width=1, name="-"):
    p0 = (p[0] + r * np.cos(th),             p[1] + r * np.sin(th))
    p1 = (p[0] + r * np.cos(th + 2*np.pi/3), p[1] + r * np.sin(th + 2*np.pi/3))
    p2 = (p[0] + r * np.cos(th + 4*np.pi/3), p[1] + r * np.sin(th + 4*np.pi/3))
    return draw_poly_cmd([p0, p1, p2], color, width, name)

def arr2txt(a, title=""):
    a2 = a.reshape(-1)
    return " ".join([f"{title}[{i}]: {a2[i]:.03f}" for i in range(a2.shape[0])])

SCREEN_SIZE=(1500, 1200)

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
    def __init__(self, scale=1, screen_size=SCREEN_SIZE, offset=[0, 0], cursor_show=False):
        pygame.init()
        pygame.mouse.set_visible(cursor_show)
        pygame.event.clear()
        self.scale = scale
        self.screen_size = screen_size
        self.screen = pygame.display.set_mode(self.screen_size)
        self.offset = offset
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Calibri', 15, True, False)
        self.cursor_show = cursor_show
        self.cur_pos = None


    def text(self, ss, color=None):
        for i, s in enumerate(ss):
            text = self.font.render(s, True, s2color(color))
            self.screen.blit(text, [100, i*30+100])

    def conv_pos(self, p):
        ret = self.scale * np.array([p[0], -p[1]]) + np.array(self.screen_size)/2 +  np.array([self.screen_size[0] * self.offset[0], self.screen_size[1] * self.offset[1]])
        return ret

    def rconv_pos(self, p):
        off = np.array([self.screen_size[0] * self.offset[0], self.screen_size[1] * self.offset[1]])
        ret = (np.array([p[0], p[1]]) - off - np.array(self.screen_size)/2) / self.scale
        ret[1] = -ret[1]
        return ret

    def clear(self):
        self.screen.fill(WHITE)

    def handle_event(self, handler = None):
        for event in pygame.event.get():
            #print(event)
            if event.type == pl.QUIT:
                pygame.quit()
                sys.exit()
            if handler is None:
                continue
            mods = pygame.key.get_mods()
            args = {"shift": mods & pl.KMOD_LSHIFT}
            if event.type == pl.KEYDOWN or event.type == pl.KEYUP:
                keyname = pygame.key.name(event.key)
                handler(keyname, "DOWN" if event.type == pl.KEYDOWN else "UP", args)
            if event.type == pygame.MOUSEBUTTONDOWN:
                args["pos"] = self.rconv_pos(event.pos)
                self.cur_pos = event.pos
                if event.button == pygame.BUTTON_LEFT:
                    handler("LB", "DOWN", args)
                elif event.button == pygame.BUTTON_RIGHT:
                    handler("RB", "DOWN", args)
            if event.type == pygame.MOUSEBUTTONUP:
                args["pos"] = self.rconv_pos(event.pos)
                self.cur_pos = event.pos
                if event.button == pygame.BUTTON_LEFT:
                    handler("LB", "UP", args)
                elif event.button == pygame.BUTTON_RIGHT:
                    handler("RB", "UP", args)
            if event.type == pygame.MOUSEMOTION:
                args["pos"] = self.rconv_pos(event.pos)
                handler("MOVE", "MOVE", args)
                self.cur_pos = event.pos



    def draw_horizon(self, y):
        pygame.draw.line(self.screen, BLACK, self.conv_pos((-1000,y)), self.conv_pos((1000,y)), width=int(2))

    def draw(self, cmds):
        for cmd in cmds:
            if cmd["type"] == "lineseg":
                pygame.draw.line(self.screen, s2color(cmd.get("color")), self.conv_pos(cmd["start"]), self.conv_pos(cmd["end"]), width=1)
            elif cmd["type"] == "poly":
                pygame.draw.polygon(self.screen, s2color(cmd.get("color")), [self.conv_pos(p) for p in cmd['points']], width=0)
            elif cmd["type"] == "circle":
                pygame.draw.circle(self.screen, s2color(cmd.get("color")), self.conv_pos(cmd["origin"]), self.scale * cmd["r"], width=cmd["width"])
            elif cmd["type"] == "arcseg":
                c = self.conv_pos(cmd["origin"])
                r = self.scale * cmd["r"]
                rect = pygame.Rect(c[0] - r, c[1] - r, 2 * r, 2 * r)
                pygame.draw.arc(self.screen, s2color(cmd.get("color")), rect, cmd["start"], cmd["end"], width=1)

    def flush(self, Hz):
        if not self.cursor_show:
            lx = self.screen_size[0]
            ly = self.screen_size[1]
            if self.cur_pos is not None:
                x = self.cur_pos[0]
                y = self.cur_pos[1]
                pygame.draw.line(self.screen, BLACK, (-lx,y), (lx,y), width=int(1))
                pygame.draw.line(self.screen, BLACK, (x,-ly), (x,ly), width=int(1))
        pygame.display.flip()
        self.clock.tick(Hz)


