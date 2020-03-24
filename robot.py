import random
import math
import pygame
import pygame.gfxdraw as gfxdraw
import numpy as np
from shapely.geometry import *

WHITE = (255, 255, 255)
GREEN = (20, 255, 140)
GREY = (210, 210, 210)
RED = (255, 0, 0)
PURPLE = (255, 0, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GOLD = (255,215,0)
SILVER = (192, 192, 192)
LIGHTBLUE = (135,206,250)
VERDE = (71,243,52)

scaler = 10
PI = np.pi
cos = np.cos
sin = np.sin
tan = np.tan
arctan = np.arctan
decrease_factor = 1
increase_factor = 1

class Robot:
    def __init__(self, x, y, left_velocity, right_velocity, theta,radius, walls):
        self.x = x
        self.y = y
        self.reset_position()
        self.radius = radius
        self.history = []
        self.color = GREEN
        self.left_velocity = left_velocity
        self.right_velocity = right_velocity
        self.velocity = (self.left_velocity + self.right_velocity) / 2
        self.theta = (theta/180)*PI
        self.omega = None
        self.icc_radius = None
        self.icc_centre_x = None
        self.icc_centre_y = None
        self.update_icc()
        self.time = 0
        self.decrease_factor = 1
        self.increase_factor = 1
        self.walls = walls
        self.collision = None
        self.collided_wall = None
        self.to_collide = []
        self.time_step = 1
        self.sensors = []
        self.environment = Environment(1)
        self.text = None
        self.collision1 = 0 # for the fitness

    def update_velocities(self, left, right):
        self.left_velocity = left
        self.right_velocity = right

    def get_sensors(self):
        return [np.round(sensor.value, 3) for sensor in self.sensors]

    def reset_position(self):
        count = 0
        while (self.wall_robot()):
            count += 1
            self.x = self.x + random.randint(-50, 50)
            self.y = self.y + random.randint(-50, 50)
            print(count)
            if count > 15:
                break
        return

    def wall_robot(self):
        for wall in walls:
            project = wall.linestring.project(Point((self.x, self.y)))
            nearest_point = wall.linestring.interpolate(project).coords
            point2 = Point([nearest_point[0][0], nearest_point[0][1]])
            # if point2.distance(wall.linestring) > 2:
            if point2.distance(Point((self.x, self.y))) < 25:
                return True
        return False

    def draw(self, display = True, display_history = True):
        if len(self.history) > 10 and display:
            if display_history:
                for i, h in enumerate(self.history[2:-2]):
                    pygame.draw.line(screen, self.color, self.history[i], self.history[i+1], 3)

        if abs(self.x) > 10 ** 7 or abs(self.y > 10 ** 7):
            # this is for odd number,
            pygame.draw.circle(screen, self.color, (500, 500), 1)
            # self.history.append([500, 500])
            self.sensors = []
            angle = 0
            while (angle <= 359):
                self.sensors.append(Sensor([500, 500]))
                angle = angle + 30
            return
        else:
            if display:
                pygame.draw.circle(screen, self.color, (int(round(self.x)), int(round(self.y))), self.radius)
            self.history.append([int(round(self.x)), int(round(self.y))])

        angle = 0
        self.sensors = []
        if self.text is not None:
            blit_text(f"{self.text}", self.x, self.y, BLACK, SILVER, 18)

        while (angle <= 359):
            self.sensors.append(Sensor([self.x + self.radius * cos(self.theta + angle * PI / 180), \
                                       self.y + self.radius * sin(self.theta + angle * PI / 180)]))
            angle = angle + 30

    def draw_sensors(self, display = True):
        circle_centre = Point((self.x, self.y))
        for sensor in self.sensors:
            sensor_point = Point((sensor.x, sensor.y))
            sensor.to = None
            if display:
                pygame.draw.circle(screen, BLUE, (int(round(sensor.x)),int(round(sensor.y))), 2)
            sensor_distance = np.inf
            for wall in walls:
                px, py = sensor.line_line_intersection(self.x, self.y, wall)
                sensor_reach = Point((px, py))
                point_in_the_wall =  wall.linestring.distance(sensor_reach)
                sensor_to_wall = sensor_point.distance(sensor_reach)
                if point_in_the_wall >= 1 or sensor_to_wall >= 200:
                    continue
                if sensor_to_wall < sensor_distance:
                    sensor_distance = sensor_to_wall
                    sensor.observed_wall = wall
                    sensor.to = sensor_reach
                    sensor.dist_to_wall = round(sensor_to_wall, 2)
            if (sensor.to is None):
                if display:
                    blit_text("> 200", sensor.x, sensor.y, BLACK, None, 12)
                sensor.value = 200
                continue
            line_from_sensor_to_wall = LineString((sensor_point, sensor.to))
            centre_on_the_line = line_from_sensor_to_wall.distance(circle_centre)
            if centre_on_the_line <= 2:
                if display:
                    blit_text("> 200", sensor.x, sensor.y, BLACK, None, 12)
                sensor.value = 200
                continue
            if display:
                pygame.draw.line(screen, SILVER, \
                                 (int(round(sensor.x)), int(round(sensor.y))), \
                                 (int(round(sensor.to.xy[0][0])), int(round(sensor.to.xy[1][0]))), 2)
                blit_text(f"{sensor.dist_to_wall}", sensor.to.xy[0][0], sensor.to.xy[1][0], BLACK, SILVER, 12)
            sensor.value = sensor.dist_to_wall
        # print([x.value for x in self.sensors])
        return

    def draw_icc(self, display = False):
        if (abs(self.left_velocity - self.right_velocity) < 1):
            return
        if display:
            print("MENSCH")
            print(self.icc_centre_x)
            print(self.icc_centre_y)
            print(int(round(self.icc_centre_x)))
            print(int(round(self.icc_centre_y)))
            pygame.draw.circle(screen, PURPLE, [int(round(self.icc_centre_x)), int(round(self.icc_centre_y))], 2)
        return

    def draw_direction(self, display = True):
        p1 = (int(round(self.x + .25 * self.radius * cos(self.theta))), \
              int(round(self.y + .25 * self.radius * sin(self.theta))))
        p4 = (int(round(self.x + self.radius * cos(self.theta))), \
            int(round(self.y + self.radius * sin(self.theta))))
        p6 = (int(round(self.x + .25 * self.radius * cos(self.theta - PI / 6))), \
              int(round(self.y + .25 * self.radius * sin(self.theta - PI / 6))))
        p5 = (int(round(self.x + self.radius * cos(self.theta - PI / 6))), \
              int(round(self.y + self.radius * sin(self.theta - PI / 6))))
        p2 = (int(round(self.x + .25 * self.radius * cos(self.theta + PI / 6))), \
              int(round(self.y + .25 * self.radius * sin(self.theta + PI / 6))))
        p3 = (int(round(self.x + self.radius * cos(self.theta + PI / 6))), \
              int(round(self.y + self.radius * sin(self.theta + PI / 6))))
        try:
            if (display):
                pygame.draw.polygon(screen, SILVER, [p1, p2, p3, p4, p5, p6])
            else:
                return
        except:
            print(([p1, p2, p3, p4, p5, p6]))
    def speedup_left(self):
        self.left_velocity += self.increase_factor

    def slowdown_left(self):
        self.left_velocity -= self.decrease_factor

    def speedup_right(self):
        self.right_velocity += self.increase_factor

    def slowdown_right(self):
        self.right_velocity -= self.decrease_factor

    def speedup_both(self):
        self.left_velocity += self.increase_factor
        self.right_velocity += self.increase_factor

    def slowdown_both(self):
        self.left_velocity -= self.decrease_factor
        self.right_velocity -= self.decrease_factor

    def stop_both(self):
        self.right_velocity = self.left_velocity = 0

    def update_icc(self):
        self.velocity = (self.left_velocity + self.right_velocity) / 2
        self.omega = (self.right_velocity - self.left_velocity) / (2 * self.radius)
        self.icc_radius = self.radius * ((self.left_velocity) + (self.right_velocity)) / (
                (self.right_velocity - self.left_velocity) + 0.0001)
        self.icc_centre_x = self.x - self.icc_radius * sin(self.theta)
        self.icc_centre_y = self.y + self.icc_radius * cos(self.theta)
        max_v = max(abs(self.left_velocity), abs(self.right_velocity))
        if (max_v // 5 != 0):
            self.time_step = round(1 / (max_v // 5), 2)
        else:
            self.time_step = 1
        self.time_step = 1/1

    def move(self, color = GREEN, text = None):
        prev_x, prev_y, prev_theta = self.x, self.y, self.theta
        self.collision = False
        self.collision_num = 0
        self.color = color
        self.go_half = False
        self.ppdistance_to_each_wall()
        self.text = None
        if text is not None:
            self.text = text
        rColliding = self.get_positions_new(self.x, self.y, self.theta)
        sColliding = self.get_positions_new(self.x, self.y, self.theta, slide = True)
        # blit_text(f"rc: {rColliding}, sc: {sColliding}", 500, 900, BLACK, None, 20)

        if (not sColliding and not rColliding) or (sColliding and not rColliding):
            self.x, self.y, self.theta = self.update_pos(time_step = 1)

            if self.collision1 >= -15:
                self.collision1 -= 1
            return

        elif (rColliding and not sColliding):
            self.x, self.y, self.theta = self.slide(time_step = 1)
            self.color = GREY if color is GREEN else color
            if self.collision1 <= 15:
                self.collision1 += 1
            self.update_icc()
            return

        elif (sColliding and rColliding):
            self.theta += self.omega
            self.color = BLACK if color is GREEN else color
            if self.collision1 <= 30:
                self.collision1 += 2
            for wall in self.to_collide:
                wall.color = GOLD
            self.update_icc()
            return
        return

    def update_pos(self, x = None, y = None, theta = None, time_step = 1):
        if x is None:
            x = self.x
            y = self.y
            theta = self.theta
            time_step = 1 / time_step
        else:
            time_step = time_step

        if (self.left_velocity == self.right_velocity):
            new_x = x + time_step * (self.velocity * cos(theta))
            new_y = y + time_step * (self.velocity * sin(theta))
            new_theta = theta + self.omega * time_step
        else:
            p = np.dot(np.array([[cos(self.omega * time_step), -sin(self.omega * time_step), 0],
                                 [sin(self.omega * time_step), cos(self.omega * time_step), 0],
                                 [0, 0, 1]]), \
                       np.array(
                           [self.icc_radius * sin(theta),
                            -self.icc_radius * cos(theta),
                            theta]))
            new_x = p[0] + self.icc_centre_x
            new_y = p[1] + self.icc_centre_y
            new_theta = p[2] + self.omega * time_step
        return [new_x, new_y, new_theta]

    def slide(self, x = None, y = None, theta = None, time_step = 1):
        if x is None:
            x = self.x
            y = self.y
            theta = self.theta
            time_step = 1 / time_step
        slide_on_me = None
        d = np.inf
        if len(self.to_collide) == 1:
            slide_on_me = self.to_collide[0]
        else:
            for w in self.to_collide:
                if w.dist < d:
                    d = w.dist
                    slide_on_me = w

        slide_on_me.color = GOLD
        theta1 = slide_on_me.angle
        # theta1 = self.collided_wall.angle
        direction = round(cos(theta1 - theta), 4)
        # blit_text(f"{direction}", 500, 500, BLACK, None, 20)

        #
        # if (-0.0005 <= direction <= 0.0005 or direction >= 0.9995 or direction <= -0.9995) and len(self.to_collide) == 1:
        #     next_x, next_y, next_theta = self.update_pos(x, y, theta, time_step)
        #     return next_x, next_y, next_theta

        if direction <= 0.0:
            next_x = x + time_step * (self.velocity * - cos(theta1))
            next_y = y + time_step * (self.velocity * - sin(theta1))
        else:
            next_x = x + time_step * (self.velocity * cos(theta1))
            next_y = y + time_step * (self.velocity * sin(theta1))
        next_theta = theta + self.omega * time_step
        return next_x, next_y, next_theta

    def ppdistance_to_each_wall(self):
        point = Point(self.x, self.y)
        closest_dist = np.inf
        closest_wall = None
        self.to_collide = []
        for wall in walls:
            project = wall.linestring.project(point)
            nearest_point = wall.linestring.interpolate(project).coords
            point2 = Point([nearest_point[0][0], nearest_point[0][1]])
            if point2.distance(wall.linestring) > 0.5:
                continue
            # pygame.draw.circle(screen, BLACK, (int(round(point2.xy[0][0])), int(round(point2.xy[1][0]))), 6)
            dist = point.distance(point2)
            if dist < closest_dist:
                closest_dist = dist
                closest_wall = wall
                if dist < self.radius + 2:
                    wall.dist = dist
                    self.to_collide.append(wall)
        if not self.to_collide:
            self.to_collide.append(closest_wall)
        # print(f"clo dist: {closest_dist}")
        return

    def get_positions_new(self, startx, starty, start_theta, slide = False, steps = 2):
        time_step = 1 / steps
        new_x, new_y, new_theta = startx, starty, start_theta
        start_point = Point(startx, starty)
        e_points = []
        for i in range(steps):
            if not slide:
                next_step = self.update_pos(new_x, new_y, new_theta, time_step)
            else:
                next_step = self.slide(new_x, new_y, new_theta, time_step)
            new_x, new_y, new_theta = next_step[0], next_step[1], next_step[2]
            e_points.append(Point([new_x, new_y]))

        line1 = LineString([start_point, e_points[0]])
        line2 = LineString([e_points[0], e_points[1]])
        line3 = LineString([start_point, e_points[1]])
        for wall in walls:
            wall.color = LIGHTBLUE
            wline = wall.linestring
            dist1 = start_point.distance(wline)
            dist2 = e_points[1].distance(wline)
            dist3 = e_points[0].distance(wline)
            # C1 = wline.intersection(line1).coords
            # C2 = wline.intersection(line2).coords
            # C3 = wline.intersection(line3).coords
            C1 = C2 = C3 = []
            # print([bool(C1), bool(C2), bool(C3), bool(dist2 < self.radius), bool(dist3 < self.radius),
            #        bool(dist1 < self.radius)])
            if (C1 or C2 or C3 or dist2 < self.radius or dist3 < self.radius or dist1 < self.radius):
                return True
        return False

class Wall:
    def __init__(self, start_point, end_point, color):
        self.start_point = start_point
        self.end_point = end_point
        self.color = color
        self.m = None
        self.angle = self.get_angle()
        self.dist = None
        self.hit = None
        self.linestring = LineString([(start_point[0], start_point[1]), (end_point[0], end_point[1])])

    def draw(self):
        pygame.draw.line(screen, self.color, self.start_point, self.end_point, 3)
        landMarkStart = LandMark(self.start_point[0], self.start_point[1], 7, GREEN)
        landMarkEnd = LandMark(self.end_point[0], self.end_point[1], 7, GREEN)
        landMarkStart.draw()
        landMarkEnd.draw()

    def get_angle(self):
        if (self.end_point[0] != self.start_point[0]):
            self.m = (self.end_point[1] - self.start_point[1]) / (self.end_point[0] - self.start_point[0])
            return np.arctan(self.m)
        else:
            return PI / 2

class Sensor():
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
        self.value = None

    def line_line_intersection(self, cx, cy, wall):
        x1, y1 = self.x, self.y
        x2, y2 = cx, cy
        x3, y3 = wall.start_point[0], wall.start_point[1]
        x4, y4 = wall.end_point[0], wall.end_point[1]

        self.Px = ( ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / \
               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)) )
        self.Py = ( ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / \
               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)) )
        return self.Px, self.Py

class Dust:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.collected = False

    def draw(self):
        if self.collected:
            return
        if not self.collected:
            gfxdraw.aacircle(screen, int(round(self.x)), int(round(self.y)), 1, GREY)

    def clear(self):
        if not self.collected:
            self.collected = True
        else:
            return

class Environment:
    def __init__(self, density = 1):
        self.all_dusts = []
        self.cleared_dust = 0
        if density == 3:
            interval = 12
            for w in range(8, width - 8, interval):
                for h in range(8, height - 8, interval):
                    self.all_dusts.append(Dust(w, h))
        if density == 2:
            interval = 18
            for w in range(8, width - 8, interval):
                for h in range(8, height - 8, interval):
                    self.all_dusts.append(Dust(w, h))
        if density == 1:
            interval = 24
            for w in range(8, width - 8, interval):
                for h in range(8, height - 8, interval):
                    self.all_dusts.append(Dust(w, h))

    def draw_dusts(self, robot, disaplay = False):
        for d in self.all_dusts:
            if d.collected: continue
            dist = np.sqrt((d.x - robot.x) ** 2 + (d.y - robot.y) ** 2)
            if dist <= robot.radius:
                self.cleared_dust += 1
                d.clear()
            if disaplay:
                d.draw()

class Player(pygame.sprite.Sprite):
    def __init__(self, pos, size=(200, 200)):
        super(Player, self).__init__()
        self.original_image = load_image('img/tireForward_0.png')
        # pygame.draw.line(self.original_image, (255, 0, 255), (size[0] / 2, 0), (size[0] / 2, size[1]), 3)
        # pygame.draw.line(self.original_image, (0, 255, 255), (size[1], 0), (0, size[1]), 3)
        self.image = self.original_image
        self.rect = self.image.get_rect()
        self.rect.center = pos
        self.angle = 0

    def update(self, x1, y1, theta, radius, shift):

        x2 = (x1 + radius * cos(theta))
        y2 = (y1 + radius * sin(theta))
        x2 += shift
        y2 += shift

        # self.image = pygame.transform.rotate(self.original_image, self.angle)
        self.angle = (-np.arctan2((y2 - y1), (x2 - x1)) * 180 / np.pi) - 90
        # x, y = self.rect.center  # Save its current center.
        self.rect = self.image.get_rect()  # Replace old rect with new rect.
        self.rect.center = (x2, y2)  # Put the new rect's center at old center.

class Grid():
    def __init__(self, surface, cell_size):
        self.surface = surface
        self.col = surface.get_width() // cell_size
        self.line = surface.get_height() // cell_size
        self.cell_size = cell_size
        self.grid = [[0 for i in range(self.col)] for j in range(self.line)]

    def draw_grid(self):
        for l in range(self.line):
            liCoord = 5 + l * self.cell_size
            pygame.draw.line(self.surface, SILVER, (5, liCoord),
                             (self.surface.get_width(), liCoord))
        for c in range(self.col):
            colCoord = 5 + c * self.cell_size
            pygame.draw.line(self.surface, SILVER, (colCoord, 5),
                             (colCoord, self.surface.get_height()))

def blit_text(text, x, y, text_color = SILVER, bkg_color = None, font_size = 16):
    font1 = pygame.font.SysFont("futura", font_size)
    text = font1.render(text, True, text_color, bkg_color)
    textRect = text.get_rect()
    textRect.center = (x, y)
    screen.blit(text, textRect)

class LandMark:
    def __init__(self, x, y, radius, color):
        self.color = color
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self):
        pygame.draw.circle(screen, self.color, (int(round(self.x)), int(round(self.y))), self.radius)

def main():
    loopExit = True
    screen.blit(pygame.transform.scale(screen, (1000, 1000)), (0, 0))
    try:
        while loopExit:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    loopExit = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        block.speedup_left()
                        block.update_icc()
                    elif event.key == pygame.K_s:
                        block.slowdown_left()
                        block.update_icc()
                    elif event.key == pygame.K_o:
                        block.speedup_right()
                        block.update_icc()
                    elif event.key == pygame.K_l:
                        block.slowdown_right()  # decrement of left wheel
                        block.update_icc()
                    elif event.key == pygame.K_x:
                        block.stop_both()  # zero both wheel speed
                        block.update_icc()
                    elif event.key == pygame.K_t:
                        block.speedup_both()  # increment both wheel speed
                        block.update_icc()
                    elif event.key == pygame.K_g:
                        block.slowdown_both()  # decrement both wheel speed
                        block.update_icc()
                    elif event.key == pygame.K_ESCAPE:
                        loopExit = False

            screen.fill((255, 128, 128))
            grid.draw_grid()
            # blit_text(f'L: {block.left_velocity}; R: {block.right_velocity}', 800, 300, SILVER, BLACK)
            # screen.blit(player2.image, player2.rect)
            block.move()
            block.draw()
            block.draw_direction()
            for w in walls:
                w.draw()
            block.draw_icc()
            block.draw_sensors()
            e.draw_dusts(block)

            # pygame.display.flip()
            clock.tick(120)
            pygame.display.update()
        pygame.quit()
    except SystemExit:
        pygame.quit()

width = 1000
height = 1000
clock = pygame.time.Clock()

# screen = pygame.display.set_mode((300, 300))
#screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.HWSURFACE)
screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.HWSURFACE | pygame.FULLSCREEN)
walls = []
east_border = Wall((width - 5 , 0), (width - 5, height - 5), LIGHTBLUE)
west_border = Wall((5, 5), (5, height - 5), LIGHTBLUE)
south_border = Wall((5, height - 5), (width - 5, height - 5), LIGHTBLUE)
north_border = Wall((5, 5), (width - 5, 5), LIGHTBLUE)
#
# Double rectangle
# walls.append(Wall((250, 250), (750, 250), LIGHTBLUE))
# # walls.append(Wall((750, 250), (750, 750), LIGHTBLUE))
# # walls.append(Wall((750, 750), (250, 750), LIGHTBLUE))
# # walls.append(Wall((250, 750), (250, 250), LIGHTBLUE))

# for localization
walls.append(Wall((0, 200), (700, 200), LIGHTBLUE))
walls.append(Wall((400, 400), (1000, 400), LIGHTBLUE))
walls.append(Wall((400, 400), (400, 800), LIGHTBLUE))
walls.append(Wall((800, 1000), (800, 600), LIGHTBLUE))


# Trapezoid
#walls.append(Wall((10, 10), (990, 250), LIGHTBLUE))
#walls.append(Wall((990, 250), (990, 750), LIGHTBLUE))
#walls.append(Wall((990, 750), (10, 990), LIGHTBLUE))
#walls.append(Wall((10, 990), (10, 10), LIGHTBLUE))

# Random
# walls.append(Wall((100, 200), (400, 300), LIGHTBLUE))
# walls.append(Wall((600, 500), (800, 900), LIGHTBLUE))
# walls.append(Wall((300, 500), (300, 750), LIGHTBLUE))
# walls.append(Wall((600, 400), (600, 805), LIGHTBLUE))
walls.append(east_border)
walls.append(west_border)
walls.append(south_border)
walls.append(north_border)

e = Environment(1)
block = Robot(520, 690, 2, 3,0, 20, walls)
# block = Robot(220, 290 , 2 , 3 , 20 , walls)
grid = Grid(screen, cell_size=100)

if __name__ == '__main__':
    pygame.init()
    font = pygame.font.SysFont("futura", 16)
    main()