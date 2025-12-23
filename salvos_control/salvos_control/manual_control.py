import pygame
import time
import math



class ManualControl():

    def __init__(self, height, x, y):
        self.height = height
        self.x = x
        self.y = y
        self.thrust = 0.0
        self.yaw = 0.0
        self.max_pitch = math.pi / 10


    def detect_keys(self, keys: list):
        
        pygame.event.pump()

        pressed = pygame.key.get_pressed()
        out = {}

        for k in keys:
            #eg translate 'w' -> pygame.K_w
            try:
                key_const = getattr(pygame, f'K_{k}')
            except AttributeError as e:
                raise ValueError(f"Unsupported key name: {k!r}") from e
            out[k] = bool(pressed[key_const])

        return out



    def keys_to_cmd(self, kp):
        def resolve(a, b):
            return -1 if a == b else a

        pairs = []
        for a_key, b_key in [('o', 'l'), ('d', 'a'), ('w', 's'), ('i', 'p')]:
            a = kp.get(a_key, False)
            b = kp.get(b_key, False)
            pairs.append(resolve(a, b))


        cmd = [0 if v == -1 else (self.max_pitch if v else -self.max_pitch) for v in pairs] #= [thrust, roll, pitch, yaw], placeholder!

        self.thrust += cmd[0]*math.pi

        if self.thrust <= 0:
            self.thrust = 0


        self.thrust = cmd[0]*4
        cmd.pop(0)
        cmd.insert(0, self.thrust)
        cmd.append(self.height)
        cmd.append(self.x)
        cmd.append(self.y)

        return cmd





def slider_input():
    return None


def cmd_to_rpy(cmd):

    max_angle = math.pi / 8





    




if __name__ == '__main__':
    pygame.init()
    pygame.display.set_caption("Keyboard capture (pygame)")
    pygame.display.set_mode((200, 100))

    try:
        clock = pygame.time.Clock()
        keys_to_watch = ['w', 'a', 's', 'd']

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            state = detect_keys(keys_to_watch)
            print(state)

            if pygame.key.get_pressed()[pygame.K_ESCAPE]:
                break

            clock.tick(20) 

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
