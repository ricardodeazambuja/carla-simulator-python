import numpy as np

import pygame


class CarlaPygameHelper():

    def __init__(self, width=800, height=600, default_font='ubuntumono'):
        pygame.init()
        self.display = pygame.display.set_mode((width,height), 
                                               pygame.HWSURFACE | pygame.DOUBLEBUF) # https://stackoverflow.com/a/29135900/7658422
        self.font = self.get_font(default_font)
        self.clock = pygame.time.Clock()

    def blit(self, *args):
        self.display.blit(*args)

    @staticmethod
    def flip():
        pygame.display.flip()

    @staticmethod
    def quit():
        pygame.quit()

    def draw_image(self, image, blend=False):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if blend:
            image_surface.set_alpha(100)
        self.display.blit(image_surface, (0, 0))


    def get_font(self, default_font):
        fonts = [x for x in pygame.font.get_fonts()]
        font = default_font if default_font in fonts else fonts[0]
        font = pygame.font.match_font(font)
        return pygame.font.Font(font, 14)


    def should_quit(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_ESCAPE:
                    return True
        return False
