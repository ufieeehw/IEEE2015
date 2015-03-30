import pygame

class Text_Box(object):
    '''Text_Box()
    You never have to initialize this! Just call Text_Box.draw(display, pos, color, text)
    It draws the same way a pygame primitive would.
    '''
    pygame.font.init()
    font = pygame.font.SysFont("monospace", 15)


    @classmethod
    def draw(self, display, pos=(0, 0), color=(255, 255, 255), text="Empty!"):
        ''' draw(display, pos=(0, 0), color=(255, 255, 255), text="Empty!"):

        pos: In pygame coordinates
        color: [0, 255]
        text: Can by multiline, of arbitrary length

        To change text during operation, use the "set_text method"
        Ex:
            >>> tb = Text_Box()
            >>> tb.draw(display, text='hello')
        or in a draw loop,
            >>> tb.draw(display, pos, color, text)
        '''
        lines = text.splitlines()
        width = height = 0
        for l in lines:
            width = max(width, self.font.size(l)[0])
            height += self.font.get_linesize()

        height = 0
        for l in lines:
            t = self.font.render(l, 0, color)
            display.blit(
                t, 
                (pos[0], pos[1] + height)
            )
            height += self.font.get_linesize()
