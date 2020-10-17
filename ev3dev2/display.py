# -----------------------------------------------------------------------------
# Copyright (c) 2015 Ralph Hempel <rhempel@hempeldesigngroup.com>
# Copyright (c) 2015 Anton Vanhoucke <antonvh@gmail.com>
# Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>
# Copyright (c) 2015 Eric Pascual <eric@pobot.org>
# Copyright (c) 2020 Lukasz Machura <lukasz.machura@us.edu.pl>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

class Display(object):
    """
    Provides text functions.

    TODO:
    Provides drawing functions from the python imaging library (PIL).
    """

    GRID_COLUMNS = 19
    GRID_COLUMN_PIXELS = None
    GRID_ROWS = 8
    GRID_ROW_PIXELS = None

    def __init__(self, desc='Display'):
        self.platform = "ev3sim"
        self.desc = desc

    def __str__(self):
        return self.desc

    @property
    def xres(self):
        """
        Horizontal screen resolution
        """
        raise NotImplementedError

    @property
    def yres(self):
        """
        Vertical screen resolution
        """
        raise NotImplementedError

    @property
    def shape(self):
        """
        Dimensions of the screen.
        """
        raise NotImplementedError

    @property
    def draw(self):
        """
        Returns a handle to PIL.ImageDraw.Draw class associated with the screen.

        Example::

            screen.draw.rectangle((10,10,60,20), fill='black')
        """
        raise NotImplementedError

    @property
    def image(self):
        """
        Returns a handle to PIL.Image class that is backing the screen. This can
        be accessed for blitting images to the screen.

        Example::

            screen.image.paste(picture, (0, 0))
        """
        raise NotImplementedError

    def clear(self):
        """
        Clears the screen
        """
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'ClearScreen',
            [], [], [], bytearray(),
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode

    def _color565(self, r, g, b):
        """Convert red, green, blue components to a 16-bit 565 RGB value. Components
        should be values 0 to 255.
        """
        raise NotImplementedError

    def _img_to_rgb565_bytes(self):
        raise NotImplementedError

    def update(self):
        """
        Applies pending changes to the screen.
        Nothing will be drawn on the screen until this function is called.
        """
        raise NotImplementedError

    def image_filename(self, filename, clear_screen=True, x1=0, y1=0, x2=None, y2=None):
        raise NotImplementedError

    def line(self, clear_screen=True, x1=10, y1=10, x2=50, y2=50, line_color='black', width=1):
        """
        Draw a line from (x1, y1) to (x2, y2)
        """
        raise NotImplementedError

    def circle(self, clear_screen=True, x=50, y=50, radius=40, fill_color='black', outline_color='black'):
        """
        Draw a circle of 'radius' centered at (x, y)
        """
        raise NotImplementedError

    def rectangle(self, clear_screen=True, x1=10, y1=10, x2=80, y2=40, fill_color='black', outline_color='black'):
        """
        Draw a rectangle where the top left corner is at (x1, y1) and the
        bottom right corner is at (x2, y2)
        """
        raise NotImplementedError

    def point(self, clear_screen=True, x=10, y=10, point_color='black'):
        """
        Draw a single pixel at (x, y)
        """
        raise NotImplementedError

    def text_pixels(self, text, clear_screen=True, x=0, y=0, text_color='black', font=None):
        """
        This will work in the same way as Display.text_grid() function
        (see return call)

        Display ``text`` starting at character (x, y).

        The EV3sim display is 19x8 characters

        - (0, 0) would be the top left corner of the display

        ``text_color`` : only black color allowed

        ``font`` : only default
        """
        self.text_grid(self, text, clear_screen, x, y, text_color, font)

    def text_grid(self, text, clear_screen=True, x=0, y=0, text_color='black', font=None):
        """
        Display ``text`` starting at grid (x, y)

        The EV3 display can be broken down in a grid that is 19 columns wide
        and 8 rows tall.

        ``text_color`` : only black color allowed, leave it 'black'

        ``font`` : only default sim font will work, leavie it None
        """

        assert 0 <= x < Display.GRID_COLUMNS,\
            "grid columns must be between 0 and %d, %d was requested" %\
            ((Display.GRID_COLUMNS - 1, x))

        assert 0 <= y < Display.GRID_ROWS,\
            "grid rows must be between 0 and %d, %d was requested" %\
            ((Display.GRID_ROWS - 1), y)

        if clear_screen:
            self.clear()

        if VERBOSE and (len(text) + x > Display.GRID_COLUMNS):
                print('Your text will not be visible')

        display_text = " " * x + str(text)
        returnCode = self._write_line_on_display(y, display_text)
        return returnCode

    def _write_line_on_display(self, line_number, text):
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'TextOut',
            [line_number], [], [text], bytearray(),
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode

    def reset_screen(self):
        self.clear()
