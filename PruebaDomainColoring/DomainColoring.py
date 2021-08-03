#Adaptación de código en hernanat/dcolor /// link: https://github.com/hernanat/dcolor/blob/master/dcolor.pyimage p
# J. 3/08/21

from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import matplotlib.colors as mcolors
from matplotlib.colors import hsv_to_rgb


class DColor:
    def __init__(self, samples=4500, xmin=-10, xmax=10, ymin=-10, ymax=10):
        #Hay que cambiar el tamaño predeterminado de samples
        #a uno en función del tamaño del viewbox deseado (pues determinará
        #la cantidad de pixeles en la imagen
        #plot settings
        self._samples = samples
        #axes
        self._xmin = xmin
        self._xmax = xmax
        self._ymin = ymin
        self._ymax = ymax
        self.makeDomain()

    def makeDomain(self):
        """Create the domains for Real (x) and Imaginary (y) values respectively"""
        x = np.linspace(self._xmin, self._xmax, self._samples)
        y = np.linspace(self._ymin, self._ymax, self._samples)
        self.xx, self.yy=np.meshgrid(x,y)

    def makeColorModel(self, zz):
        """Create the HSV color model for the function domain that will be plotted"""
        # Aquí valdría la pena analizar diversos esquemas de coloración
        H = self.normalize(np.angle(zz) % (2. * np.pi)) #Hue determined by arg(z)
        r = np.log2(1. + np.abs(zz))
        S = (1. + np.abs(np.sin(2. * np.pi * r))) / 2.
        V = (1. + np.abs(np.cos(2. * np.pi * r))) / 2.

        return H,S,V

    def normalize(self, arr):
        """Used for normalizing data in array based on min/max values"""
        arrMin = np.min(arr)
        arrMax = np.max(arr)
        arr = arr - arrMin
        return arr / (arrMax - arrMin)

    def plot(self, f, xdim=10, ydim=10, plt_dpi=100):
        """Plot a complex-valued function
            Arguments:
            f -- a (preferably) lambda-function defining a complex-valued function
            Keyword Arguments:
            xdim -- x dimensions
            ydim -- y dimensions
            plt_dpi -- density of pixels per inch
        """
        zz=f(self.z(self.xx,self.yy))
        h,s,v = self.makeColorModel(zz)
        rgb = hsv_to_rgb(np.dstack((h,s,v)))

        # fig = plt.figure(figsize=(xdim, ydim), dpi=plt_dpi)
        # plt.imshow(rgb)
        # plt.gca().invert_yaxis() #make CCW orientation positive
        # plt.gca().get_xaxis().set_visible(False)
        # plt.gca().get_yaxis().set_visible(False)
        # plt.show()

        return rgb

    def z(self, x, y):
        return x+1j*y

#Código para hacer dos ventanitas con plot y con la selección del ROI circular

app = pg.mkQApp("Domain coloring")
pg.setConfigOption('imageAxisOrder', 'row-major') # best performance
## Create window with GraphicsView widget
win = pg.GraphicsLayoutWidget()
win.show()
win.setWindowTitle('Domain Coloring')

w1 = win.addLayout(row=0, col=0)
v1 = w1.addViewBox(row=0, col=0, lockAspect=True)
v2 = w1.addViewBox(row=0, col=1, lockAspect=True)

disk = pg.CircleROI([1200,1200], [600, 600]) #Quisiera que el primer array fuera el centro de la imagen
v1.addItem(disk)

img = pg.ImageItem(border='k')
roiArea = pg.ImageItem(border='k')
v1.addItem(img)
v2.addItem(roiArea)

dc = DColor(xmin=-5, xmax=5, ymin=-5, ymax=5, samples=4000)

#Data da la función que se va a dibjuar en el plano

data = dc.plot(lambda z : z**2)
#data = dc.plot(lambda z : (z-1)/(z+1))

img.setImage(data)

def update(roi):
    roiArea.setImage(roi.getArrayRegion(data, img), levels=(0,1))

disk.sigRegionChanged.connect(update)


if __name__ == '__main__':
    pg.mkQApp().exec_()
