from PIL import Image, ImageDraw
import random
import math
 
def generate_voronoi_diagram(width, height, num_cells):
    image = Image.new("RGB", (width, height))
    putpixel = image.putpixel
    imgx, imgy = image.size
    nx = []
    ny = []
    nr = []
    ng = []
    nb = []
    for i in range(num_cells):
	nx.append(random.randrange(imgx))
	ny.append(random.randrange(imgy))
	nr.append(random.randrange(256))
	ng.append(random.randrange(256))
	nb.append(random.randrange(256))

    for y in range(imgy):
        for x in range(imgx):
            dmin = math.hypot(imgx-1, imgy-1)
            j = -1
	    for i in range(num_cells):
		d = math.hypot(nx[i]-x, ny[i]-y)
		if d < dmin:
		    dmin = d
		    j = i
		    putpixel((x, y), (nr[j], ng[j], nb[j]))

    draw = ImageDraw.Draw(image)
    for i in range(num_cells):
        print str(nx[i]) + "-" + str(ny[i])
        draw.ellipse((nx[i]-3, ny[i]-3, nx[i]+3, ny[i]+3), fill=0)
        
    image.save("VoronoiDiagram.png", "PNG")
    image.show()
 
generate_voronoi_diagram(500, 500, 25)
