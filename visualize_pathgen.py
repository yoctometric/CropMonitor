from pathgen import Workplace
from datetime import datetime

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np



if __name__ == "__main__":
    start = datetime.now()

    # test floodfill over payson park
    perimeter = [(43.679882271987395, -70.2693889874136), (43.68162231019378, -70.27141117491476),
            (43.68288076964761, -70.2725732966138), (43.68418710100531, -70.27068259077888),
            (43.68382568551194, -70.27015271143725), (43.684021633941214, -70.26986970769798),
            (43.68328573541723, -70.26871360731622), (43.6834468500671, -70.26843662493309),
            (43.68068606893511, -70.26390254378708), (43.68106927643034, -70.263511155637),
            (43.680760097846516, -70.26292106273382), (43.67989351836035, -70.26378813802968),
            (43.679122731145114, -70.26230086305941), (43.67906176450213, -70.26235505526479),
            (43.67959304316595, -70.2647214482337), (43.679366597097584, -70.26614850964243), 
            (43.67896160488272, -70.26780438258504)]
    workplace = Workplace(
        start_pos=(43.679882271987395, -70.2693889874136), 
        fov=(62.2, 48.8),   # the rpi cam 2 FOV 
        altitude=20, 
        perimeter=perimeter)

    rects = workplace.grid
    
    end = datetime.now()
    print(f"finished in {end - start}")

    per_x = [p[0] for p in perimeter]
    per_y = [p[1] for p in perimeter]


    rects_x = [rect.center[0] for rect in rects]
    rects_y = [rect.center[1] for rect in rects]


    plt.figure(figsize=(8, 8))
    plt.axis('equal')
    plt.fill(per_x, per_y)
    plt.scatter(rects_x, rects_y, c='red', linewidths=0.1)
    plt.title("Decomposed Payson Park")
    plt.show()